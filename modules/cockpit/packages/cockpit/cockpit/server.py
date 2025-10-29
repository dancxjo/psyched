"""HTTP server for the streamlined cockpit web interface."""

from __future__ import annotations

import asyncio
from contextlib import suppress
import json
import logging
import math
import re
import socket
from dataclasses import dataclass
from datetime import date, datetime, time, timezone
from pathlib import Path
from typing import (
    Any,
    Dict,
    Iterable,
    List,
    Mapping,
    MutableMapping,
    Optional,
    Sequence,
    Set,
)

from aiohttp import WSMsgType, web

from .actions import ActionError, ActionRegistry
from .actions.builtin import register_builtin_actions
from .module_api import register_module_api_actions
from .module_catalog import ModuleCatalog, ModuleInfo
from .config import (
    ModuleDescriptor,
    discover_active_modules,
    load_host_config,
    save_host_config,
    set_module_config,
)
from .ros import RosClient, TopicStream, TopicStreamError

_LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class CommandResult:
    """Outcome of a shell command executed by the cockpit server.

    The helper centralises command execution diagnostics so HTTP handlers can
    serialise the result consistently. ``stdout`` and ``stderr`` are decoded as
    UTF-8 with surrogate escaping to avoid crashes when the subprocess emits
    binary output.

    Example
    -------
    >>> result = CommandResult(["echo", "hello"], 0, "hello\n", "")
    >>> result.success
    True
    >>> result.to_payload()["command"]
    ['echo', 'hello']
    """

    command: List[str]
    returncode: int
    stdout: str
    stderr: str

    @property
    def success(self) -> bool:
        """Return ``True`` when the subprocess completed successfully."""

        return self.returncode == 0

    def to_payload(self) -> Dict[str, Any]:
        """Serialise the command outcome for JSON responses."""

        return {
            "command": self.command,
            "returncode": self.returncode,
            "stdout": self.stdout,
            "stderr": self.stderr,
            "success": self.success,
        }


_PSH_BULK_OPERATIONS: Dict[str, Sequence[str]] = {
    "module-setup": ("mod", "setup"),
    "module-up": ("mod", "up"),
    "module-down": ("mod", "down"),
}

_PSH_MODULE_SYSTEMD_ACTIONS: Dict[str, Sequence[str]] = {
    "setup": ("sys", "setup", "--module"),
    "teardown": ("sys", "teardown", "--module"),
    "enable": ("sys", "enable", "--module"),
    "disable": ("sys", "disable", "--module"),
    "up": ("sys", "up", "--module"),
    "down": ("sys", "down", "--module"),
    "debug": ("sys", "debug", "--module"),
}


@dataclass(slots=True)
class CockpitSettings:
    """Runtime configuration for the cockpit server."""

    host_config_path: Path
    frontend_root: Path
    modules_root: Path
    repo_root: Optional[Path] = None
    listen_host: str = "0.0.0.0"
    listen_port: int = 8088
    bridge_mode: str = "actions"
    video_base: Optional[str] = None
    video_port: Optional[int] = None

    def __post_init__(self) -> None:
        if self.repo_root is None:
            self.repo_root = self.modules_root.resolve().parent

    def load_config(self) -> MutableMapping[str, Any]:
        """Return the parsed host configuration."""
        return load_host_config(self.host_config_path)

    def active_modules(self) -> List[ModuleDescriptor]:
        """Return descriptors for modules that should surface cockpit panels."""
        return discover_active_modules(self.load_config())

class StreamManager:
    """Track active action streams and coordinate teardown."""

    def __init__(self, ros: RosClient) -> None:
        self._ros = ros
        self._streams: Dict[str, TopicStream] = {}

    def register(self, stream: TopicStream) -> str:
        stream_id = stream.metadata.id
        self._streams[stream_id] = stream
        return stream_id

    def get(self, stream_id: str) -> Optional[TopicStream]:
        return self._streams.get(stream_id)

    async def release(self, stream_id: str) -> None:
        self._streams.pop(stream_id, None)
        await self._ros.close_stream(stream_id)

    async def close_all(self) -> None:
        for stream_id in list(self._streams.keys()):
            await self.release(stream_id)


COCKPIT_SETTINGS_KEY: web.AppKey[CockpitSettings] = web.AppKey("cockpit_settings")
MODULE_CATALOG_KEY: web.AppKey[ModuleCatalog] = web.AppKey("module_catalog")
ACTION_REGISTRY_KEY: web.AppKey[ActionRegistry] = web.AppKey("action_registry")
ROS_CLIENT_KEY: web.AppKey[RosClient] = web.AppKey("ros_client")
STREAM_MANAGER_KEY: web.AppKey[StreamManager] = web.AppKey("stream_manager")


# Streamlined log handling -------------------------------------------------

#: Maximum number of log lines returned to the cockpit for a single request.
MODULE_LOG_LINE_LIMIT: int = 200

_MODULE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")


def create_app(*, settings: CockpitSettings) -> web.Application:
    """Create the aiohttp application serving the cockpit assets."""

    app = web.Application()
    app[COCKPIT_SETTINGS_KEY] = settings
    repo_root = settings.repo_root
    if repo_root is None:
        raise RuntimeError("CockpitSettings.repo_root must be resolved before creating the app")

    catalog = ModuleCatalog(settings.modules_root)

    app[MODULE_CATALOG_KEY] = catalog
    app["module_catalog"] = catalog  # Fallback for simple test harnesses.

    ros_client = RosClient()
    app[ROS_CLIENT_KEY] = ros_client

    registry = ActionRegistry()
    app[ACTION_REGISTRY_KEY] = registry
    app["action_registry"] = registry

    stream_manager = StreamManager(ros_client)
    app[STREAM_MANAGER_KEY] = stream_manager

    try:
        module_names = [info.name for info in catalog.list_modules()]
    except Exception:  # pragma: no cover - defensive guard for unexpected filesystem errors
        module_names = []
        _LOGGER.exception("Failed to enumerate modules while registering actions")
    register_builtin_actions(registry, ros=ros_client, modules=module_names)

    extra_action_roots: list[Path] = []
    if repo_root is not None:
        repo_modules = repo_root / "modules"
        if repo_modules.resolve() != settings.modules_root.resolve():
            extra_action_roots.append(repo_modules)

    register_module_api_actions(
        registry,
        modules_root=settings.modules_root,
        ros=ros_client,
        extra_roots=extra_action_roots,
    )

    async def _shutdown_resources(app: web.Application) -> None:
        await stream_manager.close_all()
        await ros_client.shutdown()

    app.on_cleanup.append(_shutdown_resources)

    app.router.add_get("/api/modules", _modules_handler)
    app.router.add_get("/api/modules/{module_name}/logs", _module_log_handler)
    app.router.add_delete("/api/modules/{module_name}/logs", _module_log_clear_handler)
    app.router.add_post(
        "/api/modules/{module_name}/systemd/{action}",
        _module_systemd_operation_handler,
    )
    app.router.add_get("/api/module-config", _module_config_handler)
    app.router.add_put("/api/module-config/{module_name}", _module_config_update_handler)
    app.router.add_post("/api/ops/git-pull", _git_pull_handler)
    app.router.add_post("/api/ops/psh", _psh_operation_handler)
    app.router.add_get("/api/actions", _actions_handler)
    app.router.add_post("/api/actions/{module_name}/{action_name}", _action_execute_handler)
    app.router.add_get("/api/streams/{stream_id}", _stream_handler)

    app.router.add_get("/", _index_handler)
    app.router.add_get("/{tail:.*}", _static_handler)
    return app


async def _modules_handler(request: web.Request) -> web.Response:
    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    catalog = _get_module_catalog(request.app)

    config = settings.load_config()
    active_descriptors = discover_active_modules(config)
    catalog.refresh()
    modules = await asyncio.gather(
        *(_module_payload_with_status(descriptor, catalog) for descriptor in active_descriptors)
    )
    payload = {
        "modules": modules,
        "bridge": {
            "mode": settings.bridge_mode,
            "actions": {
                "list": "/api/actions",
                "invoke": "/api/actions/{module}/{action}",
                "stream": "/api/streams/{stream}",
            },
            "video_base": settings.video_base,
            "video_port": settings.video_port,
        },
        "host": _host_metadata(config),
    }
    return web.json_response(payload)


async def _module_log_handler(request: web.Request) -> web.Response:
    """Return recent log lines for the requested module."""

    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    module_name = (request.match_info.get("module_name") or "").strip()
    if not module_name:
        raise web.HTTPBadRequest(text="Module name must be provided in the request path")
    if not _MODULE_NAME_PATTERN.fullmatch(module_name):
        raise web.HTTPBadRequest(text="Module name contains invalid characters")

    catalog = _get_module_catalog(request.app)
    catalog.refresh()
    try:
        module_info = catalog.get_module(module_name)
    except KeyError as exc:
        raise web.HTTPNotFound(text=f"Module not found: {module_name}") from exc

    repo_root = settings.repo_root
    if repo_root is None:
        raise web.HTTPInternalServerError(text="Cockpit repository root is not configured")

    lines, truncated, updated_at = _read_module_log(repo_root, module_name)
    payload: Dict[str, Any] = {
        "module": module_name,
        "display_name": module_info.display_name,
        "lines": lines,
        "truncated": truncated,
        "updated_at": updated_at.isoformat() if updated_at else None,
    }
    return web.json_response(payload)


async def _module_log_clear_handler(request: web.Request) -> web.Response:
    """Truncate a module's log file on demand."""

    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    module_name = (request.match_info.get("module_name") or "").strip()
    if not module_name:
        raise web.HTTPBadRequest(text="Module name must be provided in the request path")
    if not _MODULE_NAME_PATTERN.fullmatch(module_name):
        raise web.HTTPBadRequest(text="Module name contains invalid characters")

    catalog = _get_module_catalog(request.app)
    catalog.refresh()
    try:
        catalog.get_module(module_name)
    except KeyError as exc:
        raise web.HTTPNotFound(text=f"Module not found: {module_name}") from exc

    repo_root = settings.repo_root
    if repo_root is None:
        raise web.HTTPInternalServerError(text="Cockpit repository root is not configured")

    log_path = repo_root / "log" / "modules" / f"{module_name}.log"
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text("", encoding="utf-8")
    except OSError as exc:  # pragma: no cover - filesystem errors are environment specific
        _LOGGER.exception("Failed to clear module log for module %s", module_name)
        raise web.HTTPInternalServerError(text="Failed to clear module log") from exc

    return web.json_response({"module": module_name, "cleared": True})


async def _module_systemd_operation_handler(request: web.Request) -> web.Response:
    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    repo_root = settings.repo_root
    if repo_root is None:
        raise web.HTTPInternalServerError(text="Repository root is not configured")

    module_name = (request.match_info.get("module_name") or "").strip()
    if not module_name:
        raise web.HTTPBadRequest(text="Module name must be provided in the request path")
    if not _MODULE_NAME_PATTERN.fullmatch(module_name):
        raise web.HTTPBadRequest(text="Module name contains invalid characters")

    action = (request.match_info.get("action") or "").strip().lower()
    if not action:
        raise web.HTTPBadRequest(text="Action must be provided in the request path")

    command_prefix = _PSH_MODULE_SYSTEMD_ACTIONS.get(action)
    if command_prefix is None:
        raise web.HTTPBadRequest(text=f"Unsupported systemd action: {action}")

    catalog = _get_module_catalog(request.app)
    catalog.refresh()
    try:
        catalog.get_module(module_name)
    except KeyError as exc:
        raise web.HTTPNotFound(text=f"Module not found: {module_name}") from exc

    command = ["psh", *command_prefix, module_name]

    try:
        result = await _run_command(command=command, cwd=repo_root)
    except FileNotFoundError as exc:
        _LOGGER.error("psh binary not found while handling systemd action %s", action, exc_info=True)
        raise web.HTTPInternalServerError(text="psh binary is not available on this host") from exc
    except OSError as exc:  # pragma: no cover - defensive guard for unexpected OS errors
        _LOGGER.error("psh systemd action %s failed to spawn", action, exc_info=True)
        raise web.HTTPInternalServerError(text="Failed to execute systemd action") from exc

    status = await _module_systemd_status(module_name)
    payload = result.to_payload()
    payload.update({
        "module": module_name,
        "action": action,
        "status": status,
    })
    return web.json_response(payload)


async def _module_config_handler(request: web.Request) -> web.Response:
    """Return host configuration data for modules."""

    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    catalog = _get_module_catalog(request.app)
    catalog.refresh()
    host_config = settings.load_config()
    payload = {
        "modules": _module_config_entries(host_config, catalog),
    }
    return web.json_response(payload)


async def _module_config_update_handler(request: web.Request) -> web.Response:
    """Persist updates to a module's configuration block."""

    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    module_name = request.match_info.get("module_name", "").strip()
    if not module_name:
        raise web.HTTPBadRequest(text="Module name must be provided in the request path")

    try:
        payload = await request.json()
    except Exception as exc:  # pragma: no cover - aiohttp raises for bad JSON
        raise web.HTTPBadRequest(text="Request body must be valid JSON") from exc

    if not isinstance(payload, Mapping):
        raise web.HTTPBadRequest(text="Module configuration payload must be a JSON object")

    sanitized = _validate_module_payload(payload)
    config_data = settings.load_config()
    set_module_config(config_data, module_name, sanitized)
    save_host_config(settings.host_config_path, config_data)

    catalog = _get_module_catalog(request.app)
    catalog.refresh()
    entry = _module_config_entry(
        module_name,
        config_data,
        catalog,
    )
    return web.json_response(entry)


async def _git_pull_handler(request: web.Request) -> web.Response:
    """Run ``git pull --ff-only`` inside the workspace and expose the result."""

    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    repo_root = settings.repo_root
    if repo_root is None:
        raise web.HTTPInternalServerError(text="Repository root is not configured")

    try:
        result = await _run_command(command=["git", "pull", "--ff-only"], cwd=repo_root)
    except FileNotFoundError as exc:
        _LOGGER.error("git binary not found while handling git pull", exc_info=True)
        raise web.HTTPInternalServerError(text="git binary is not available on this host") from exc
    except OSError as exc:  # pragma: no cover - defensive guard for unexpected OS errors
        _LOGGER.error("git pull failed to spawn", exc_info=True)
        raise web.HTTPInternalServerError(text="Failed to execute git pull") from exc

    payload = result.to_payload()
    return web.json_response(payload)


async def _psh_operation_handler(request: web.Request) -> web.Response:
    """Execute approved bulk ``psh`` operations requested by the cockpit."""

    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    repo_root = settings.repo_root
    if repo_root is None:
        raise web.HTTPInternalServerError(text="Repository root is not configured")

    try:
        payload = await request.json()
    except Exception as exc:  # pragma: no cover - aiohttp raises for bad JSON
        raise web.HTTPBadRequest(text="Request body must be valid JSON") from exc

    if not isinstance(payload, Mapping):
        raise web.HTTPBadRequest(text="Request payload must be a JSON object")

    raw_operation = payload.get("operation")
    if not isinstance(raw_operation, str) or not raw_operation.strip():
        raise web.HTTPBadRequest(text="operation must be a non-empty string")

    operation = raw_operation.strip()
    command = _resolve_psh_command(operation)
    if command is None:
        raise web.HTTPBadRequest(text=f"Unsupported psh operation: {operation}")

    try:
        result = await _run_command(command=["psh", *command], cwd=repo_root)
    except FileNotFoundError as exc:
        _LOGGER.error("psh binary not found while handling %s", operation, exc_info=True)
        raise web.HTTPInternalServerError(text="psh binary is not available on this host") from exc
    except OSError as exc:  # pragma: no cover - defensive guard for unexpected OS errors
        _LOGGER.error("psh operation %s failed to spawn", operation, exc_info=True)
        raise web.HTTPInternalServerError(text="Failed to execute psh operation") from exc

    response_payload = result.to_payload()
    response_payload["operation"] = operation
    return web.json_response(response_payload)


async def _actions_handler(request: web.Request) -> web.Response:
    """Return the catalog of available cockpit actions."""

    registry = _get_action_registry(request.app)
    payload = registry.to_payload()
    return web.json_response(payload)


async def _action_execute_handler(request: web.Request) -> web.Response:
    """Execute a module action and optionally provision a streaming bridge."""

    module_name = (request.match_info.get("module_name") or "").strip()
    action_name = (request.match_info.get("action_name") or "").strip()
    if not module_name or not action_name:
        raise web.HTTPBadRequest(text="Module name and action name must be provided in the request path")

    try:
        payload = await request.json()
    except Exception as exc:  # pragma: no cover - aiohttp raises for bad JSON
        raise web.HTTPBadRequest(text="Request body must be valid JSON") from exc

    if payload is None:
        payload = {}
    if not isinstance(payload, Mapping):
        raise web.HTTPBadRequest(text="Action payload must be a JSON object")

    arguments = payload.get("arguments", {})
    if arguments is None:
        arguments = {}
    if not isinstance(arguments, Mapping):
        raise web.HTTPBadRequest(text="arguments must be expressed as a JSON object")

    registry = _get_action_registry(request.app)
    try:
        result = await registry.execute(
            module_name,
            action_name,
            app=request.app,
            arguments=arguments,
            request=request,
        )
    except KeyError as exc:
        raise web.HTTPNotFound(text=str(exc)) from exc
    except ActionError as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc

    response_payload: Dict[str, Any] = {
        "module": module_name,
        "action": action_name,
    }
    if result.payload is not None:
        response_payload["result"] = dict(result.payload)

    if result.streaming:
        stream = result.stream
        if not isinstance(stream, TopicStream):
            raise web.HTTPInternalServerError(text="Streaming actions must return a TopicStream")
        stream_manager = _get_stream_manager(request.app)
        stream_id = stream_manager.register(stream)
        response_payload["stream"] = stream.metadata.to_dict()
        response_payload["stream"]["id"] = stream_id

    return web.json_response(response_payload)


async def _stream_handler(request: web.Request) -> web.StreamResponse:
    """Upgrade the connection to a websocket that forwards stream events."""

    stream_id = (request.match_info.get("stream_id") or "").strip()
    if not stream_id:
        raise web.HTTPBadRequest(text="Stream identifier must be provided in the request path")

    stream_manager = _get_stream_manager(request.app)
    stream = stream_manager.get(stream_id)
    if stream is None:
        raise web.HTTPNotFound(text=f"Stream not found: {stream_id}")

    ws = web.WebSocketResponse(heartbeat=30.0)
    await ws.prepare(request)

    await ws.send_json({"event": "status", "data": {"state": "connected"}})

    forward_task = asyncio.create_task(_forward_stream_events(ws, stream))
    receive_task = asyncio.create_task(_receive_stream_messages(ws, stream))

    try:
        await asyncio.wait(
            [forward_task, receive_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
    finally:
        for task in (forward_task, receive_task):
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
        await stream_manager.release(stream_id)
        if not ws.closed:
            await ws.close()

    return ws


async def _forward_stream_events(ws: web.WebSocketResponse, stream: TopicStream) -> None:
    try:
        while True:
            event = await stream.next_event()
            await ws.send_json(event)
            if event.get("event") == "status" and event.get("data", {}).get("state") == "closed":
                break
    except asyncio.CancelledError:  # pragma: no cover - cancellation is expected during shutdown
        pass
    except ConnectionResetError:  # pragma: no cover - network interruptions
        _LOGGER.debug("Websocket connection reset while forwarding stream %s", stream.metadata.id)
    except Exception:  # pragma: no cover - unexpected websocket/serialization issues
        _LOGGER.exception("Failed to forward events for stream %s", stream.metadata.id)


async def _receive_stream_messages(ws: web.WebSocketResponse, stream: TopicStream) -> None:
    try:
        async for message in ws:
            if message.type == WSMsgType.TEXT:
                try:
                    payload = json.loads(message.data)
                except json.JSONDecodeError:
                    await ws.send_json({"event": "error", "data": {"message": "Invalid JSON payload"}})
                    continue

                event_type = payload.get("event")
                if event_type == "publish":
                    body = payload.get("data", {})
                    if not isinstance(body, Mapping):
                        await ws.send_json({"event": "error", "data": {"message": "Publish payload must be an object"}})
                        continue
                    try:
                        await stream.publish(body)
                    except TopicStreamError as exc:
                        await ws.send_json({"event": "error", "data": {"message": str(exc)}})
                elif event_type == "close":
                    await ws.close()
                    break
                else:
                    await ws.send_json({"event": "error", "data": {"message": f"Unsupported event: {event_type}"}})
            elif message.type in {WSMsgType.CLOSED, WSMsgType.CLOSE}:
                break
            elif message.type == WSMsgType.ERROR:
                _LOGGER.debug(
                    "Websocket for stream %s closed with error: %s",
                    stream.metadata.id,
                    ws.exception(),
                )
                break
    except asyncio.CancelledError:  # pragma: no cover - expected during shutdown
        pass
    except Exception:  # pragma: no cover - defensive guard
        _LOGGER.exception("Error while handling inbound websocket messages for stream %s", stream.metadata.id)

def _resolve_psh_command(operation: str) -> Optional[List[str]]:
    """Return the command tuple for a whitelisted ``psh`` operation."""

    command = _PSH_BULK_OPERATIONS.get(operation)
    if command is None:
        return None
    return list(command)


async def _run_command(*, command: Sequence[str], cwd: Path) -> CommandResult:
    """Execute *command* inside *cwd* and capture stdout/stderr."""

    process = await asyncio.create_subprocess_exec(
        *command,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
        cwd=str(cwd),
    )
    stdout_bytes, stderr_bytes = await process.communicate()
    stdout_text = stdout_bytes.decode("utf-8", "replace")
    stderr_text = stderr_bytes.decode("utf-8", "replace")
    return CommandResult(
        command=list(command),
        returncode=process.returncode,
        stdout=stdout_text,
        stderr=stderr_text,
    )


def _normalize_parts(tail: str) -> List[str]:
    """Return a list of safe path segments extracted from *tail*."""

    return [part for part in tail.split("/") if part and part not in {".", ".."}]


def _resolve_frontend_asset(frontend_root: Path, tail: str) -> Optional[Path]:
    """Resolve *tail* within the cockpit frontend root, if possible."""

    parts = _normalize_parts(tail)
    candidate = frontend_root.joinpath(*parts) if parts else frontend_root
    if candidate.is_dir():
        candidate = candidate / "index.html"
    if candidate.exists() and candidate.is_file():
        return candidate
    return None


def _resolve_overlay_asset(
    modules_root: Path,
    tail: str,
    *,
    repo_root: Optional[Path] = None,
) -> Optional[Path]:
    """Resolve *tail* inside a module's cockpit overlay directory, if present.

    The cockpit normally serves static assets from the synchronised module
    overlays under ``modules_root``. When new dashboard files land in the repo
    but the operator has not re-run ``psh mod setup`` yet, those files only
    exist under ``repo_root/modules``. In that case we fall back to the repo so
    fresh assets are available immediately while still preferring the deployed
    overlay tree when it contains the requested file.
    """

    parts = _normalize_parts(tail)
    if len(parts) < 2 or parts[0] != "modules":
        return None

    module_name = parts[1]
    remainder = parts[2:]

    search_roots: List[Path] = []

    overlay_base = modules_root / module_name / "cockpit"
    if overlay_base.exists():
        search_roots.append(overlay_base)

    if repo_root is not None:
        repo_base = repo_root / "modules" / module_name / "cockpit"
        # Avoid double entries when repo and overlay share the same root.
        if repo_base.exists():
            resolved_roots = {root.resolve() for root in search_roots}
            if repo_base.resolve() not in resolved_roots:
                search_roots.append(repo_base)

    for base in search_roots:
        candidate = base.joinpath(*remainder) if remainder else base
        if candidate.is_dir():
            candidate = candidate / "index.html"
        if candidate.exists() and candidate.is_file():
            return candidate

    return None


async def _index_handler(request: web.Request) -> web.StreamResponse:
    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    index_path = settings.frontend_root / "index.html"
    if not index_path.exists():
        raise web.HTTPNotFound(text="index.html not found in cockpit frontend")
    return web.FileResponse(index_path)


async def _static_handler(request: web.Request) -> web.StreamResponse:
    settings: CockpitSettings = request.app[COCKPIT_SETTINGS_KEY]
    tail = request.match_info.get("tail", "")
    if tail.startswith("api/"):
        raise web.HTTPNotFound()

    frontend_target = _resolve_frontend_asset(settings.frontend_root, tail)
    if frontend_target:
        return web.FileResponse(frontend_target)

    overlay_target = _resolve_overlay_asset(
        settings.modules_root,
        tail,
        repo_root=settings.repo_root,
    )
    if overlay_target:
        return web.FileResponse(overlay_target)

    raise web.HTTPNotFound()


class CockpitServer:
    """Manage the aiohttp web server lifecycle."""

    def __init__(self, settings: CockpitSettings) -> None:
        self._settings = settings
        self._app = create_app(settings=settings)
        self._runner: Optional[web.AppRunner] = None
        self._site: Optional[web.TCPSite] = None

    @property
    def app(self) -> web.Application:
        return self._app

    async def start(self) -> None:
        """Start the HTTP server."""

        if self._runner is not None:
            return

        self._runner = web.AppRunner(self._app)
        await self._runner.setup()
        self._site = web.TCPSite(
            self._runner,
            host=self._settings.listen_host,
            port=self._settings.listen_port,
        )
        await self._site.start()
        _LOGGER.info(
            "cockpit server listening on http://%s:%s",
            self._settings.listen_host,
            self._settings.listen_port,
        )

    async def stop(self) -> None:
        """Stop the HTTP server and release resources."""

        if self._site is not None:
            await self._site.stop()
            self._site = None
        if self._runner is not None:
            await self._runner.cleanup()
            self._runner = None

    async def __aenter__(self) -> "CockpitServer":
        await self.start()
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        await self.stop()


def _read_module_log(
    repo_root: Path, module_name: str
) -> tuple[List[str], bool, Optional[datetime]]:
    """Return a bounded snapshot of a module's log file.

    Parameters
    ----------
    repo_root:
        Base directory for the repository. The helper inspects the
        ``log/modules`` folder beneath this location.
    module_name:
        Name of the module whose log should be read.

    Returns
    -------
    tuple[list[str], bool, Optional[datetime]]
        A tuple containing the collected lines (newest first), a flag that
        indicates whether the log was truncated to :data:`MODULE_LOG_LINE_LIMIT`,
        and the timestamp of the last modification if available.

    Examples
    --------
    >>> repo = Path("/tmp/psyched-test")
    >>> repo.mkdir(exist_ok=True)
    >>> (repo / "log" / "modules").mkdir(parents=True, exist_ok=True)
    >>> lines, truncated, updated_at = _read_module_log(repo, "nav")
    >>> lines
    []
    >>> truncated
    False
    >>> updated_at is None
    True
    """

    log_path = repo_root / "log" / "modules" / f"{module_name}.log"
    if not log_path.exists():
        return [], False, None

    try:
        text = log_path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        text = log_path.read_text(encoding="utf-8", errors="replace")

    all_lines = text.splitlines()
    truncated = len(all_lines) > MODULE_LOG_LINE_LIMIT
    lines = all_lines[-MODULE_LOG_LINE_LIMIT:] if truncated else all_lines
    # Reverse to show newest first
    lines = list(reversed(lines))

    try:
        stat = log_path.stat()
    except FileNotFoundError:
        updated_at: Optional[datetime] = None
    else:
        updated_at = datetime.fromtimestamp(stat.st_mtime, tz=timezone.utc)

    return lines, truncated, updated_at


def _get_action_registry(app: web.Application) -> ActionRegistry:
    registry = app.get(ACTION_REGISTRY_KEY)
    if registry is None:
        registry = app.get("action_registry")
    if registry is None:
        raise RuntimeError("Action registry has not been initialised")
    return registry


def _get_stream_manager(app: web.Application) -> StreamManager:
    manager = app.get(STREAM_MANAGER_KEY)
    if manager is None:
        raise RuntimeError("Stream manager has not been initialised")
    return manager


def _get_ros_client(app: web.Application) -> RosClient:
    client = app.get(ROS_CLIENT_KEY)
    if client is None:
        raise RuntimeError("ROS client has not been initialised")
    return client


def _module_payload(descriptor: ModuleDescriptor, catalog: ModuleCatalog) -> Dict[str, Any]:
    try:
        info = catalog.get_module(descriptor.name)
    except KeyError:
        info = None
    if info is not None:
        data = info.to_dict()
        data["display_name"] = descriptor.display_name or data.get("display_name") or descriptor.name
        dashboard_url = info.dashboard_url
    else:
        data = {
            "name": descriptor.name,
            "display_name": descriptor.display_name or descriptor.name,
            "description": "",
            "has_cockpit": False,
        }
        dashboard_url = None
    data["slug"] = descriptor.slug
    if dashboard_url:
        data["dashboard_url"] = dashboard_url
    return data


async def _module_payload_with_status(
    descriptor: ModuleDescriptor,
    catalog: ModuleCatalog,
) -> Dict[str, Any]:
    payload = _module_payload(descriptor, catalog)
    payload["systemd"] = await _module_systemd_status(descriptor.name)
    return payload


def _systemd_unit_name(module_name: str) -> str:
    normalized = module_name.strip()
    return f"psh-module-{normalized}.service"


async def _module_systemd_status(module_name: str) -> Dict[str, Any]:
    unit = _systemd_unit_name(module_name)
    try:
        process = await asyncio.create_subprocess_exec(
            "systemctl",
            "show",
            "--property=ActiveState,SubState,UnitFileState,LoadState",
            unit,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
    except FileNotFoundError:
        return {
            "supported": False,
            "unit": unit,
            "active": False,
            "enabled": False,
            "exists": False,
            "load_state": "missing",
            "active_state": "unknown",
            "sub_state": "unknown",
            "unit_file_state": "absent",
            "message": "systemctl binary is not available on this host",
        }

    stdout_bytes, stderr_bytes = await process.communicate()
    stdout_text = stdout_bytes.decode("utf-8", "replace")
    metadata: Dict[str, str] = {}
    for line in stdout_text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        metadata[key.strip()] = value.strip()

    load_state = metadata.get("LoadState", "")
    active_state = metadata.get("ActiveState", "")
    sub_state = metadata.get("SubState", "")
    unit_file_state = metadata.get("UnitFileState", "")

    exists = load_state not in {"", "not-found", "bad-setting", "error"}
    enabled = unit_file_state in {"enabled", "linked", "static"}
    active = active_state in {"active", "activating", "reloading"}

    payload: Dict[str, Any] = {
        "supported": True,
        "unit": unit,
        "load_state": load_state or "unknown",
        "active_state": active_state or "inactive",
        "sub_state": sub_state or "dead",
        "unit_file_state": unit_file_state or "disabled",
        "exists": exists,
        "enabled": enabled if exists else False,
        "active": active if exists else False,
    }

    if process.returncode != 0 and not exists:
        payload["message"] = "Systemd unit not found"
        payload["returncode"] = process.returncode
    else:
        stderr_text = stderr_bytes.decode("utf-8", "replace").strip()
        if stderr_text:
            payload["message"] = stderr_text
            payload["returncode"] = process.returncode

    return payload


def _host_metadata(config: Mapping[str, Any]) -> Dict[str, str]:
    """Return host metadata that helps dashboards connect to telemetry feeds.

    The helper prefers the ``[host].name`` entry from the TOML configuration
    and falls back to :func:`socket.gethostname` when the value is absent.

    Examples
    --------
    >>> _host_metadata({"host": {"name": "pete.local"}})["shortname"]
    'pete'
    >>> _host_metadata({"host": {}})["name"]  # doctest: +SKIP
    'pete'
    """

    host_section = config.get("host")
    raw_name = ""
    if isinstance(host_section, Mapping):
        name_value = host_section.get("name")
        if isinstance(name_value, str):
            raw_name = name_value.strip()
        elif name_value is not None:
            raw_name = str(name_value).strip()

    shortname = raw_name.split(".")[0] if raw_name else ""
    if not shortname:
        fallback = socket.gethostname()
        shortname = fallback.split(".")[0] if fallback else "host"

    if not shortname:
        shortname = "host"

    name = raw_name or shortname
    return {"name": name, "shortname": shortname}


def _get_module_catalog(app: web.Application) -> ModuleCatalog:
    catalog = app.get(MODULE_CATALOG_KEY)
    if catalog is None:
        catalog = app.get("module_catalog")
    if catalog is None:
        raise RuntimeError("Module catalog not configured on cockpit application")
    return catalog


def _module_config_entries(
    config: Mapping[str, Any],
    catalog: ModuleCatalog,
) -> List[Dict[str, Any]]:
    active = {descriptor.name: descriptor for descriptor in discover_active_modules(config)}
    module_configs = _extract_module_configs(config)
    host_modules = _extract_host_modules(config)
    host_set = set(host_modules)
    catalog_map = {info.name: info for info in catalog.list_modules()}

    ordered_names = list(host_modules)
    remaining: Set[str] = set(module_configs)
    remaining.difference_update(host_set)
    for name in sorted(remaining):
        if name not in ordered_names:
            ordered_names.append(name)

    entries: List[Dict[str, Any]] = []
    for name in ordered_names:
        entries.append(
            _module_config_entry_from_parts(
                name=name,
                raw_config=module_configs.get(name, {}),
                descriptor=active.get(name),
                catalog_info=catalog_map.get(name),
                listed=name in host_set,
            )
        )
    return entries


def _module_config_entry(
    module_name: str,
    config: Mapping[str, Any],
    catalog: ModuleCatalog,
) -> Dict[str, Any]:
    module_configs = _extract_module_configs(config)
    host_modules = _extract_host_modules(config)
    host_set = set(host_modules)
    active = {descriptor.name: descriptor for descriptor in discover_active_modules(config)}
    try:
        catalog_info = catalog.get_module(module_name)
    except KeyError:
        catalog_info = None

    return _module_config_entry_from_parts(
        name=module_name,
        raw_config=module_configs.get(module_name, {}),
        descriptor=active.get(module_name),
        catalog_info=catalog_info,
        listed=module_name in host_set,
    )


def _module_config_entry_from_parts(
    *,
    name: str,
    raw_config: Mapping[str, Any],
    descriptor: ModuleDescriptor | None,
    catalog_info: ModuleInfo | None,
    listed: bool,
) -> Dict[str, Any]:
    display_name = (
        descriptor.display_name
        if descriptor is not None
        else catalog_info.display_name
        if catalog_info is not None
        else name.replace("_", " ").title()
    )
    description = catalog_info.description if catalog_info is not None else ""
    dashboard_url = catalog_info.dashboard_url if catalog_info is not None else None
    has_cockpit = bool(catalog_info.cockpit_assets) if catalog_info is not None else False

    return {
        "name": name,
        "display_name": display_name,
        "description": description,
        "slug": name.replace("_", "-"),
        "listed": listed,
        "active": descriptor is not None,
        "has_cockpit": has_cockpit,
        "dashboard_url": dashboard_url,
        "config": _json_compatible(raw_config),
    }


def _extract_module_configs(config: Mapping[str, Any]) -> Dict[str, Mapping[str, Any]]:
    result: Dict[str, Mapping[str, Any]] = {}
    config_section = config.get("config")
    if not isinstance(config_section, Mapping):
        return result
    modules_section = config_section.get("mod")
    if not isinstance(modules_section, Mapping):
        return result
    for raw_name, raw_value in modules_section.items():
        name = str(raw_name)
        result[name] = _coerce_mapping(raw_value)
    return result


def _extract_host_modules(config: Mapping[str, Any]) -> List[str]:
    host_section = config.get("host")
    if not isinstance(host_section, Mapping):
        return []
    modules = host_section.get("modules")
    if modules is None:
        return []
    if isinstance(modules, (str, bytes)):
        text = modules.strip()
        return [text] if text else []
    if isinstance(modules, Iterable):
        result: List[str] = []
        for item in modules:
            text = str(item).strip()
            if text and text not in result:
                result.append(text)
        return result
    return []


def _coerce_mapping(value: Any) -> Mapping[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    if isinstance(value, bool):
        return {"launch": value}
    if value is None:
        return {}
    return {}


def _json_compatible(value: Any) -> Any:
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return str(value)
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
    if isinstance(value, (datetime, date, time)):
        return value.isoformat()
    return str(value)


def _validate_module_payload(value: Any, path: str | None = None) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise web.HTTPBadRequest(text="Module configuration payload must be a JSON object")
    return {
        str(key): _validate_value(item, _extend_path(path, str(key)))
        for key, item in value.items()
    }


def _validate_value(value: Any, path: str) -> Any:
    if isinstance(value, Mapping):
        return {
            str(key): _validate_value(item, _extend_path(path, str(key)))
            for key, item in value.items()
        }
    if isinstance(value, list):
        return [
            _validate_value(item, _extend_path(path, f"[{index}]"))
            for index, item in enumerate(value)
        ]
    if isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            raise web.HTTPBadRequest(
                text=f"Unsupported float value at {path or 'root'}"
            )
        return value
    if value is None:
        raise web.HTTPBadRequest(
            text=f"Null values are not supported in module configs ({path or 'root'})"
        )
    raise web.HTTPBadRequest(
        text=f"Unsupported value type {type(value).__name__} at {path or 'root'}"
    )


def _extend_path(root: str | None, segment: str) -> str:
    if not root:
        return segment
    if segment.startswith("["):
        return f"{root}{segment}"
    return f"{root}.{segment}"
