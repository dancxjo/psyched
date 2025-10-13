"""HTTP server for the streamlined pilot frontend."""

from __future__ import annotations

import asyncio
import logging
import math
import socket
from dataclasses import dataclass
from datetime import date, datetime, time
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

from aiohttp import web

from .module_catalog import ModuleCatalog, ModuleInfo
from .config import (
    ModuleDescriptor,
    discover_active_modules,
    load_host_config,
    save_host_config,
    set_module_config,
)

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


@dataclass(slots=True)
class PilotSettings:
    """Runtime configuration for the cockpit server."""

    host_config_path: Path
    frontend_root: Path
    modules_root: Path
    repo_root: Optional[Path] = None
    listen_host: str = "0.0.0.0"
    listen_port: int = 8088
    bridge_mode: str = "rosbridge"
    rosbridge_uri: str = "ws://127.0.0.1:9090"
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


PILOT_SETTINGS_KEY: web.AppKey[PilotSettings] = web.AppKey("pilot_settings")
MODULE_CATALOG_KEY: web.AppKey[ModuleCatalog] = web.AppKey("module_catalog")


def create_app(*, settings: PilotSettings) -> web.Application:
    """Create the aiohttp application serving the pilot assets."""

    app = web.Application()
    app[PILOT_SETTINGS_KEY] = settings
    repo_root = settings.repo_root
    if repo_root is None:
        raise RuntimeError("PilotSettings.repo_root must be resolved before creating the app")

    catalog = ModuleCatalog(settings.modules_root)

    app[MODULE_CATALOG_KEY] = catalog
    # Fallback string keys for test harnesses that cannot import the AppKey constants.
    app["module_catalog"] = catalog

    app.router.add_get("/api/modules", _modules_handler)
    app.router.add_get("/api/module-config", _module_config_handler)
    app.router.add_put("/api/module-config/{module_name}", _module_config_update_handler)
    app.router.add_post("/api/ops/git-pull", _git_pull_handler)
    app.router.add_post("/api/ops/psh", _psh_operation_handler)

    app.router.add_get("/", _index_handler)
    app.router.add_get("/{tail:.*}", _static_handler)
    return app


async def _modules_handler(request: web.Request) -> web.Response:
    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
    catalog = _get_module_catalog(request.app)

    config = settings.load_config()
    active_descriptors = discover_active_modules(config)
    catalog.refresh()
    modules = [_module_payload(descriptor, catalog) for descriptor in active_descriptors]
    payload = {
        "modules": modules,
        "bridge": {
            "mode": settings.bridge_mode,
            "rosbridge_uri": settings.rosbridge_uri,
            "video_base": settings.video_base,
            "video_port": settings.video_port,
        },
        "host": _host_metadata(config),
    }
    return web.json_response(payload)


async def _module_config_handler(request: web.Request) -> web.Response:
    """Return host configuration data for modules."""

    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
    catalog = _get_module_catalog(request.app)
    catalog.refresh()
    host_config = settings.load_config()
    payload = {
        "modules": _module_config_entries(host_config, catalog),
    }
    return web.json_response(payload)


async def _module_config_update_handler(request: web.Request) -> web.Response:
    """Persist updates to a module's configuration block."""

    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
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

    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
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

    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
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
    """Resolve *tail* within the pilot frontend root, if possible."""

    parts = _normalize_parts(tail)
    candidate = frontend_root.joinpath(*parts) if parts else frontend_root
    if candidate.is_dir():
        candidate = candidate / "index.html"
    if candidate.exists() and candidate.is_file():
        return candidate
    return None


def _resolve_overlay_asset(modules_root: Path, tail: str) -> Optional[Path]:
    """Resolve *tail* inside a module's pilot overlay directory, if present."""

    parts = _normalize_parts(tail)
    if len(parts) < 2 or parts[0] != "modules":
        return None

    module_name = parts[1]
    remainder = parts[2:]
    base = modules_root / module_name / "pilot"
    if not base.exists():
        return None

    candidate = base.joinpath(*remainder) if remainder else base
    if candidate.is_dir():
        candidate = candidate / "index.html"
    if candidate.exists() and candidate.is_file():
        return candidate
    return None


async def _index_handler(request: web.Request) -> web.StreamResponse:
    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
    index_path = settings.frontend_root / "index.html"
    if not index_path.exists():
        raise web.HTTPNotFound(text="index.html not found in pilot frontend")
    return web.FileResponse(index_path)


async def _static_handler(request: web.Request) -> web.StreamResponse:
    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
    tail = request.match_info.get("tail", "")
    if tail.startswith("api/"):
        raise web.HTTPNotFound()

    frontend_target = _resolve_frontend_asset(settings.frontend_root, tail)
    if frontend_target:
        return web.FileResponse(frontend_target)

    overlay_target = _resolve_overlay_asset(settings.modules_root, tail)
    if overlay_target:
        return web.FileResponse(overlay_target)

    raise web.HTTPNotFound()


class CockpitServer:
    """Manage the aiohttp web server lifecycle."""

    def __init__(self, settings: PilotSettings) -> None:
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
            "has_pilot": False,
        }
        dashboard_url = None
    data["slug"] = descriptor.slug
    if dashboard_url:
        data["dashboard_url"] = dashboard_url
    return data


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
        raise RuntimeError("Module catalog not configured on pilot application")
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
    has_pilot = bool(catalog_info.pilot_assets) if catalog_info is not None else False

    return {
        "name": name,
        "display_name": display_name,
        "description": description,
        "slug": name.replace("_", "-"),
        "listed": listed,
        "active": descriptor is not None,
        "has_pilot": has_pilot,
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
