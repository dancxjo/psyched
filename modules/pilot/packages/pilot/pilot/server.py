"""HTTP server for the pilot cockpit."""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, MutableMapping, Optional, Sequence

from aiohttp import web

from .commands import CommandExecutor
from .module_catalog import ModuleCatalog, ModuleInfo
from .config import ModuleDescriptor, discover_active_modules, load_host_config

_LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class PilotSettings:
    """Runtime configuration for the cockpit server."""

    host_config_path: Path
    frontend_root: Path
    modules_root: Path
    repo_root: Optional[Path] = None
    listen_host: str = "0.0.0.0"
    listen_port: int = 8088

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
COMMAND_EXECUTOR_KEY: web.AppKey[CommandExecutor] = web.AppKey("command_executor")
ROS_BRIDGE_KEY: web.AppKey[Any] = web.AppKey("ros_bridge")


def create_app(*, settings: PilotSettings, ros_bridge: Any) -> web.Application:
    """Create the aiohttp application that powers the cockpit."""

    app = web.Application()
    app[PILOT_SETTINGS_KEY] = settings
    app[ROS_BRIDGE_KEY] = ros_bridge
    repo_root = settings.repo_root
    if repo_root is None:
        raise RuntimeError("PilotSettings.repo_root must be resolved before creating the app")

    catalog = ModuleCatalog(settings.modules_root)
    executor = CommandExecutor(repo_root=repo_root)

    app[MODULE_CATALOG_KEY] = catalog
    app[COMMAND_EXECUTOR_KEY] = executor
    # Fallback string keys for test harnesses that cannot import the AppKey constants.
    app["module_catalog"] = catalog
    app["command_executor"] = executor

    app.router.add_get("/api/modules", _modules_handler)
    app.router.add_get("/api/topics/bridge", _topics_bridge_handler)
    app.router.add_get("/ws", _topics_bridge_handler)
    app.router.add_post("/api/modules/{module}/commands", _modules_command_handler)

    app.router.add_get("/", _index_handler)
    app.router.add_get("/{tail:.*}", _static_handler)
    return app


async def _modules_handler(request: web.Request) -> web.Response:
    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
    catalog = _get_module_catalog(request.app)

    active_descriptors = settings.active_modules()
    catalog.refresh()

    if active_descriptors:
        modules = [_module_payload(descriptor, catalog) for descriptor in active_descriptors]
    else:
        modules = [
            _module_payload_from_info(info)
            for info in catalog.list_modules()
        ]
    return web.json_response({"modules": modules})


async def _modules_command_handler(request: web.Request) -> web.Response:
    module = request.match_info.get("module", "").strip()
    if not module:
        raise web.HTTPBadRequest(text="Module name required")

    try:
        payload = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text=f"Invalid JSON payload: {exc}") from exc

    if not isinstance(payload, MutableMapping):
        raise web.HTTPBadRequest(text="Command payload must be a JSON object")

    scope = str(payload.get("scope", "")).strip()
    command = str(payload.get("command", "")).strip()
    if not scope or not command:
        raise web.HTTPBadRequest(text="Both 'scope' and 'command' fields are required")

    args = payload.get("args") or []
    if isinstance(args, Sequence) and not isinstance(args, (str, bytes, bytearray)):
        coerced_args = [str(item) for item in args]
    else:
        raise web.HTTPBadRequest(text="'args' must be an array of arguments")

    catalog = _get_module_catalog(request.app)
    executor = _get_command_executor(request.app)

    allowed_commands: Iterable[str] | None = None
    try:
        metadata = catalog.get_module(module)
    except KeyError:
        metadata = None

    if metadata is not None:
        if scope.lower() == "mod":
            allowed_commands = metadata.commands.mod
        elif scope.lower() == "sys":
            allowed_commands = metadata.commands.system
        if allowed_commands is not None and command not in allowed_commands:
            raise web.HTTPBadRequest(text=f"Unsupported command '{command}' for module '{module}'")

    result = await executor.run(scope, module, command, coerced_args)
    return web.json_response({"result": result})


async def _topics_bridge_handler(request: web.Request) -> web.StreamResponse:
    ros_bridge = request.app[ROS_BRIDGE_KEY]
    if ros_bridge is None:
        return web.Response(status=501, text="ROS bridge unavailable")
    handler = getattr(ros_bridge, "handle_websocket", None)
    if handler is None:
        raise web.HTTPInternalServerError(text="ROS bridge lacks websocket handler")
    return await handler(request)


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

    target = settings.frontend_root / tail
    if target.is_dir():
        target = target / "index.html"

    if not target.exists() or not target.is_file():
        raise web.HTTPNotFound()

    return web.FileResponse(target)


class CockpitServer:
    """Manage the aiohttp web server lifecycle."""

    def __init__(self, settings: PilotSettings, ros_bridge: Any) -> None:
        self._settings = settings
        self._ros_bridge = ros_bridge
        self._app = create_app(settings=settings, ros_bridge=ros_bridge)
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
    else:
        data = {
            "name": descriptor.name,
            "display_name": descriptor.display_name or descriptor.name,
            "description": "",
            "regimes": [],
            "topics": [],
            "commands": {"mod": [], "system": []},
        }
    data["slug"] = descriptor.slug
    return data


def _module_payload_from_info(info: ModuleInfo) -> Dict[str, Any]:
    data = info.to_dict()
    data["slug"] = info.name.replace("_", "-")
    return data


def _get_module_catalog(app: web.Application) -> ModuleCatalog:
    catalog = app.get(MODULE_CATALOG_KEY)
    if catalog is None:
        catalog = app.get("module_catalog")
    if catalog is None:
        raise RuntimeError("Module catalog not configured on pilot application")
    return catalog


def _get_command_executor(app: web.Application) -> CommandExecutor:
    executor = app.get("command_executor")
    if executor is None:
        executor = app.get(COMMAND_EXECUTOR_KEY)
    if executor is None:
        raise RuntimeError("Command executor not configured on pilot application")
    return executor
