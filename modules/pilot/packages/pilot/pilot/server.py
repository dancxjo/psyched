"""HTTP server for the pilot cockpit."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from pathlib import Path
from typing import Any, List, MutableMapping, Optional

from aiohttp import web

from .config import ModuleDescriptor, discover_active_modules, load_host_config

_LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class PilotSettings:
    """Runtime configuration for the cockpit server."""

    host_config_path: Path
    frontend_root: Path
    listen_host: str = "0.0.0.0"
    listen_port: int = 8088

    def load_config(self) -> MutableMapping[str, Any]:
        """Return the parsed host configuration."""
        return load_host_config(self.host_config_path)

    def active_modules(self) -> List[ModuleDescriptor]:
        """Return descriptors for modules that should surface cockpit panels."""
        return discover_active_modules(self.load_config())


PILOT_SETTINGS_KEY: web.AppKey[PilotSettings] = web.AppKey("pilot_settings")
ROS_BRIDGE_KEY: web.AppKey[Any] = web.AppKey("ros_bridge")


def create_app(*, settings: PilotSettings, ros_bridge: Any) -> web.Application:
    """Create the aiohttp application that powers the cockpit."""

    app = web.Application()
    app[PILOT_SETTINGS_KEY] = settings
    app[ROS_BRIDGE_KEY] = ros_bridge

    app.router.add_get("/api/modules", _modules_handler)
    app.router.add_get("/api/topics/bridge", _topics_bridge_handler)

    app.router.add_get("/", _index_handler)
    app.router.add_get("/{tail:.*}", _static_handler)
    return app


async def _modules_handler(request: web.Request) -> web.Response:
    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
    modules = [
        {
            "name": module.name,
            "display_name": module.display_name,
            "slug": module.slug,
        }
        for module in settings.active_modules()
    ]
    return web.json_response({"modules": modules})


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
