"""HTTP server for the streamlined pilot frontend."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, MutableMapping, Optional

from aiohttp import web

from .module_catalog import ModuleCatalog
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

    app.router.add_get("/", _index_handler)
    app.router.add_get("/{tail:.*}", _static_handler)
    return app


async def _modules_handler(request: web.Request) -> web.Response:
    settings: PilotSettings = request.app[PILOT_SETTINGS_KEY]
    catalog = _get_module_catalog(request.app)

    active_descriptors = settings.active_modules()
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
    }
    return web.json_response(payload)


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
    return data


def _get_module_catalog(app: web.Application) -> ModuleCatalog:
    catalog = app.get(MODULE_CATALOG_KEY)
    if catalog is None:
        catalog = app.get("module_catalog")
    if catalog is None:
        raise RuntimeError("Module catalog not configured on pilot application")
    return catalog
