"""Cockpit ROS 2 control server."""

from importlib import import_module
from typing import TYPE_CHECKING

from .config import HostConfigError, ModuleDescriptor, discover_active_modules, load_host_config

if TYPE_CHECKING:  # pragma: no cover - import only for static analysis
    from .server import CockpitServer, CockpitSettings

__all__ = [
    "CockpitServer",
    "HostConfigError",
    "ModuleDescriptor",
    "CockpitSettings",
    "discover_active_modules",
    "load_host_config",
]


def __getattr__(name: str):
    if name in {"CockpitServer", "CockpitSettings"}:
        server = import_module(".server", __name__)
        value = getattr(server, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
