"""Pilot backend package entrypoint."""

from __future__ import annotations

from typing import TYPE_CHECKING

from .module_catalog import ModuleCatalog, ModuleInfo, ModuleTopic, ModuleCommands
from .qos import QosConfig
from .voice_config import VoiceConfigStore

if TYPE_CHECKING:  # pragma: no cover - used only for static analysis
    from .app import PilotApplication, create_app

__all__ = [
    "ModuleCatalog",
    "ModuleInfo",
    "ModuleTopic",
    "ModuleCommands",
    "QosConfig",
    "VoiceConfigStore",
    "create_app",
    "PilotApplication",
]


def __getattr__(name: str):  # noqa: D401 - module attribute loader
    """Lazily expose application factories to avoid optional dependency churn."""

    if name in {"create_app", "PilotApplication"}:
        from .app import PilotApplication as _PilotApplication, create_app as _create_app

        return {"create_app": _create_app, "PilotApplication": _PilotApplication}[name]
    raise AttributeError(f"module 'pilot' has no attribute {name!r}")
