"""Minimal typing_extensions shim for Python 3.12 environments.

This module provides the subset of the typing_extensions API required by rclpy
when the real package is unavailable inside the ROS virtualenv. It intentionally
keeps the surface area small; add more symbols here if future upgrades demand
it.
"""
from __future__ import annotations

from typing import TypeAlias as _TypeAlias
from typing import Unpack as _Unpack

try:  # Python 3.12 ships typing.deprecated in typing_extensions but not typing
    from typing import deprecated as _deprecated  # type: ignore[attr-defined]
except ImportError:  # pragma: no cover - fallback for current Python versions
    def deprecated(*_args, **_kwargs):  # type: ignore[override]
        def decorator(obj):
            return obj

        return decorator
else:  # pragma: no cover - if typing.deprecated becomes available later
    deprecated = _deprecated

TypeAlias = _TypeAlias
Unpack = _Unpack

__all__ = ["TypeAlias", "Unpack", "deprecated"]
