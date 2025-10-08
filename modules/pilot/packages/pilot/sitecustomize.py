"""Interpreter customisations for the Pilot cockpit backend.

This module is imported automatically by Python (when discoverable on
``sys.path``) and allows us to provide compatibility shims before the ROS
runtime starts loading third-party packages.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _ensure_typing_extensions() -> None:
    """Expose a minimal typing_extensions module when the real package is absent."""

    try:
        import typing_extensions  # noqa: F401  # pragma: no cover
    except ModuleNotFoundError:
        vendor_path = (
            Path(__file__).parent / "pilot" / "_vendor" / "typing_extensions.py"
        )
        spec = importlib.util.spec_from_file_location(
            "typing_extensions", vendor_path
        )
        if not spec or not spec.loader:  # pragma: no cover - defensive
            raise RuntimeError(
                f"Failed to load vendored typing_extensions from {vendor_path}"
            )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules.setdefault("typing_extensions", module)


_ensure_typing_extensions()

