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


def _prefer_system_numpy() -> None:
    """Ensure Debian's numpy takes precedence over user site installs."""

    def parse_major(version: str) -> int:
        for chunk in version.split("."):
            if chunk.isdigit():
                return int(chunk)
        return 0

    system_paths = []
    for entry in list(sys.path):
        path = Path(entry)
        if not path.exists():
            continue
        if "dist-packages" in entry and (path / "numpy" / "__init__.py").exists():
            system_paths.append(entry)

    if system_paths:
        for dist_path in reversed(system_paths):
            try:
                sys.path.remove(dist_path)
            except ValueError:  # pragma: no cover - defensive
                continue
        sys.path[:0] = system_paths

    existing = sys.modules.get("numpy")
    if existing is None:
        return

    major = parse_major(str(getattr(existing, "__version__", "0")))
    if major < 2:
        return

    for key in [name for name in sys.modules if name == "numpy" or name.startswith("numpy.")]:
        sys.modules.pop(key, None)

    for entry in system_paths:
        package_root = Path(entry) / "numpy"
        init_file = package_root / "__init__.py"
        if not init_file.exists():
            continue
        spec = importlib.util.spec_from_file_location(
            "numpy",
            str(init_file),
            submodule_search_locations=[str(package_root)],
        )
        if not spec or not spec.loader:
            continue
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules["numpy"] = module
        return


_prefer_system_numpy()

