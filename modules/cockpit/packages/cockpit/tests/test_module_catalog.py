"""Expectations for module catalog metadata exposed to the cockpit UI."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[5]
PACKAGE_DIR = REPO_ROOT / "modules" / "cockpit" / "packages" / "cockpit" / "cockpit"

spec = importlib.util.spec_from_file_location(
    "cockpit.module_catalog",
    PACKAGE_DIR / "module_catalog.py",
)
module_catalog = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module_catalog
assert spec.loader is not None  # safeguard for type checkers
spec.loader.exec_module(module_catalog)

ModuleCatalog = module_catalog.ModuleCatalog


def test_nav_module_reports_cockpit_dashboard() -> None:
    """The navigation module should surface cockpit assets for operators."""

    modules_root = REPO_ROOT / "modules"
    catalog = ModuleCatalog(modules_root)

    nav_info = catalog.get_module("nav")

    assert nav_info.cockpit_assets is True
    assert nav_info.dashboard_url.endswith("/modules/nav/")


def test_cockpit_module_surfaces_self_dashboard() -> None:
    """Cockpit should expose a cockpit dashboard for its own orchestration tools."""

    modules_root = REPO_ROOT / "modules"
    catalog = ModuleCatalog(modules_root)

    cockpit_info = catalog.get_module("cockpit")

    assert cockpit_info.cockpit_assets is True
    assert cockpit_info.dashboard_url.endswith("/modules/cockpit/")
