"""Behavioral tests for host configuration discovery utilities."""

from __future__ import annotations

from pathlib import Path
from textwrap import dedent
from typing import List

import json

import pytest

pytest.importorskip("yaml")

from pilot.config import HostConfigError, ModuleDescriptor, discover_active_modules, load_host_config


@pytest.fixture()
def host_config_path(tmp_path: Path) -> Path:
    """Create a temporary host configuration file for use in tests."""
    payload = {
        "host": {
            "name": "testbot",
        },
        "modules": {
            "imu": {"launch": True},
            "foot": {"launch": {"arguments": {"foo": "bar"}}, "display_name": "Create Base"},
            "memory": {"launch": False},
            "legacy": {},
        },
    }
    path = tmp_path / "testbot.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_load_host_config_reads_json(host_config_path: Path) -> None:
    """Given a JSON host file, the loader should return its parsed content."""
    config = load_host_config(host_config_path)
    assert config["host"]["name"] == "testbot"
    assert "modules" in config


def test_load_host_config_rejects_missing_file(tmp_path: Path) -> None:
    """The loader raises a helpful error when the file is absent."""
    missing_path = tmp_path / "absent.json"
    with pytest.raises(HostConfigError):
        load_host_config(missing_path)


def test_discover_active_modules_selects_launch_enabled_entries(host_config_path: Path) -> None:
    """Modules should be marked active when launch is enabled in the module directive."""
    config = load_host_config(host_config_path)

    modules: List[ModuleDescriptor] = discover_active_modules(config)
    names = {module.name for module in modules}

    assert names == {"imu", "foot"}

    foot_module = next(module for module in modules if module.name == "foot")
    assert foot_module.display_name == "Create Base"

    imu_module = next(module for module in modules if module.name == "imu")
    assert imu_module.display_name == "Imu"


def test_discover_active_modules_supports_legacy_host_list(host_config_path: Path) -> None:
    """Host manifests that still rely on host.modules continue to work."""
    config = load_host_config(host_config_path)
    config["host"]["modules"] = ["legacy", "imu"]  # type: ignore[index]

    modules: List[ModuleDescriptor] = discover_active_modules(config)
    names = {module.name for module in modules}

    assert names == {"imu", "foot", "legacy"}

    legacy_module = next(module for module in modules if module.name == "legacy")
    assert legacy_module.display_name == "Legacy"


def test_discover_active_modules_handles_empty_config() -> None:
    """An empty configuration yields an empty module list without raising exceptions."""
    modules = discover_active_modules({})
    assert modules == []


def test_load_host_config_supports_jsonc(tmp_path: Path) -> None:
    """Trailing comments in JSONC host configs should be ignored."""
    text = dedent(
        """
        {
          // Modules configured for telemetry coverage
          "modules": {
            "imu": {"launch": true}
          }
        }
        """
    )
    path = tmp_path / "config.jsonc"
    path.write_text(text, encoding="utf-8")

    config = load_host_config(path)
    assert config["modules"]["imu"]["launch"] is True
