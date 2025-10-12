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
    """Create a temporary TOML host configuration using the new schema."""

    text = dedent(
        """
        [host]
        name = "testbot"
        modules = ["imu", "foot"]

        [config.mod.foot]
        display_name = "Create Base"

        [config.mod.foot.launch.arguments]
        foo = "bar"

        [config.mod.legacy]
        launch = true

        [config.mod.memory]
        launch = true
        """
    )
    path = tmp_path / "testbot.toml"
    path.write_text(text, encoding="utf-8")
    return path


@pytest.fixture()
def legacy_host_config_path(tmp_path: Path) -> Path:
    """Create a temporary JSON host configuration for legacy behaviour tests."""

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
    path = tmp_path / "legacy.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_load_host_config_reads_toml(host_config_path: Path) -> None:
    """Given a TOML host file, the loader should return its parsed content."""
    config = load_host_config(host_config_path)
    assert config["host"]["name"] == "testbot"
    assert "config" in config


def test_load_host_config_rejects_missing_file(tmp_path: Path) -> None:
    """The loader raises a helpful error when the file is absent."""
    missing_path = tmp_path / "absent.json"
    with pytest.raises(HostConfigError):
        load_host_config(missing_path)


def test_load_host_config_reads_json(legacy_host_config_path: Path) -> None:
    """JSON host files remain supported for backwards compatibility."""
    config = load_host_config(legacy_host_config_path)
    assert config["host"]["name"] == "testbot"
    assert "modules" in config


def test_discover_active_modules_use_host_module_list(host_config_path: Path) -> None:
    """Modules listed in host.modules should surface in the reported order."""
    config = load_host_config(host_config_path)

    modules: List[ModuleDescriptor] = discover_active_modules(config)
    names = [module.name for module in modules]

    assert names == ["imu", "foot"]

    foot_module = next(module for module in modules if module.name == "foot")
    assert foot_module.display_name == "Create Base"

    imu_module = next(module for module in modules if module.name == "imu")
    assert imu_module.display_name == "Imu"

    assert all(module.name != "legacy" for module in modules)


def test_discover_active_modules_supports_legacy_launch_entries(
    legacy_host_config_path: Path,
) -> None:
    """Legacy manifests without host.modules rely on launch directives."""
    config = load_host_config(legacy_host_config_path)

    modules: List[ModuleDescriptor] = discover_active_modules(config)
    names = {module.name for module in modules}

    assert names == {"imu", "foot"}


def test_discover_active_modules_handles_empty_config() -> None:
    """An empty configuration yields an empty module list without raising exceptions."""
    modules = discover_active_modules({})
    assert modules == []


def test_discover_active_modules_accepts_sequence_modules() -> None:
    """Module lists expressed as sequences should mark each entry as active."""
    modules = discover_active_modules({"host": {"modules": ["imu", "pilot"]}})
    names = [module.name for module in modules]
    assert names == ["imu", "pilot"]


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
