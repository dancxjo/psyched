"""Tests for the pilot module catalog helpers."""

from __future__ import annotations

from pathlib import Path

import pytest

from pilot.module_catalog import ModuleCatalog


@pytest.fixture()
def repo_root() -> Path:
    """Return the repository root for integration style tests."""

    current = Path(__file__).resolve()
    # modules/pilot/packages/pilot/tests -> repo root
    return current.parents[5]


def test_catalog_discovers_modules(repo_root: Path) -> None:
    """The catalog should find modules and expose display metadata."""

    catalog = ModuleCatalog(repo_root / "modules")

    modules = {module.name: module for module in catalog.list_modules()}

    assert "pilot" in modules, "Pilot module should be present"

    pilot = modules["pilot"]
    assert pilot.display_name.lower().startswith("pilot")
    assert any(topic.topic == "/cmd_vel" for topic in pilot.topics)


@pytest.mark.parametrize(
    "module_name, expected_command",
    [
        ("pilot", "restart"),
        ("nav", "status"),
    ],
)
def test_catalog_has_standard_commands(repo_root: Path, module_name: str, expected_command: str) -> None:
    """Each module should expose a set of standard mod/sys commands."""

    catalog = ModuleCatalog(repo_root / "modules")
    module = catalog.get_module(module_name)

    assert expected_command in module.commands.mod or expected_command in module.commands.system


def test_catalog_topics_include_qos(repo_root: Path) -> None:
    """Ensure QoS information is preserved for UI consumption."""

    catalog = ModuleCatalog(repo_root / "modules")
    pilot = catalog.get_module("pilot")

    imu_topics = [topic for topic in pilot.topics if topic.topic == "/imu"]
    assert imu_topics, "Pilot module should describe the IMU topic"

    imu_topic = imu_topics[0]
    assert imu_topic.qos.depth >= 1
    assert imu_topic.qos.reliability in {"reliable", "best_effort"}
