"""Tests for the pilot module catalog helpers."""

from __future__ import annotations

import socket
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


def test_catalog_includes_regimes(repo_root: Path) -> None:
    """Every module should advertise at least one regime for grouping."""

    catalog = ModuleCatalog(repo_root / "modules")

    modules = catalog.list_modules()
    assert modules, "Expected modules to be discovered"

    for module in modules:
        assert module.regimes, f"Module {module.name} must declare at least one regime"

    nav_module = catalog.get_module("nav")
    assert "navigation" in nav_module.regimes


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


def test_catalog_host_health_topic_matches_hostname(repo_root: Path) -> None:
    """Pilot module should expose the resolved hosts/health/<hostname> path."""

    catalog = ModuleCatalog(repo_root / "modules")
    pilot = catalog.get_module("pilot")

    short_host = socket.gethostname().split(".")[0]
    expected = f"/hosts/health/{short_host}"

    topics = [topic.topic for topic in pilot.topics]
    assert expected in topics, f"Expected host health topic {expected}, found {topics}"


def test_catalog_topics_are_unique(repo_root: Path) -> None:
    """Modules should not declare duplicate topics in their manifest."""

    catalog = ModuleCatalog(repo_root / "modules")

    for module in catalog.list_modules():
        topic_names = [topic.topic for topic in module.topics]
        assert len(topic_names) == len(set(topic_names)), (
            f"Duplicate topics found in module {module.name}: {topic_names}"
        )


def test_ear_module_describes_transcription_topic(repo_root: Path) -> None:
    """Ear manifest should surface transcription metadata for the pilot UI."""

    catalog = ModuleCatalog(repo_root / "modules")
    ear = catalog.get_module("ear")

    transcripts = [topic for topic in ear.topics if topic.topic == "/audio/transcription"]
    assert transcripts, "Ear module must describe the transcription topic"

    transcript_topic = transcripts[0]
    assert transcript_topic.type == "psyched_msgs/msg/Transcript"
    assert transcript_topic.presentation == "transcription"
