"""Regression tests for the GPS module setup scripts.

These tests ensure the provisioning helpers keep the configuration lines we
rely on when preparing linux hosts for the u-blox 7 receiver.
"""

from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def read_script(name: str) -> str:
    return (REPO_ROOT / "modules" / "gps" / "scripts" / name).read_text(encoding="utf-8")


def test_configure_gpsd_enables_non_daemon_mode() -> None:
    """gpsd should start automatically and stream without waiting for clients."""

    script = read_script("configure_gpsd.sh")
    assert "GPSD_OPTIONS=\"-n\"" in script


def test_enable_gpsd_handles_socket_activation() -> None:
    """The gpsd systemd socket must be enabled so hotplugged receivers connect."""

    script = read_script("enable_gpsd.sh")
    assert "gpsd.socket" in script

