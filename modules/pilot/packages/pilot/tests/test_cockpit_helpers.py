"""Tests for cockpit integration helpers used by the pilot node."""

from __future__ import annotations

import json
from pathlib import Path

from pilot.cockpit_helpers import (
    load_local_cockpit_actions,
    normalise_cockpit_modules_payload,
)


def _write_actions_manifest(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_load_local_cockpit_actions_reads_all_modules(tmp_path: Path) -> None:
    """Local cockpit manifests should be aggregated into a flat action list."""

    modules_root = tmp_path / "modules"
    _write_actions_manifest(
        modules_root / "alpha" / "cockpit" / "api" / "actions.json",
        {
            "actions": [
                {
                    "name": "say_hi",
                    "description": "Speak a greeting",
                    "parameters": {"text": {"type": "string"}},
                },
                {"name": "noop", "description": "Do nothing"},
            ]
        },
    )
    _write_actions_manifest(
        modules_root / "beta" / "cockpit" / "api" / "actions.json",
        {
            "actions": [
                {
                    "name": "wave",
                    "description": "Wave a limb",
                    "parameters": {"duration": {"type": "number"}},
                },
                {"name": "", "description": "invalid"},
                "not-a-dict",
            ]
        },
    )

    metadata, schemas = load_local_cockpit_actions(modules_root)

    assert sorted(metadata.keys()) == [
        "alpha.noop",
        "alpha.say_hi",
        "beta.wave",
    ]
    assert metadata["alpha.say_hi"]["signature"].startswith("say_hi(")
    assert metadata["alpha.say_hi"]["description"] == "Speak a greeting"
    assert metadata["beta.wave"]["module"] == "beta"

    assert schemas == {
        "alpha.noop": {},
        "alpha.say_hi": {"text": {"type": "string"}},
        "beta.wave": {"duration": {"type": "number"}},
    }


def test_load_local_cockpit_actions_handles_missing_root(tmp_path: Path) -> None:
    """Missing module directories should yield an empty action set."""

    empty_root = tmp_path / "no-modules-here"

    metadata, schemas = load_local_cockpit_actions(empty_root)

    assert metadata == {}
    assert schemas == {}


def test_normalise_cockpit_modules_payload(tmp_path: Path) -> None:
    """Module payloads from the cockpit API should be condensed by module name."""

    payload = {
        "modules": [
            {
                "name": "cockpit",
                "display_name": "Cockpit",
                "slug": "cockpit",
                "dashboard_url": "/modules/cockpit/",
                "has_cockpit": True,
                "systemd": {
                    "unit": "psh-module-cockpit.service",
                    "active": True,
                    "enabled": True,
                },
                "extra": "ignored",
            },
            {
                "name": "imu",
                "display_name": "IMU",
                "systemd": "not-a-dict",
            },
            "not-a-module",
        ],
        "host": {"name": "pete.dev", "shortname": "pete"},
        "bridge": {"mode": "ws", "video_port": 8089},
    }

    status = normalise_cockpit_modules_payload(payload)

    assert status == {
        "modules": {
            "cockpit": {
                "display_name": "Cockpit",
                "slug": "cockpit",
                "dashboard_url": "/modules/cockpit/",
                "has_cockpit": True,
                "systemd": {
                    "unit": "psh-module-cockpit.service",
                    "active": True,
                    "enabled": True,
                },
            },
            "imu": {
                "display_name": "IMU",
                "slug": None,
                "dashboard_url": None,
                "has_cockpit": False,
                "systemd": {},
            },
        },
        "host": {"name": "pete.dev", "shortname": "pete"},
        "bridge": {"mode": "ws", "video_port": 8089},
    }

