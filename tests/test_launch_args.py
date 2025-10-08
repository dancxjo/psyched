from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


import json

from tools.launch_args import toml_to_launch_arguments


def write(tmp_path: Path, content: str, *, suffix: str = ".toml") -> Path:
    path = tmp_path / f"config{suffix}"
    text = content if content.endswith("\n") else f"{content}\n"
    path.write_text(text, encoding="utf-8")
    return path


def test_legacy_launch_arguments(tmp_path: Path) -> None:
    config = write(
        tmp_path,
        """
[launch.arguments]
foo = "bar"
retry = 3

[launch.other]
ignored = "value"
""",
    )

    assert toml_to_launch_arguments(config) == ["foo:=\"bar\"", "retry:=3"]


def test_module_scoped_launch_arguments(tmp_path: Path) -> None:
    config = write(
        tmp_path,
        """
[modules.chat.launch.arguments]
model = "gpt-oss:20b"
max_history = 8
""",
    )

    assert toml_to_launch_arguments(config, module="chat") == [
        "model:=\"gpt-oss:20b\"",
        "max_history:=8",
    ]


def test_module_flat_arguments(tmp_path: Path) -> None:
    config = write(
        tmp_path,
        """
[modules.wifi]
interface = "wlan1"
channel = 11
launch = true
""",
    )

    assert toml_to_launch_arguments(config, module="wifi") == [
        "interface:=\"wlan1\"",
        "channel:=11",
        "launch:=true",
    ]


def test_json_launch_arguments(tmp_path: Path) -> None:
    payload = {
        "host": {"modules": ["voice"]},
        "modules": {
            "voice": {
                "arguments": {
                    "voice_topic": "/voice",
                }
            }
        },
    }
    config = write(tmp_path, json.dumps(payload, indent=2), suffix=".json")

    assert toml_to_launch_arguments(config, module="voice") == [
        "voice_topic:=\"/voice\"",
    ]


def test_empty_config_returns_empty_list(tmp_path: Path) -> None:
    config = write(tmp_path, "")
    assert toml_to_launch_arguments(config) == []


def test_unknown_module_returns_empty(tmp_path: Path) -> None:
    config = write(
        tmp_path,
        """
[modules.chat.launch.arguments]
model = "gpt-oss"
""",
    )

    assert toml_to_launch_arguments(config, module="does_not_exist") == []


def test_duplicate_tables_do_not_block_other_modules(tmp_path: Path) -> None:
    """A duplicate table for an unrelated module should not prevent parsing."""

    config = write(
        tmp_path,
        """
[modules.faces.launch.arguments]
camera_topic = "/camera/old"

[modules.faces.launch.arguments]
camera_topic = "/camera/new"

[modules.nav.launch.arguments]
kinect_rgb_topic = "/camera/color/image_raw"
kinect_depth_topic = "/camera/depth/image_rect_raw"
camera_frame = "camera_link"
""",
    )

    assert toml_to_launch_arguments(config, module="nav") == [
        "kinect_rgb_topic:=\"/camera/color/image_raw\"",
        "kinect_depth_topic:=\"/camera/depth/image_rect_raw\"",
        "camera_frame:=\"camera_link\"",
    ]


def test_duplicate_module_table_prefers_last_definition(tmp_path: Path) -> None:
    """When the module itself is duplicated the newest values should win."""

    config = write(
        tmp_path,
        """
[modules.nav.launch.arguments]
kinect_rgb_topic = "/camera/old"

[modules.nav.launch.arguments]
kinect_rgb_topic = "/camera/new"
""",
    )

    assert toml_to_launch_arguments(config, module="nav") == [
        "kinect_rgb_topic:=\"/camera/new\"",
    ]
