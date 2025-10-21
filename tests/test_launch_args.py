from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


from tools.launch_args import toml_to_launch_arguments


def write(tmp_path: Path, content: str) -> Path:
    path = tmp_path / "config.toml"
    text = content if content.endswith("\n") else f"{content}\n"
    path.write_text(text, encoding="utf-8")
    return path


def test_module_launch_arguments_from_config_scope(tmp_path: Path) -> None:
    config = write(
        tmp_path,
        """
[config.mod.voice.launch.arguments]
model = "gpt-oss:20b"
max_history = 8
""",
    )

    assert toml_to_launch_arguments(config, module="voice") == [
        "model:=\"gpt-oss:20b\"",
        "max_history:=8",
    ]


def test_module_arguments_from_flat_table(tmp_path: Path) -> None:
    config = write(
        tmp_path,
        """
[config.mod.wifi]
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


def test_empty_config_returns_empty_list(tmp_path: Path) -> None:
    config = write(tmp_path, "")
    assert toml_to_launch_arguments(config, module="voice") == []


def test_unknown_module_returns_empty(tmp_path: Path) -> None:
    config = write(
        tmp_path,
        """
[config.mod.voice.launch.arguments]
model = "gpt-oss"
""",
    )

    assert toml_to_launch_arguments(config, module="does_not_exist") == []


def test_module_without_launch_arguments_returns_empty(tmp_path: Path) -> None:
    config = write(
        tmp_path,
        """
[config.mod.camera.launch]
enabled = true
""",
    )

    assert toml_to_launch_arguments(config, module="camera") == []


def test_module_argument_requires_module_scope(tmp_path: Path) -> None:
    config = write(
        tmp_path,
        """
[config.mod.nav.launch.arguments]
frame_id = "map"
""",
    )

    assert toml_to_launch_arguments(config) == []
