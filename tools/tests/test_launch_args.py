from pathlib import Path

import pytest

from tools.launch_args import toml_to_launch_arguments


def write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "config.toml"
    path.write_text(text.strip() + "\n", encoding="utf-8")
    return path


def test_empty_file_returns_no_arguments(tmp_path: Path) -> None:
    path = write(tmp_path, "")
    assert toml_to_launch_arguments(path) == []


def test_arguments_table_converts_to_launch_args(tmp_path: Path) -> None:
    path = write(
        tmp_path,
        """
        [launch.arguments]
        system_prompt = "Hello world"
        enable_http = true
        web_port = 8080
        """
    )
    assert toml_to_launch_arguments(path) == [
        'system_prompt:="Hello world"',
        'enable_http:=true',
        'web_port:=8080',
    ]


def test_invalid_type_raises(tmp_path: Path) -> None:
    path = write(
        tmp_path,
        """
        [launch.arguments]
        options = [1, 2, 3]
        """
    )
    with pytest.raises(TypeError):
        toml_to_launch_arguments(path)
