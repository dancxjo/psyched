"""Tests for the pilot backend entrypoint helpers."""

from __future__ import annotations

from pathlib import Path
import sys
import types

import pytest

# Provide lightweight stubs so importing pilot.main does not require ROS 2.
rclpy_stub = types.ModuleType("rclpy")
rclpy_stub.init = lambda *args, **kwargs: None
rclpy_stub.create_node = lambda *args, **kwargs: object()
rclpy_stub.shutdown = lambda *args, **kwargs: None

executors_stub = types.ModuleType("executors")


class _DummyExecutor:
    def add_node(self, *_args, **_kwargs) -> None:  # pragma: no cover - trivial stub
        pass

    def shutdown(self) -> None:  # pragma: no cover - trivial stub
        pass

    def spin(self) -> None:  # pragma: no cover - trivial stub
        pass


executors_stub.MultiThreadedExecutor = _DummyExecutor

sys.modules.setdefault("rclpy", rclpy_stub)
sys.modules.setdefault("rclpy.executors", executors_stub)

from pilot import main


def test_find_repo_root_discovers_modules(tmp_path: Path) -> None:
    repo_root = tmp_path / "psyched"
    module_dir = repo_root / "modules" / "pilot" / "packages" / "pilot" / "pilot"
    module_dir.mkdir(parents=True)
    (repo_root / "modules").mkdir(parents=True, exist_ok=True)
    (repo_root / "psh").mkdir(parents=True, exist_ok=True)
    (repo_root / "psh" / "main.ts").write_text("// stub\n")

    located = main.find_repo_root(module_dir / "main.py")

    assert located == repo_root


def test_find_repo_root_errors_when_missing_sentinel(tmp_path: Path) -> None:
    start = tmp_path / "random" / "path" / "main.py"
    start.parent.mkdir(parents=True)

    with pytest.raises(RuntimeError):
        main.find_repo_root(start)
