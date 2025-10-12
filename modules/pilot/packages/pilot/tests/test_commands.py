"""Unit tests for command execution helpers."""

from __future__ import annotations

import asyncio
import importlib.util
import sys
from pathlib import Path
from typing import Any, Dict, Tuple

import pytest

import shutil

REPO_ROOT = Path(__file__).resolve().parents[5]
PACKAGE_DIR = REPO_ROOT / "modules" / "pilot" / "packages" / "pilot" / "pilot"

spec = importlib.util.spec_from_file_location(
    "pilot",
    PACKAGE_DIR / "__init__.py",
    submodule_search_locations=[str(PACKAGE_DIR)],
)
module = importlib.util.module_from_spec(spec)
sys.modules.pop("pilot", None)
sys.modules["pilot"] = module
spec.loader.exec_module(module)

from pilot.commands import CommandExecutor  # type: ignore[import]


class _StubProcess:
    def __init__(self, args: Tuple[str, ...]) -> None:
        self.args = args
        self.returncode = 0

    async def communicate(self) -> Tuple[bytes, bytes]:
        return b"", b""


@pytest.fixture()
def repo_root(tmp_path: Path) -> Path:
    """Create a temporary repository layout with a stub psh entrypoint."""

    root = tmp_path / "repo"
    psh_dir = root / "tools" / "psh"
    psh_dir.mkdir(parents=True)
    (psh_dir / "main.ts").write_text("// stub psh\n", encoding="utf-8")
    return root


def test_mod_scope_places_command_before_module(
    repo_root: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: Dict[str, Any] = {}

    fake_psh = str(repo_root / "bin" / "psh")
    (repo_root / "bin").mkdir(parents=True)

    async def fake_exec(*cmd: str, **kwargs: Any) -> _StubProcess:
        captured["cmd"] = cmd
        captured["kwargs"] = kwargs
        return _StubProcess(cmd)

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    monkeypatch.setattr(shutil, "which", lambda name: fake_psh if name == "psh" else None)

    executor = CommandExecutor(repo_root=repo_root)
    result = asyncio.run(executor.run("mod", "pilot", "setup", []))

    assert captured["cmd"] == (fake_psh, "mod", "setup", "pilot")
    assert captured["kwargs"]["cwd"] == str(repo_root)
    assert result["code"] == 0


def test_sys_scope_places_command_before_module(
    repo_root: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: Dict[str, Any] = {}

    fake_psh = str(repo_root / "bin" / "psh")
    (repo_root / "bin").mkdir(parents=True)

    async def fake_exec(*cmd: str, **kwargs: Any) -> _StubProcess:
        captured["cmd"] = cmd
        captured["kwargs"] = kwargs
        return _StubProcess(cmd)

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    monkeypatch.setattr(shutil, "which", lambda name: fake_psh if name == "psh" else None)

    executor = CommandExecutor(repo_root=repo_root)
    result = asyncio.run(executor.run("sys", "pilot", "enable", []))

    assert captured["cmd"] == (fake_psh, "sys", "enable", "pilot")
    assert captured["kwargs"]["cwd"] == str(repo_root)
    assert result["code"] == 0
