"""Tests for the command execution helper used by the pilot service."""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any, Dict, Tuple

import pytest

from pilot.app import CommandExecutor


class StubProcess:
    """Asyncio subprocess stub that records invocation details."""

    def __init__(self, argv: Tuple[str, ...]) -> None:
        self.argv = argv
        self.kwargs: Dict[str, Any] = {}
        self.returncode = 0

    async def communicate(self) -> Tuple[bytes, bytes]:
        return b"", b""


@pytest.fixture()
def repo_root(tmp_path: Path) -> Path:
    """Create a minimal repository layout for the executor."""

    repo = tmp_path / "repo"
    psh_dir = repo / "psh"
    psh_dir.mkdir(parents=True)
    (psh_dir / "main.ts").write_text("// deno entrypoint")
    return repo


def test_mod_commands_place_module_before_action(monkeypatch, repo_root: Path) -> None:
    """`psh mod` expects modules before the action; ensure we mirror that."""

    captured: Dict[str, Any] = {}

    async def fake_exec(*args: str, **kwargs: Any) -> StubProcess:
        proc = StubProcess(tuple(args))
        proc.kwargs = kwargs
        captured["args"] = args
        captured["kwargs"] = kwargs
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    executor = CommandExecutor(repo_root)
    result = asyncio.run(executor.run("mod", "ear", "setup", ["--flag"]))

    assert result["code"] == 0
    assert captured["args"] == (
        "deno",
        "run",
        "-A",
        str(repo_root / "psh" / "main.ts"),
        "mod",
        "ear",
        "setup",
        "--flag",
    )


def test_system_commands_follow_cli_shape(monkeypatch, repo_root: Path) -> None:
    """System commands should match `psh sys <action> <unit>` semantics."""

    captured: Dict[str, Any] = {}

    async def fake_exec(*args: str, **kwargs: Any) -> StubProcess:
        proc = StubProcess(tuple(args))
        proc.kwargs = kwargs
        captured["args"] = args
        captured["kwargs"] = kwargs
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    executor = CommandExecutor(repo_root)
    result = asyncio.run(executor.run("sys", "pilot", "restart", []))

    assert result["code"] == 0
    assert captured["args"] == (
        "deno",
        "run",
        "-A",
        str(repo_root / "psh" / "main.ts"),
        "sys",
        "restart",
        "pilot",
    )
