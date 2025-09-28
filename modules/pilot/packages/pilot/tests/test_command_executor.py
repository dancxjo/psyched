"""Tests for the command execution helper used by the pilot service."""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any, Dict, Tuple

import pytest

from pilot.app import CommandExecutor


class StubProcess:
    """Asyncio subprocess stub that records invocation details."""

    def __init__(
        self,
        argv: Tuple[str, ...],
        *,
        stdout: bytes = b"",
        stderr: bytes = b"",
        returncode: int = 0,
    ) -> None:
        self.argv = argv
        self.kwargs: Dict[str, Any] = {}
        self._stdout = stdout
        self._stderr = stderr
        self.returncode = returncode

    async def communicate(self) -> Tuple[bytes, bytes]:
        return self._stdout, self._stderr


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


def test_run_strips_ansi_sequences(monkeypatch, repo_root: Path) -> None:
    """ANSI escape sequences should be stripped for log-friendly fields."""

    async def fake_exec(*args: str, **kwargs: Any) -> StubProcess:
        return StubProcess(
            tuple(args),
            stdout=b"\x1b[32mhello\x1b[0m\n",
            stderr=b"\x1b[31merror\x1b[0m",
            returncode=12,
        )

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    executor = CommandExecutor(repo_root)
    result = asyncio.run(executor.run("mod", "ear", "setup", []))

    assert result["code"] == 12
    assert result["stdout"] == "\x1b[32mhello\x1b[0m\n"
    assert result["stderr"] == "\x1b[31merror\x1b[0m"
    assert result["stdout_plain"] == "hello\n"
    assert result["stderr_plain"] == "error"
