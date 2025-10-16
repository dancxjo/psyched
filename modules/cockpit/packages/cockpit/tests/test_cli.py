"""Tests for command line helpers used by the cockpit launcher."""

from __future__ import annotations

import signal
import sys
from pathlib import Path
from typing import Callable

import pytest

pytest.importorskip("yaml")

from cockpit.cli import (
    _register_signal_handlers,
    resolve_frontend_root,
    resolve_modules_root,
)


def test_resolve_frontend_root_accepts_explicit_directory(tmp_path: Path) -> None:
    """An explicit directory argument should be returned as the frontend root."""

    explicit = tmp_path / "frontend"
    explicit.mkdir()

    result = resolve_frontend_root(explicit)

    assert result == explicit.resolve()


def test_resolve_frontend_root_rejects_files(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A file passed as the frontend root should trigger an error."""

    bogus = tmp_path / "frontend"
    bogus.write_text("not a directory", encoding="utf-8")

    monkeypatch.delenv("COCKPIT_FRONTEND_ROOT", raising=False)
    monkeypatch.setenv("REPO_DIR", str(tmp_path))

    with pytest.raises(SystemExit):
        resolve_frontend_root(bogus)


def test_resolve_frontend_root_prefers_environment_variable(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Environment variable overrides should be honored when they point to a directory."""

    env_dir = tmp_path / "ui"
    env_dir.mkdir()

    monkeypatch.setenv("COCKPIT_FRONTEND_ROOT", str(env_dir))
    monkeypatch.setenv("REPO_DIR", str(tmp_path / "repo"))

    result = resolve_frontend_root(None)

    assert result == env_dir.resolve()


def test_resolve_frontend_root_prefers_repo_env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """When REPO_DIR is set, prefer the repo source frontend."""

    monkeypatch.delenv("COCKPIT_FRONTEND_ROOT", raising=False)
    repo_dir = tmp_path / "workspace"
    frontend = repo_dir / "modules" / "cockpit" / "packages" / "cockpit" / "cockpit" / "frontend"
    frontend.mkdir(parents=True)
    monkeypatch.setenv("REPO_DIR", str(repo_dir))

    result = resolve_frontend_root(None)

    assert result == frontend.resolve()


def test_resolve_frontend_root_defaults_to_package(monkeypatch: pytest.MonkeyPatch) -> None:
    """Without overrides the packaged frontend should be used."""

    monkeypatch.delenv("COCKPIT_FRONTEND_ROOT", raising=False)
    monkeypatch.delenv("REPO_DIR", raising=False)

    result = resolve_frontend_root(None)

    package_frontend = Path(__file__).resolve().parents[1] / "cockpit" / "frontend"
    assert result == package_frontend.resolve()


def test_resolve_modules_root_prefers_repo_modules(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The repository's modules directory should be preferred when available."""

    repo_dir = tmp_path / "psyched"
    modules_dir = repo_dir / "modules"
    cockpit_module_dir = modules_dir / "ear"
    cockpit_module_dir.mkdir(parents=True)
    (cockpit_module_dir / "module.toml").write_text("[cockpit]\n", encoding="utf-8")

    frontend_root = repo_dir / "modules" / "cockpit" / "packages" / "cockpit" / "cockpit" / "frontend"
    frontend_root.mkdir(parents=True)

    monkeypatch.setenv("REPO_DIR", str(repo_dir))

    result = resolve_modules_root(None, frontend_root)

    assert result == modules_dir.resolve()


def test_resolve_modules_root_falls_back_to_package_layout(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """When running from an installed package, fall back to the bundled modules directory."""

    package_root = tmp_path / "site-packages" / "cockpit"
    frontend_root = package_root / "frontend"
    frontend_root.mkdir(parents=True)

    bundled_modules = frontend_root.parent.parent / "modules"
    sample_module = bundled_modules / "nav"
    sample_module.mkdir(parents=True)
    (sample_module / "module.toml").write_text("[cockpit]\n", encoding="utf-8")

    monkeypatch.delenv("REPO_DIR", raising=False)

    result = resolve_modules_root(None, frontend_root)

    assert result == bundled_modules.resolve()


def test_register_signal_handlers_requests_shutdown_on_sigint(monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture) -> None:
    """Signal handlers should request shutdown for SIGINT while ignoring SIGHUP."""

    monkeypatch.setattr(sys, "platform", "linux")

    class DummyLoop:
        def __init__(self) -> None:
            self.handlers: dict[int, Callable[[], None]] = {}

        def add_signal_handler(self, signum: int, callback: Callable[[], None]) -> None:
            self.handlers[signum] = callback

    loop = DummyLoop()
    stop_requests: list[str] = []

    _register_signal_handlers(loop, stop_callback=lambda: stop_requests.append("stop"))

    assert signal.SIGINT in loop.handlers
    assert signal.SIGTERM in loop.handlers
    sighup = getattr(signal, "SIGHUP", None)
    if sighup is not None:
        assert sighup in loop.handlers

    caplog.clear()
    with caplog.at_level("INFO"):
        loop.handlers[signal.SIGINT]()

    assert stop_requests == ["stop"]
    assert any("SIGINT" in record.getMessage() for record in caplog.records)

    if sighup is not None:
        caplog.clear()
        with caplog.at_level("INFO"):
            loop.handlers[sighup]()

        assert stop_requests == ["stop"], "SIGHUP should not trigger shutdown"
        assert any("ignoring" in record.getMessage().lower() for record in caplog.records)


def test_register_signal_handlers_windows_fallback(monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture) -> None:
    """On Windows, signal.signal should be used instead of loop.add_signal_handler."""

    monkeypatch.setattr(sys, "platform", "win32")

    recorded: dict[int, Callable[[int, object | None], None]] = {}

    def fake_signal(signum: int, handler: Callable[[int, object | None], None]) -> None:
        recorded[signum] = handler

    monkeypatch.setattr(signal, "signal", fake_signal)

    class DummyLoop:
        def __init__(self) -> None:
            self.handlers: dict[int, Callable[[], None]] = {}

        def add_signal_handler(self, signum: int, callback: Callable[[], None]) -> None:
            raise AssertionError("loop.add_signal_handler should not be used on Windows")

    loop = DummyLoop()
    stop_requests: list[str] = []

    _register_signal_handlers(loop, stop_callback=lambda: stop_requests.append("stop"))

    assert signal.SIGINT in recorded
    assert signal.SIGTERM in recorded

    caplog.clear()
    with caplog.at_level("INFO"):
        recorded[signal.SIGTERM](signal.SIGTERM, None)

    assert stop_requests == ["stop"]
    assert any("SIGTERM" in record.getMessage() for record in caplog.records)

    sighup = getattr(signal, "SIGHUP", None)
    if sighup is not None:
        caplog.clear()
        with caplog.at_level("INFO"):
            recorded[sighup](sighup, None)

        assert stop_requests == ["stop"], "SIGHUP should not request shutdown"
        assert any("ignoring" in record.getMessage().lower() for record in caplog.records)
