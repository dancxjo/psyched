"""Tests for command line helpers used by the cockpit launcher."""

from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("yaml")

from pilot.cli import resolve_frontend_root


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

    monkeypatch.delenv("PILOT_FRONTEND_ROOT", raising=False)
    monkeypatch.setenv("REPO_DIR", str(tmp_path))

    with pytest.raises(SystemExit):
        resolve_frontend_root(bogus)


def test_resolve_frontend_root_prefers_environment_variable(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Environment variable overrides should be honored when they point to a directory."""

    env_dir = tmp_path / "ui"
    env_dir.mkdir()

    monkeypatch.setenv("PILOT_FRONTEND_ROOT", str(env_dir))
    monkeypatch.setenv("REPO_DIR", str(tmp_path / "repo"))

    result = resolve_frontend_root(None)

    assert result == env_dir.resolve()


def test_resolve_frontend_root_falls_back_to_repo(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """When no overrides exist, the repo modules/pilot/frontend directory is used."""

    repo_dir = tmp_path / "repo"
    fallback = repo_dir / "modules" / "pilot" / "frontend"
    fallback.mkdir(parents=True)

    monkeypatch.delenv("PILOT_FRONTEND_ROOT", raising=False)
    monkeypatch.setenv("REPO_DIR", str(repo_dir))

    result = resolve_frontend_root(None)

    assert result == fallback.resolve()
