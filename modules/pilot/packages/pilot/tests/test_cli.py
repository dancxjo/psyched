"""Tests for command line helpers used by the cockpit launcher."""

from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("yaml")

from pilot.cli import resolve_frontend_root, resolve_modules_root


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


def test_resolve_frontend_root_prefers_repo_env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """When REPO_DIR is set, prefer the repo source frontend."""

    monkeypatch.delenv("PILOT_FRONTEND_ROOT", raising=False)
    repo_dir = tmp_path / "workspace"
    frontend = repo_dir / "modules" / "pilot" / "packages" / "pilot" / "pilot" / "frontend"
    frontend.mkdir(parents=True)
    monkeypatch.setenv("REPO_DIR", str(repo_dir))

    result = resolve_frontend_root(None)

    assert result == frontend.resolve()


def test_resolve_frontend_root_defaults_to_package(monkeypatch: pytest.MonkeyPatch) -> None:
    """Without overrides the packaged frontend should be used."""

    monkeypatch.delenv("PILOT_FRONTEND_ROOT", raising=False)
    monkeypatch.delenv("REPO_DIR", raising=False)

    result = resolve_frontend_root(None)

    package_frontend = Path(__file__).resolve().parents[1] / "pilot" / "frontend"
    assert result == package_frontend.resolve()


def test_resolve_modules_root_prefers_repo_modules(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The repository's modules directory should be preferred when available."""

    repo_dir = tmp_path / "psyched"
    modules_dir = repo_dir / "modules"
    pilot_module_dir = modules_dir / "ear"
    pilot_module_dir.mkdir(parents=True)
    (pilot_module_dir / "module.toml").write_text("[pilot]\n", encoding="utf-8")

    frontend_root = repo_dir / "modules" / "pilot" / "packages" / "pilot" / "pilot" / "frontend"
    frontend_root.mkdir(parents=True)

    monkeypatch.setenv("REPO_DIR", str(repo_dir))

    result = resolve_modules_root(None, frontend_root)

    assert result == modules_dir.resolve()


def test_resolve_modules_root_falls_back_to_package_layout(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """When running from an installed package, fall back to the bundled modules directory."""

    package_root = tmp_path / "site-packages" / "pilot"
    frontend_root = package_root / "frontend"
    frontend_root.mkdir(parents=True)

    bundled_modules = frontend_root.parent.parent / "modules"
    sample_module = bundled_modules / "nav"
    sample_module.mkdir(parents=True)
    (sample_module / "module.toml").write_text("[pilot]\n", encoding="utf-8")

    monkeypatch.delenv("REPO_DIR", raising=False)

    result = resolve_modules_root(None, frontend_root)

    assert result == bundled_modules.resolve()
