"""Regression tests for the cockpit's interpreter shims."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import ModuleType

import pytest


def _write_numpy_package(path: Path, version: str) -> None:
    package = path / "numpy"
    package.mkdir(parents=True, exist_ok=True)
    (package / "__init__.py").write_text(
        "__version__ = \"" + version + "\"\n",
        encoding="utf-8",
    )


@pytest.mark.usefixtures("reset_sitecustomize")
def test_prefers_system_numpy_over_user_site(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The shim should swap in the dist-packages build when available."""

    user_site = tmp_path / "user" / "site-packages"
    system_dist = tmp_path / "system" / "dist-packages"
    _write_numpy_package(user_site, "2.2.6")
    _write_numpy_package(system_dist, "1.26.4")

    monkeypatch.setattr(sys, "path", [str(user_site), str(system_dist)])

    # Pretend the user copy of numpy was already imported.
    user_numpy = ModuleType("numpy")
    user_numpy.__file__ = str(user_site / "numpy" / "__init__.py")
    user_numpy.__path__ = [str(user_site / "numpy")]
    user_numpy.__version__ = "2.2.6"  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "numpy", user_numpy)

    module = importlib.import_module("sitecustomize")
    module._prefer_system_numpy()

    numpy_module = sys.modules.get("numpy")
    assert numpy_module is not None
    assert getattr(numpy_module, "__version__", None) == "1.26.4"
    assert numpy_module.__file__ == str(system_dist / "numpy" / "__init__.py")


@pytest.fixture
def reset_sitecustomize(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ensure sitecustomize gets a clean import per test."""

    original = sys.modules.get("sitecustomize")
    yield
    if original is None:
        monkeypatch.delitem(sys.modules, "sitecustomize", raising=False)
    else:
        monkeypatch.setitem(sys.modules, "sitecustomize", original)

