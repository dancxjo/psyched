"""Tests for the face embedding websocket server compatibility layer."""
from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from typing import Iterable

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


@pytest.fixture(autouse=True)
def _cleanup_websocket_server(monkeypatch: pytest.MonkeyPatch) -> Iterable[None]:
    """Ensure the websocket server module is re-imported for each test."""

    module_name = "services.faces.embedding_service.websocket_server"
    monkeypatch.delitem(sys.modules, module_name, raising=False)
    yield
    monkeypatch.delitem(sys.modules, module_name, raising=False)


def _install_module(
    monkeypatch: pytest.MonkeyPatch, name: str, module: types.ModuleType
) -> None:
    """Register ``module`` under ``name`` in :data:`sys.modules`."""

    parts = name.split(".")
    for index in range(1, len(parts)):
        parent_name = ".".join(parts[:index])
        parent = sys.modules.get(parent_name)
        if parent is None:
            parent = types.ModuleType(parent_name)
            monkeypatch.setitem(sys.modules, parent_name, parent)
        if not hasattr(parent, "__path__"):
            parent.__path__ = []  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, name, module)


def test_websocket_server_fallback_for_legacy_websockets(monkeypatch: pytest.MonkeyPatch) -> None:
    """The service should fall back to ``websockets.server`` when ``asyncio`` is absent."""

    # Ensure the module cache is clean before installing stubs.
    for name in list(sys.modules):
        if name.startswith("websockets"):
            monkeypatch.delitem(sys.modules, name, raising=False)

    # Minimal stubs for third-party dependencies imported by the module.
    cv2_module = types.ModuleType("cv2")
    cv2_module.IMREAD_COLOR = 1
    cv2_module.COLOR_GRAY2RGB = 0
    cv2_module.COLOR_BGR2RGB = 1
    cv2_module.cvtColor = lambda image, _code: image  # type: ignore[attr-defined]
    cv2_module.imdecode = lambda _array, _flags: object()  # type: ignore[attr-defined]
    _install_module(monkeypatch, "cv2", cv2_module)

    numpy_module = types.ModuleType("numpy")
    numpy_module.ndarray = object  # type: ignore[attr-defined]
    numpy_module.uint8 = "uint8"  # type: ignore[attr-defined]
    numpy_module.float32 = float  # type: ignore[attr-defined]
    numpy_module.frombuffer = lambda data, dtype=None: types.SimpleNamespace(size=len(data))  # type: ignore[attr-defined]
    numpy_module.asarray = lambda array, dtype=None: array  # type: ignore[attr-defined]
    _install_module(monkeypatch, "numpy", numpy_module)

    # ``face_recognition`` is optional at import time; ensure the fallback path is exercised.
    monkeypatch.delitem(sys.modules, "face_recognition", raising=False)

    websockets_module = types.ModuleType("websockets")
    websockets_module.__path__ = []  # type: ignore[attr-defined]
    _install_module(monkeypatch, "websockets", websockets_module)

    class DummyProtocol:
        pass

    async def dummy_serve(*_args: object, **_kwargs: object) -> None:
        raise RuntimeError("should not be awaited in tests")

    websockets_server_module = types.ModuleType("websockets.server")
    websockets_server_module.WebSocketServerProtocol = DummyProtocol  # type: ignore[attr-defined]
    websockets_server_module.serve = dummy_serve  # type: ignore[attr-defined]
    _install_module(monkeypatch, "websockets.server", websockets_server_module)

    websockets_exceptions_module = types.ModuleType("websockets.exceptions")
    websockets_exceptions_module.ConnectionClosed = type("ConnectionClosed", (Exception,), {})  # type: ignore[attr-defined]
    _install_module(monkeypatch, "websockets.exceptions", websockets_exceptions_module)

    module = importlib.import_module("services.faces.embedding_service.websocket_server")

    assert module.ServerConnection is DummyProtocol
    assert module.serve is dummy_serve
