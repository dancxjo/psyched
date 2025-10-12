"""Tests for backend selection logic in :mod:`voice.node`."""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace

# ---------------------------------------------------------------------------
# Minimal ROS stubs so the voice node module can be imported without ROS 2.
if "rclpy" not in sys.modules:
    rclpy_stub = types.ModuleType("rclpy")
    rclpy_stub.init = lambda *_, **__: None
    rclpy_stub.shutdown = lambda *_, **__: None
    sys.modules["rclpy"] = rclpy_stub

if "rclpy.node" not in sys.modules:
    node_stub = types.ModuleType("rclpy.node")

    class _StubNode:
        """Lightweight stand-in used exclusively for import-time wiring."""

        def __init__(self, name: str) -> None:  # pragma: no cover - defensive
            self._name = name

    node_stub.Node = _StubNode
    sys.modules["rclpy.node"] = node_stub

if "rclpy.executors" not in sys.modules:
    executors_stub = types.ModuleType("rclpy.executors")

    class _StubExecutor:
        """Placeholder executor satisfying the type import."""

        def add_node(self, node: object) -> None:  # pragma: no cover - unused
            pass

        def spin(self) -> None:  # pragma: no cover - unused
            pass

        def remove_node(self, node: object) -> None:  # pragma: no cover - unused
            pass

    executors_stub.MultiThreadedExecutor = _StubExecutor
    sys.modules["rclpy.executors"] = executors_stub

if "std_msgs" not in sys.modules:
    std_msgs_stub = types.ModuleType("std_msgs")
    std_msgs_msg_stub = types.ModuleType("std_msgs.msg")

    class _StubMessage:
        """Simple ROS message replacement used for imports only."""

        def __init__(self) -> None:  # pragma: no cover - unused
            self.data = ""

    std_msgs_msg_stub.String = _StubMessage
    std_msgs_msg_stub.Empty = _StubMessage
    std_msgs_stub.msg = std_msgs_msg_stub
    sys.modules["std_msgs"] = std_msgs_stub
    sys.modules["std_msgs.msg"] = std_msgs_msg_stub

# ---------------------------------------------------------------------------
import pytest

from voice.node import PrintSpeechBackend, VoiceNode


class _RecordingLogger:
    """Logger shim capturing formatted log messages for assertions."""

    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def info(self, message: str, *args: object) -> None:
        self.infos.append(self._format(message, args))

    def warning(self, message: str, *args: object) -> None:
        self.warnings.append(self._format(message, args))

    def error(self, message: str, *args: object) -> None:
        self.errors.append(self._format(message, args))

    @staticmethod
    def _format(message: str, args: tuple[object, ...]) -> str:
        return message % args if args else message


def _make_node_with_backend(value: str) -> tuple[VoiceNode, _RecordingLogger]:
    """Create a partially initialised voice node with a fixed backend parameter."""

    node = VoiceNode.__new__(VoiceNode)
    logger = _RecordingLogger()

    def _declare_parameter(name: str, default: object) -> SimpleNamespace:
        return SimpleNamespace(value=value if name == "backend" else default)

    node.declare_parameter = _declare_parameter  # type: ignore[assignment]
    node.get_logger = lambda: logger  # type: ignore[assignment]
    return node, logger


def test_websocket_backend_falls_back_to_espeak(monkeypatch: pytest.MonkeyPatch) -> None:
    """When websocket initialisation fails the node should try espeak."""

    sentinel_backend = object()
    node, logger = _make_node_with_backend("websocket")
    node._create_websocket_backend = lambda: None  # type: ignore[assignment]
    monkeypatch.setattr("voice.node.EspeakSpeechBackend", lambda: sentinel_backend)

    backend = node._create_backend()

    assert backend is sentinel_backend
    assert (
        "Falling back to espeak backend after websocket backend initialisation failure"
        in logger.warnings
    )


def test_websocket_backend_falls_back_to_print_when_espeak_missing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """If espeak is unavailable the node ultimately falls back to print."""

    node, logger = _make_node_with_backend("coqui")
    node._create_websocket_backend = lambda: None  # type: ignore[assignment]

    def _raise_missing() -> None:
        raise FileNotFoundError("no espeak")

    monkeypatch.setattr("voice.node.EspeakSpeechBackend", _raise_missing)

    backend = node._create_backend()

    assert isinstance(backend, PrintSpeechBackend)
    assert any("espeak backend unavailable" in warning for warning in logger.warnings)
