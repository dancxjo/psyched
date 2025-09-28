from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def _install_stub_packages() -> None:
    """Install minimal stub modules required to import voice.node."""

    if "rclpy" not in sys.modules:
        rclpy = types.ModuleType("rclpy")
        rclpy.__path__ = []  # type: ignore[attr-defined]
        rclpy.init = lambda *args, **kwargs: None
        rclpy.shutdown = lambda *args, **kwargs: None
        sys.modules["rclpy"] = rclpy
    else:
        rclpy = sys.modules["rclpy"]

    executors = types.ModuleType("rclpy.executors")
    executors.__path__ = []  # type: ignore[attr-defined]

    class _DummyExecutor:
        def add_node(self, _node) -> None:  # pragma: no cover - stub only.
            return None

        def spin(self) -> None:  # pragma: no cover - stub only.
            return None

    executors.MultiThreadedExecutor = _DummyExecutor  # type: ignore[attr-defined]
    sys.modules["rclpy.executors"] = executors
    setattr(rclpy, "executors", executors)

    node = types.ModuleType("rclpy.node")
    node.__path__ = []  # type: ignore[attr-defined]

    class _DummyLogger:
        def info(self, *_args, **_kwargs) -> None:  # pragma: no cover - stub only.
            return None

        warning = error = exception = info

    class _DummyNode:
        def __init__(self, _name: str) -> None:
            self._name = _name

        def get_logger(self) -> _DummyLogger:
            return _DummyLogger()

    node.Node = _DummyNode  # type: ignore[attr-defined]
    sys.modules["rclpy.node"] = node
    setattr(rclpy, "node", node)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.__path__ = []  # type: ignore[attr-defined]
    sys.modules["std_msgs"] = std_msgs

    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.__path__ = []  # type: ignore[attr-defined]

    class _DummyMsg:
        def __init__(self) -> None:
            self.data = None

    std_msgs_msg.Empty = std_msgs_msg.Float32 = std_msgs_msg.String = std_msgs_msg.UInt32 = _DummyMsg  # type: ignore[attr-defined]
    sys.modules["std_msgs.msg"] = std_msgs_msg
    setattr(std_msgs, "msg", std_msgs_msg)

    psyched_msgs = types.ModuleType("psyched_msgs")
    psyched_msgs.__path__ = []  # type: ignore[attr-defined]
    sys.modules["psyched_msgs"] = psyched_msgs

    psyched_msgs_msg = types.ModuleType("psyched_msgs.msg")
    psyched_msgs_msg.__path__ = []  # type: ignore[attr-defined]

    class _DummyMessage:
        def __init__(self) -> None:
            self.role = ""
            self.content = ""
            self.speaker = ""
            self.confidence = 0.0

    psyched_msgs_msg.Message = _DummyMessage  # type: ignore[attr-defined]
    sys.modules["psyched_msgs.msg"] = psyched_msgs_msg
    setattr(psyched_msgs, "msg", psyched_msgs_msg)


def test_voice_node_imports_without_name_error() -> None:
    module_name = "voice.node"
    for name in [key for key in sys.modules if key.startswith("voice")]:
        del sys.modules[name]

    _install_stub_packages()

    module = importlib.import_module(module_name)

    assert hasattr(module, "VoiceNode"), "VoiceNode should be defined in voice.node"
