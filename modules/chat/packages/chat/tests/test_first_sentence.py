from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path

import pytest


@pytest.fixture(name="chat_module")
def _chat_module():
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

    # Provide lightweight stubs for ROS interfaces so the module can import.
    if "rclpy" not in sys.modules:
        fake_rclpy = types.ModuleType("rclpy")
        fake_node_mod = types.ModuleType("rclpy.node")
        fake_exec_mod = types.ModuleType("rclpy.executors")

        class _Node:
            def __init__(self, *_args, **_kwargs):
                pass

        class _Executor:
            def __init__(self, *_args, **_kwargs):
                pass

            def add_node(self, _node):
                pass

            def spin(self):
                pass

        fake_node_mod.Node = _Node
        fake_exec_mod.MultiThreadedExecutor = _Executor
        fake_rclpy.node = fake_node_mod
        fake_rclpy.executors = fake_exec_mod
        fake_rclpy.init = lambda *args, **kwargs: None
        fake_rclpy.shutdown = lambda *args, **kwargs: None
        sys.modules["rclpy"] = fake_rclpy
        sys.modules["rclpy.node"] = fake_node_mod
        sys.modules["rclpy.executors"] = fake_exec_mod

    if "std_msgs" not in sys.modules:
        fake_std_msgs = types.ModuleType("std_msgs")
        fake_std_msgs_msg = types.ModuleType("std_msgs.msg")

        class _String:
            def __init__(self, data: str = "") -> None:
                self.data = data

        fake_std_msgs_msg.String = _String
        sys.modules["std_msgs"] = fake_std_msgs
        sys.modules["std_msgs.msg"] = fake_std_msgs_msg

    if "psyched_msgs" not in sys.modules:
        fake_pkg = types.ModuleType("psyched_msgs")
        fake_pkg_msg = types.ModuleType("psyched_msgs.msg")

        class _Message:
            def __init__(self) -> None:
                self.role = ""
                self.content = ""
                self.speaker = ""
                self.confidence = 0.0

        fake_pkg_msg.Message = _Message
        sys.modules["psyched_msgs"] = fake_pkg
        sys.modules["psyched_msgs.msg"] = fake_pkg_msg

    module_name = "chat.node"
    if module_name in sys.modules:
        del sys.modules[module_name]
    return importlib.import_module(module_name)


def test_first_sentence_basic(chat_module):
    assert chat_module.first_sentence("Hello world. How are you?") == "Hello world."


def test_first_sentence_no_terminator(chat_module):
    text = "This is a long statement without punctuation"
    result = chat_module.first_sentence(text)
    assert result.startswith("This is a long statement")
    assert len(result) <= 201


def test_first_sentence_handles_whitespace(chat_module):
    assert chat_module.first_sentence("   \n   Hello there!  \n") == "Hello there!"


def test_first_sentence_empty(chat_module):
    assert chat_module.first_sentence("   ") == ""
