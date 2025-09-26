from pathlib import Path
import sys
import types

package_root = str(Path(__file__).resolve().parents[1])
if package_root not in sys.path:
    sys.path.insert(0, package_root)

if "rclpy" not in sys.modules:
    fake_rclpy = types.ModuleType("rclpy")
    fake_node_mod = types.ModuleType("rclpy.node")
    fake_exec_mod = types.ModuleType("rclpy.executors")

    class _Node:
        def __init__(self, *_args, **_kwargs) -> None:
            pass

    class _Executor:
        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def add_node(self, _node) -> None:
            pass

        def spin(self) -> None:
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

    class _Transcript:
        def __init__(self) -> None:
            self.text = ""
            self.speaker = ""
            self.confidence = 0.0

    fake_pkg_msg.Message = _Message
    fake_pkg_msg.Transcript = _Transcript
    sys.modules["psyched_msgs"] = fake_pkg
    sys.modules["psyched_msgs.msg"] = fake_pkg_msg

import pytest

from psyched_msgs.msg import Message as MsgMessage, Transcript

from chat.node import transcript_to_message


def test_transcript_to_message_includes_metadata():
    transcript = Transcript()
    transcript.text = 'Hello robot'
    transcript.speaker = 'operator'
    transcript.confidence = 0.72

    msg = transcript_to_message(transcript)

    assert isinstance(msg, MsgMessage)
    assert msg.role == 'user'
    assert msg.content == 'Hello robot'
    assert msg.speaker == 'operator'
    assert msg.confidence == pytest.approx(0.72)


def test_transcript_to_message_requires_text():
    transcript = Transcript()
    transcript.text = '   '
    transcript.speaker = 'operator'

    msg = transcript_to_message(transcript)

    assert msg is None
