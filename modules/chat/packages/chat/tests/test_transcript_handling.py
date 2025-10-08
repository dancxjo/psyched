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

    class _FakeParameterValue:
        def __init__(self, value: object) -> None:
            self.string_value = str(value)
            try:
                self.integer_value = int(value)  # type: ignore[arg-type]
            except Exception:
                self.integer_value = 0
            try:
                self.double_value = float(value)  # type: ignore[arg-type]
            except Exception:
                self.double_value = 0.0
            self.bool_value = bool(value)

    class _FakeParameter:
        def __init__(self, value: object) -> None:
            self._value = value

        def get_parameter_value(self) -> _FakeParameterValue:
            return _FakeParameterValue(self._value)

        @property
        def value(self) -> object:
            return self._value

    class _FakePublisher:
        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    class _FakeSubscription:
        def __init__(self, topic: str, callback) -> None:  # noqa: ANN001
            self.topic = topic
            self.callback = callback

    class _Node:
        def __init__(self, *_args, **_kwargs) -> None:
            self._parameters: dict[str, _FakeParameter] = {}
            self.publishers: list[_FakePublisher] = []
            self.subscriptions: list[_FakeSubscription] = []
            self._logger = types.SimpleNamespace(
                info=lambda *_args, **_kwargs: None,
                warning=lambda *_args, **_kwargs: None,
                error=lambda *_args, **_kwargs: None,
            )

        def declare_parameter(self, name: str, default_value) -> _FakeParameter:  # noqa: ANN001
            parameter = _FakeParameter(default_value)
            self._parameters[name] = parameter
            return parameter

        def get_parameter(self, name: str) -> _FakeParameter:
            return self._parameters[name]

        def create_publisher(self, _msg_type, topic: str, _qos):  # noqa: ANN001
            publisher = _FakePublisher(topic)
            self.publishers.append(publisher)
            return publisher

        def create_subscription(self, _msg_type, topic: str, callback, _qos):  # noqa: ANN001
            subscription = _FakeSubscription(topic, callback)
            self.subscriptions.append(subscription)
            return subscription

        def get_logger(self):
            return self._logger

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
            self.segments = []
            self.words = []

    class _Transcript:
        def __init__(self) -> None:
            self.text = ""
            self.speaker = ""
            self.confidence = 0.0
            self.segments = []
            self.words = []

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
    transcript.segments = [{'start': 0.0, 'end': 1.2, 'text': 'Hello robot', 'speaker': 'operator'}]
    transcript.words = [
        {'start': 0.0, 'end': 0.6, 'text': 'Hello'},
        {'start': 0.6, 'end': 1.2, 'text': 'robot'},
    ]

    msg = transcript_to_message(transcript)

    assert isinstance(msg, MsgMessage)
    assert msg.role == 'user'
    assert msg.content == 'Hello robot'
    assert msg.speaker == 'operator'
    assert msg.confidence == pytest.approx(0.72)
    assert msg.segments == transcript.segments
    assert msg.words == transcript.words


def test_transcript_to_message_requires_text():
    transcript = Transcript()
    transcript.text = '   '
    transcript.speaker = 'operator'

    msg = transcript_to_message(transcript)

    assert msg is None
