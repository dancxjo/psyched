from __future__ import annotations

import os
from pathlib import Path
import sys
import types

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))


os.environ.setdefault("FOREBRAIN_LLM_URL", "")


# ---------------------------------------------------------------------------
# Stub ROS dependencies before importing the node under test
# ---------------------------------------------------------------------------
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

    class _FakeLogger:
        def __init__(self) -> None:
            self.messages: list[tuple[str, str]] = []

        def info(self, message: str) -> None:
            self.messages.append(("info", message))

        def warning(self, message: str) -> None:
            self.messages.append(("warning", message))

        def error(self, message: str) -> None:
            self.messages.append(("error", message))

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
            self.destroyed_publishers: list[_FakePublisher] = []
            self._logger = _FakeLogger()

        def declare_parameter(self, name: str, default_value) -> _FakeParameter:  # noqa: ANN001
            parameter = _FakeParameter(default_value)
            self._parameters[name] = parameter
            return parameter

        def get_parameter(self, name: str) -> _FakeParameter:
            return self._parameters[name]

        def get_parameter_or(self, name: str, default_value) -> _FakeParameter:  # noqa: ANN001
            return self._parameters.get(name, _FakeParameter(default_value))

        def create_publisher(self, _msg_type, topic: str, _qos):  # noqa: ANN001
            publisher = _FakePublisher(topic)
            self.publishers.append(publisher)
            return publisher

        def create_subscription(self, _msg_type, topic: str, callback, _qos):  # noqa: ANN001
            subscription = _FakeSubscription(topic, callback)
            self.subscriptions.append(subscription)
            return subscription

        def destroy_publisher(self, publisher: _FakePublisher) -> None:
            self.destroyed_publishers.append(publisher)

        def destroy_subscription(self, subscription: _FakeSubscription) -> None:
            try:
                self.subscriptions.remove(subscription)
            except ValueError:
                pass

        def get_logger(self) -> _FakeLogger:
            return self._logger

    class _Executor:
        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def add_node(self, _node) -> None:  # noqa: ANN001
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

if "requests" not in sys.modules:
    fake_requests = types.ModuleType("requests")

    class _Response:
        def __init__(self, payload: dict[str, object]) -> None:
            self.status_code = 200
            self._payload = payload

        def json(self) -> dict[str, object]:
            return self._payload

    fake_requests.get = lambda *args, **kwargs: _Response({"models": []})
    fake_requests.post = lambda *args, **kwargs: _Response({"message": {"content": "stub"}})
    sys.modules["requests"] = fake_requests


from psyched_msgs.msg import Message as MsgMessage  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from chat.llm_client import ForebrainUnavailable  # noqa: E402
from chat.node import ChatNode  # noqa: E402


def test_chatnode_adds_assistant_message_after_voice_done() -> None:
    node = ChatNode()
    node._ollama_chat = lambda messages: "Here is a complete answer. With trailing sentence."  # noqa: ANN001

    user = MsgMessage()
    user.role = "user"
    user.content = "Explain the plan."
    user.speaker = "operator"
    user.confidence = 0.8

    node._publish_user_turn(user)
    node.on_conversation(user)

    # User message should be published immediately
    assert node.pub_conversation.messages[-1].role == "user"
    assert node.pub_voice.messages[-1].data == "Here is a complete answer."
    assert not any(msg.role == "assistant" for msg in node.pub_conversation.messages)

    # Simulate voice completion with matching payload
    node.on_voice_done(String("Here is a complete answer."))

    assistant_messages = [msg for msg in node.pub_conversation.messages if msg.role == "assistant"]
    assert len(assistant_messages) == 1
    assert assistant_messages[0].content == "Here is a complete answer."
    assert node.pending_to_confirm == []


def test_chatnode_publishes_spoken_text_even_when_mismatched() -> None:
    node = ChatNode()
    node._ollama_chat = lambda messages: "Affirmative."  # noqa: ANN001

    user = MsgMessage()
    user.role = "user"
    user.content = "Status?"

    node._publish_user_turn(user)
    node.on_conversation(user)

    assert len([msg for msg in node.pub_conversation.messages if msg.role == "assistant"]) == 0

    node.on_voice_done(String("Different reply"))

    assistant_messages = [msg for msg in node.pub_conversation.messages if msg.role == "assistant"]
    assert len(assistant_messages) == 1
    assert assistant_messages[0].content == "Different reply"
    assert node.pending_to_confirm == []


def test_chatnode_matches_voice_done_with_whitespace_variations() -> None:
    node = ChatNode()
    node._ollama_chat = lambda messages: "Mind the gap"  # noqa: ANN001

    user = MsgMessage()
    user.role = "user"
    user.content = "Give a reminder"

    node._publish_user_turn(user)
    node.on_conversation(user)
    node.on_voice_done(String("  Mind   the    gap \n"))

    assistant_messages = [msg for msg in node.pub_conversation.messages if msg.role == "assistant"]
    assert len(assistant_messages) == 1
    assert assistant_messages[0].content == "Mind   the    gap"
    assert node.pending_to_confirm == []


def test_chatnode_falls_back_to_ollama_when_llm_unreachable() -> None:
    node = ChatNode()

    class StubLLM:
        def __init__(self) -> None:
            self.calls = 0

        def generate(self, history):  # noqa: ANN001
            self.calls += 1
            raise ForebrainUnavailable("offline")

    stub = StubLLM()
    node._llm_client = stub
    node._llm_ws_url = "ws://example.invalid/chat"
    node._ollama_chat = lambda messages: "Rescued by Ollama."  # noqa: ANN001

    user = MsgMessage()
    user.role = "user"
    user.content = "Hello?"

    node._publish_user_turn(user)
    node.on_conversation(user)

    assert node.pub_voice.messages[-1].data == "Rescued by Ollama."
    assert stub.calls == 1

