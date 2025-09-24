"""Behaviour tree scaffolding tests for the Psyched BT package."""
from __future__ import annotations

import sys
import types
from typing import Any, List

def _ensure_psyched_msgs_stub() -> None:
    if "psyched_msgs.msg" in sys.modules:
        return

    msg_module = types.SimpleNamespace()

    class Message:  # pylint: disable=too-few-public-methods
        """Simple stand-in for the ROS message."""

        def __init__(self) -> None:
            self.role: str = ""
            self.content: str = ""

    msg_module.Message = Message
    pkg_module = types.SimpleNamespace(msg=msg_module)
    sys.modules.setdefault("psyched_msgs", pkg_module)
    sys.modules.setdefault("psyched_msgs.msg", msg_module)


def _ensure_ros_stubs() -> None:
    if "rclpy" not in sys.modules:
        class _StubLogger:
            def debug(self, _msg: str) -> None:  # pragma: no cover - trivial stub
                pass

            info = debug
            warning = debug
            error = debug

        class _StubParamValue:
            def __init__(self, value: str = "idle") -> None:
                self.string_value = value

        class _StubParam:
            def __init__(self, value: str = "idle") -> None:
                self._value = value

            def get_parameter_value(self) -> _StubParamValue:
                return _StubParamValue(self._value)

        class _StubNode:
            def __init__(self, _name: str) -> None:
                self._logger = _StubLogger()

            def declare_parameter(self, _name: str, _default: Any) -> None:
                return None

            def get_parameter(self, _name: str) -> _StubParam:
                return _StubParam()

            def create_subscription(self, *_args: Any, **_kwargs: Any) -> Any:
                return types.SimpleNamespace()

            def create_timer(self, *_args: Any, **_kwargs: Any) -> Any:
                return types.SimpleNamespace()

            def get_logger(self) -> _StubLogger:
                return self._logger

            def destroy_node(self) -> bool:
                return True

        rclpy_module = types.SimpleNamespace(
            init=lambda *args, **kwargs: None,
            shutdown=lambda *args, **kwargs: None,
            spin=lambda *args, **kwargs: None,
            node=types.SimpleNamespace(Node=_StubNode),
        )
        sys.modules.setdefault("rclpy", rclpy_module)
        sys.modules.setdefault("rclpy.node", rclpy_module.node)

    if "std_msgs.msg" not in sys.modules:
        class _SimpleString:  # pylint: disable=too-few-public-methods
            def __init__(self) -> None:
                self.data: str = ""

        std_msgs_module = types.SimpleNamespace(msg=types.SimpleNamespace(String=_SimpleString))
        sys.modules.setdefault("std_msgs", std_msgs_module)
        sys.modules.setdefault("std_msgs.msg", std_msgs_module.msg)


_ensure_ros_stubs()
_ensure_psyched_msgs_stub()

import pytest
import py_trees

from py_trees.common import Access, Status

from psyched_bt.trees.common_blackboard import (
    CONVERSATION_SEED_KEY,
    PERSON_INFO_KEY,
    VOICE_TEXT_KEY,
    SetText,
)
from psyched_bt.trees.common_actions import Speak
from psyched_bt.trees import converse_on_face
from psyched_bt.regime_manager_node import parse_face_payload


class DummyLogger:
    """Minimal logger used by dummy ROS nodes in tests."""

    def debug(self, msg: str) -> None:  # pragma: no cover - trivial logging helper
        self._log("DEBUG", msg)

    def info(self, msg: str) -> None:  # pragma: no cover - trivial logging helper
        self._log("INFO", msg)

    def warning(self, msg: str) -> None:  # pragma: no cover - trivial logging helper
        self._log("WARNING", msg)

    def error(self, msg: str) -> None:  # pragma: no cover - trivial logging helper
        self._log("ERROR", msg)

    @staticmethod
    def _log(level: str, msg: str) -> None:
        print(f"[{level}] {msg}")


class DummyPublisher:
    """Captures messages published by behaviours during tests."""

    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.messages.append(msg)


class DummyNode:
    """Tiny ROS-like node used to exercise behaviours without ROS."""

    def __init__(self) -> None:
        self.publishers: List[DummyPublisher] = []
        self.subscriptions: List[Any] = []
        self._logger = DummyLogger()

    def create_publisher(self, _msg_type: Any, topic: str, _queue_size: int) -> DummyPublisher:
        publisher = DummyPublisher(topic)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(
        self, _msg_type: Any, topic: str, callback: Any, _queue_size: int
    ) -> Any:
        self.subscriptions.append((topic, callback))
        return callback

    def get_logger(self) -> DummyLogger:
        return self._logger


def test_parse_face_payload_handles_json() -> None:
    """Faces published as JSON should return a name dictionary."""
    payload = parse_face_payload('{"name":"Stranger"}')
    assert payload == {"name": "Stranger"}


def test_parse_face_payload_empty_string_returns_none() -> None:
    """Empty payloads should be ignored by the regime manager."""
    assert parse_face_payload("   ") is None


def test_set_text_populates_blackboard() -> None:
    """SetText should convert a detection into voice + conversation prompts."""
    manager_client = py_trees.blackboard.Client(name="manager")
    manager_client.register_key(key=PERSON_INFO_KEY, access=Access.WRITE)
    setattr(manager_client, PERSON_INFO_KEY, {"name": "Ada"})

    behaviour = SetText()
    tree = py_trees.trees.BehaviourTree(root=behaviour)
    tree.setup(timeout=1.0)
    tree.tick()

    reader = py_trees.blackboard.Client(name="reader")
    reader.register_key(key=VOICE_TEXT_KEY, access=Access.READ)
    reader.register_key(key=CONVERSATION_SEED_KEY, access=Access.READ)

    assert getattr(reader, VOICE_TEXT_KEY).startswith("Hi Ada")
    assert "Ada" in getattr(reader, CONVERSATION_SEED_KEY)["content"]


def test_converse_tree_runs_to_completion() -> None:
    """The tree should greet, wait for completion, then seed the chat."""

    manager_client = py_trees.blackboard.Client(name="manager")
    manager_client.register_key(key=PERSON_INFO_KEY, access=Access.WRITE)
    setattr(manager_client, PERSON_INFO_KEY, {"name": "Grace"})

    node = DummyNode()
    root = converse_on_face.create_tree()
    tree = py_trees.trees.BehaviourTree(root=root)
    tree.setup(timeout=1.0, node=node)

    # First tick should publish speech and wait for completion.
    tree.tick()
    speak_behaviour = root.children[1]
    assert speak_behaviour.status == Status.RUNNING
    publisher = node.publishers[0]
    assert publisher.topic == "/voice"
    assert publisher.messages, "Speak should publish a greeting"
    utterance = getattr(publisher.messages[-1], "data", None)
    assert isinstance(utterance, str) and "Grace" in utterance

    # Simulate the voice_done callback with a generic payload and tick again.
    done_msg = types.SimpleNamespace(data="done")
    node.subscriptions[0][1](done_msg)

    tree.tick()
    assert speak_behaviour.status == Status.SUCCESS
    seed_behaviour = root.children[2]
    assert seed_behaviour.status == Status.SUCCESS

    conversation_pub = node.publishers[1]
    assert conversation_pub.topic == "/conversation"
    message = conversation_pub.messages[-1]
    assert getattr(message, "role", "") == "user"
    assert "Grace" in getattr(message, "content", "")


def test_speak_accepts_generic_voice_done_when_enabled() -> None:
    """Speak should finish when configured to accept any completion signal."""

    client = py_trees.blackboard.Client(name="voice_writer")
    client.register_key(key=VOICE_TEXT_KEY, access=Access.WRITE)
    setattr(client, VOICE_TEXT_KEY, "Testing one two")

    behaviour = Speak(accept_any_done=True, voice_topic="/voice", voice_done_topic="voice_done")
    node = DummyNode()
    behaviour.setup(node=node)
    behaviour.initialise()
    assert behaviour.update() == Status.RUNNING

    # Publish completion with a mismatched payload.
    callback = node.subscriptions[0][1]
    callback(types.SimpleNamespace(data="done"))

    assert behaviour.update() == Status.SUCCESS


def test_converse_tree_respects_custom_topics() -> None:
    """Tree wiring should honour overrides for the speech and conversation topics."""

    manager_client = py_trees.blackboard.Client(name="manager_custom")
    manager_client.register_key(key=PERSON_INFO_KEY, access=Access.WRITE)
    setattr(manager_client, PERSON_INFO_KEY, {"name": "Custom"})

    node = DummyNode()
    root = converse_on_face.create_tree(
        voice_topic="/alt_voice",
        voice_done_topic="alt_done",
        conversation_topic="/alt_conversation",
    )
    tree = py_trees.trees.BehaviourTree(root=root)
    tree.setup(timeout=1.0, node=node)

    tree.tick()
    publisher = node.publishers[0]
    assert publisher.topic == "/alt_voice"
    subscription_topic, _ = node.subscriptions[0]
    assert subscription_topic == "alt_done"

    # Complete speech and ensure conversation uses the overridden topic.
    node.subscriptions[0][1](types.SimpleNamespace(data="done"))
    tree.tick()
    assert node.publishers[1].topic == "/alt_conversation"
