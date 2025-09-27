"""Unit tests for the topic session manager."""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from pilot.qos import QosConfig
from pilot.topic_manager import TopicSession, TopicSessionManager, _normalise_json_types


@dataclass
class StubPublisher:
    topic_name: str
    last_message: object | None = None

    def publish(self, message):  # noqa: ANN001
        self.last_message = message


@dataclass
class StubSubscription:
    topic_name: str


class StubNode:
    def __init__(self) -> None:
        self.created_publishers = []
        self.created_subscriptions = []

    def create_publisher(self, msg_type, topic: str, qos_profile):  # noqa: ANN001
        self.created_publishers.append((msg_type, topic, qos_profile))
        return StubPublisher(topic)

    def create_subscription(self, msg_type, topic: str, callback, qos_profile):  # noqa: ANN001
        self.created_subscriptions.append((msg_type, topic, qos_profile))
        return StubSubscription(topic)

    def destroy_publisher(self, publisher: StubPublisher) -> None:
        pass

    def destroy_subscription(self, subscription: StubSubscription) -> None:
        pass


class StubMessage:  # noqa: D401 - lightweight message stub
    """Simple ROS-like message object used to verify conversions."""

    def __init__(self, data: float = 0.0) -> None:
        self.data = data


class StubMessageFactory:
    def __call__(self):
        return StubMessage()


class StubMessageIntrospection:
    def __init__(self) -> None:
        self.registry = {"std_msgs/msg/Float64": StubMessage}

    def resolve(self, type_name: str):  # noqa: ANN001
        if type_name not in self.registry:
            raise ValueError(type_name)
        return self.registry[type_name]


def test_topic_manager_creates_qos_profiles():
    node = StubNode()
    introspection = StubMessageIntrospection()
    manager = TopicSessionManager(node=node, introspection=introspection)

    qos = QosConfig(history="keep_last", depth=5, reliability="reliable", durability="volatile")
    session = manager.create_session(topic="/test", access="rw", qos=qos.asdict(), module=None, message_type="std_msgs/msg/Float64")

    assert isinstance(session, TopicSession)
    assert node.created_publishers
    assert node.created_subscriptions


def test_topic_manager_rejects_unknown_types():
    node = StubNode()
    introspection = StubMessageIntrospection()
    manager = TopicSessionManager(node=node, introspection=introspection)

    with pytest.raises(ValueError):
        manager.create_session(topic="/bad", access="ro", qos=None, module=None, message_type="unknown/msg/Foo")


def test_normalise_json_types_replaces_invalid_numbers():
    payload = {
        "finite": 1.23,
        "nan": float("nan"),
        "inf": float("inf"),
        "nested": [float("nan"), {"inner": float("-inf")}, "ok"],
    }

    result = _normalise_json_types(payload)

    assert result["finite"] == 1.23
    assert result["nan"] is None
    assert result["inf"] is None
    assert result["nested"][0] is None
    assert result["nested"][1]["inner"] is None
    assert result["nested"][2] == "ok"
