"""ROS 2 topic session management for the pilot backend."""

from __future__ import annotations

import asyncio
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping

try:
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
except ImportError:  # pragma: no cover - rclpy unavailable during some tests
    QoSProfile = object  # type: ignore[misc,assignment]

    class ReliabilityPolicy:  # type: ignore[too-many-ancestors]
        RELIABLE = "reliable"
        BEST_EFFORT = "best_effort"

    class HistoryPolicy:  # type: ignore[too-many-ancestors]
        KEEP_LAST = "keep_last"
        KEEP_ALL = "keep_all"

    class DurabilityPolicy:  # type: ignore[too-many-ancestors]
        VOLATILE = "volatile"
        TRANSIENT_LOCAL = "transient_local"

try:
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordered_dict, convert_dict_to_ros_message
except ImportError:  # pragma: no cover - not available in unit tests
    get_message = None  # type: ignore[assignment]
    convert_dict_to_ros_message = None  # type: ignore[assignment]

    def message_to_ordered_dict(message):  # type: ignore[override]
        return getattr(message, "__dict__", {})


from .qos import QosConfig


@dataclass(slots=True)
class TopicSession:
    """Description of an active topic bridge."""

    session_id: str
    topic: str
    access: str
    message_type: str
    qos: QosConfig | None = None
    paused: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.session_id,
            "topic": self.topic,
            "access": self.access,
            "paused": self.paused,
            "message_type": self.message_type,
            "qos": self.qos.asdict() if self.qos else None,
        }


class MessageIntrospection:
    """Resolve ROS 2 message types dynamically."""

    def resolve(self, type_name: str):  # noqa: ANN001
        if get_message is None:
            raise RuntimeError("rosidl_runtime_py is required for message introspection")
        return get_message(type_name)


def _resolve_policy(policy_map: Mapping[str, Any], value: str, default_key: str):
    if isinstance(value, str):
        lowered = value.lower()
        if lowered in policy_map:
            return policy_map[lowered]
    return policy_map[default_key]


def _make_qos_profile(config: QosConfig | None):
    if QoSProfile is object:  # pragma: no cover - executed in test stubs
        return config.asdict() if config else {}

    cfg = config or QosConfig()
    history_policy = _resolve_policy(
        {"keep_last": HistoryPolicy.KEEP_LAST, "keep_all": HistoryPolicy.KEEP_ALL},
        cfg.history,
        "keep_last",
    )
    reliability_policy = _resolve_policy(
        {
            "reliable": ReliabilityPolicy.RELIABLE,
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
        },
        cfg.reliability,
        "reliable",
    )
    durability_policy = _resolve_policy(
        {
            "volatile": DurabilityPolicy.VOLATILE,
            "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
        },
        cfg.durability,
        "volatile",
    )
    return QoSProfile(
        depth=cfg.depth,
        history=history_policy,
        reliability=reliability_policy,
        durability=durability_policy,
    )


class TopicSessionManager:
    """Create and manage ROS topic bridges for websocket clients."""

    def __init__(
        self,
        node,
        *,
        introspection: MessageIntrospection | None = None,
        loop: asyncio.AbstractEventLoop | None = None,
    ) -> None:
        self._node = node
        self._introspection = introspection or MessageIntrospection()
        if loop is not None:
            self._loop = loop
        else:
            try:
                self._loop = asyncio.get_running_loop()
            except RuntimeError:
                self._loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self._loop)
        self._sessions: Dict[str, Dict[str, Any]] = {}

    def create_session(
        self,
        *,
        topic: str,
        access: str,
        qos: Mapping[str, Any] | None,
        module: str | None,
        message_type: str,
    ) -> TopicSession:
        if access not in {"ro", "wo", "rw"}:
            raise ValueError(f"Unsupported access mode: {access}")

        qos_config = QosConfig.from_mapping(qos if isinstance(qos, Mapping) else None)
        qos_profile = _make_qos_profile(qos_config)

        msg_class = self._introspection.resolve(message_type)

        session_id = uuid.uuid4().hex
        publisher = None
        subscription = None
        queue: asyncio.Queue = asyncio.Queue()

        if access in {"wo", "rw"}:
            publisher = self._node.create_publisher(msg_class, topic, qos_profile)

        if access in {"ro", "rw"}:
            def _callback(message):
                data = message_to_ordered_dict(message)
                if self._loop.is_closed():  # pragma: no cover - defensive
                    return
                try:
                    queue.put_nowait(data)
                except asyncio.QueueFull:  # pragma: no cover - queue size default infinite
                    pass

            subscription = self._node.create_subscription(msg_class, topic, _callback, qos_profile)

        session = TopicSession(
            session_id=session_id,
            topic=topic,
            access=access,
            message_type=message_type,
            qos=qos_config,
        )

        self._sessions[session_id] = {
            "session": session,
            "publisher": publisher,
            "subscription": subscription,
            "queue": queue,
            "msg_class": msg_class,
        }
        return session

    def list_sessions(self) -> Iterable[TopicSession]:
        return [entry["session"] for entry in self._sessions.values()]

    def get_session(self, session_id: str) -> TopicSession | None:
        entry = self._sessions.get(session_id)
        return entry["session"] if entry else None

    def drop_session(self, session_id: str) -> None:
        entry = self._sessions.pop(session_id, None)
        if not entry:
            return
        publisher = entry.get("publisher")
        subscription = entry.get("subscription")
        if publisher is not None:
            self._node.destroy_publisher(publisher)
        if subscription is not None:
            self._node.destroy_subscription(subscription)

    def shutdown(self) -> None:
        for session_id in list(self._sessions.keys()):
            self.drop_session(session_id)

    def set_paused(self, session_id: str, paused: bool) -> TopicSession:
        entry = self._sessions.get(session_id)
        if not entry:
            raise KeyError(session_id)
        session: TopicSession = entry["session"]
        session.paused = paused
        return session

    async def pump_messages(self, session_id: str):
        entry = self._sessions.get(session_id)
        if not entry:
            return
        queue: asyncio.Queue = entry["queue"]
        while True:
            payload = await queue.get()
            yield payload

    async def publish(self, session_id: str, data: Mapping[str, Any]) -> None:
        entry = self._sessions.get(session_id)
        if not entry:
            raise KeyError(session_id)
        publisher = entry.get("publisher")
        if publisher is None:
            raise RuntimeError("Session is read-only")
        if convert_dict_to_ros_message is None:
            raise RuntimeError("rosidl_runtime_py is required for publishing")
        session = entry["session"]
        ros_message = convert_dict_to_ros_message(session.message_type, dict(data))
        publisher.publish(ros_message)
