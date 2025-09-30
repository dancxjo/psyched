"""ROS 2 topic session management for the pilot backend."""

from __future__ import annotations

import asyncio
import json
import math
import re
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, MutableMapping, Sequence

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
except ImportError:  # pragma: no cover - not available in unit tests
    get_message = None  # type: ignore[assignment]

try:  # pragma: no cover - optional dependency in unit tests
    from rosidl_runtime_py.convert import message_to_ordereddict as _ros_message_to_ordereddict
except ImportError:  # pragma: no cover - API renamed in some distros
    try:
        from rosidl_runtime_py.convert import message_to_ordered_dict as _ros_message_to_ordereddict  # type: ignore[attr-defined]
    except ImportError:  # pragma: no cover - fallback when convert helpers unavailable
        _ros_message_to_ordereddict = None  # type: ignore[assignment]

try:  # pragma: no cover - optional dependency in unit tests
    from rosidl_runtime_py.set_message import set_message_fields as _set_message_fields
except ImportError:  # pragma: no cover - some distros expose older helpers
    _set_message_fields = None  # type: ignore[assignment]


def message_to_ordered_dict(message):  # type: ignore[override]
    """Convert a ROS message into an ordered mapping for JSON serialization."""

    if _ros_message_to_ordereddict is not None:
        return _ros_message_to_ordereddict(message)
    return getattr(message, "__dict__", {})


def _normalise_json_types(value: Any) -> Any:
    """Recursively normalise ROS payloads so JSON transport remains standards-compliant."""

    if isinstance(value, Mapping):
        return {key: _normalise_json_types(inner) for key, inner in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_normalise_json_types(item) for item in value]
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return None
    return value


_TRANSCRIPT_MESSAGE_TYPE = "psyched_msgs/msg/Transcript"
_BYTE_MULTI_ARRAY_MESSAGE_TYPE = "std_msgs/msg/ByteMultiArray"
_TIMING_PREFIX_PATTERN = re.compile(r"^\[[^\]]+\]\s*")


def _normalise_transcript_payload(payload: Mapping[str, Any]) -> Mapping[str, Any]:
    """Return a payload with human-friendly transcript text.

    The ASR stack sometimes emits transcript strings that contain timing markers or
    JSON-encoded segment metadata. Downstream widgets expect a plain text field, so
    this helper normalises the "text" entry whilst preserving the original
    structure. Segment lists take precedence because they already expose the
    speaker-aligned snippets without timing prefixes. When only a string is
    available we attempt to strip timing decorations and parse JSON fragments.

    Examples:
        >>> _normalise_transcript_payload({"text": "[0.0s -> 1.2s] Hello"})["text"]
        'Hello'
        >>> _normalise_transcript_payload({"text": '{"segments": [{"text": "Hi"}]}'})["text"]
        'Hi'
    """

    if not isinstance(payload, Mapping):
        return payload

    segments = payload.get("segments")
    text_from_segments = _segments_to_text(segments)

    if text_from_segments:
        return _replace_text(payload, text_from_segments)

    text_field = payload.get("text")
    cleaned_text = _coerce_transcript_text(text_field)

    if cleaned_text is None:
        return payload

    return _replace_text(payload, cleaned_text)


def _replace_text(payload: Mapping[str, Any], text: str) -> Mapping[str, Any]:
    if isinstance(payload, MutableMapping):
        payload = dict(payload)
    else:
        payload = dict(payload)
    payload["text"] = text.strip()
    return payload


def _segments_to_text(segments: Any) -> str | None:
    if not isinstance(segments, Sequence) or isinstance(segments, (str, bytes, bytearray)):
        return None
    pieces: list[str] = []
    for segment in segments:
        if not isinstance(segment, Mapping):
            continue
        raw = segment.get("text")
        cleaned = _coerce_transcript_text(raw)
        if cleaned:
            pieces.append(cleaned)
    if not pieces:
        return None
    return " ".join(piece for piece in pieces if piece).strip()


def _coerce_transcript_text(value: Any) -> str | None:
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return ""
        if text.startswith("{") or text.startswith("["):
            parsed = _parse_transcript_json(text)
            if parsed:
                return parsed
        lines = [line.strip() for line in text.replace("\r\n", "\n").split("\n")]
        cleaned_lines = []
        for line in lines:
            if not line:
                continue
            cleaned_lines.append(_TIMING_PREFIX_PATTERN.sub("", line))
        joined = " ".join(cleaned_lines).strip()
        return joined or text
    if isinstance(value, Mapping):
        return _segments_to_text(value.get("segments")) or _coerce_transcript_text(value.get("text"))
    return None


def _parse_transcript_json(text: str) -> str | None:
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        return None
    if isinstance(parsed, Mapping):
        segments_text = _segments_to_text(parsed.get("segments"))
        if segments_text:
            return segments_text
        nested = parsed.get("text")
        coerced = _coerce_transcript_text(nested)
        if coerced is not None:
            return coerced
    if isinstance(parsed, Sequence):
        return _segments_to_text(parsed)
    return None


def _byte_multi_array_payload(message):  # noqa: ANN001
    """Return a JSON-friendly representation of ByteMultiArray messages."""

    mapping = message_to_ordered_dict(message)
    payload = getattr(message, "data", None)

    converted: list[int] | None = None
    if isinstance(payload, (bytes, bytearray, memoryview)):
        converted = [int(b) & 0xFF for b in payload]
    elif isinstance(payload, Sequence) and not isinstance(payload, (str, bytes, bytearray, memoryview)):
        try:
            converted = [int(b) & 0xFF for b in payload]
        except Exception:  # pragma: no cover - guard unusual Sequence types
            converted = None

    if converted is not None:
        mapping = dict(mapping)
        mapping["data"] = converted

    return _normalise_json_types(mapping)


def _dict_to_ros_message(msg_class, message_type: str, values: Mapping[str, Any]):
    """Instantiate and populate a ROS message from a plain mapping."""

    if _set_message_fields is None:
        raise RuntimeError("rosidl_runtime_py is required for publishing")

    message = msg_class()
    try:
        _set_message_fields(message, dict(values))
    except Exception as exc:  # pragma: no cover - runtime validation
        raise ValueError(f"Failed to populate {message_type} from payload: {exc}") from exc
    return message


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
                if message_type == _BYTE_MULTI_ARRAY_MESSAGE_TYPE:
                    data = _byte_multi_array_payload(message)
                else:
                    data = _normalise_json_types(message_to_ordered_dict(message))
                    if message_type == _TRANSCRIPT_MESSAGE_TYPE:
                        data = _normalise_transcript_payload(data)
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
        session = entry["session"]
        ros_message = _dict_to_ros_message(entry["msg_class"], session.message_type, data)
        publisher.publish(ros_message)
