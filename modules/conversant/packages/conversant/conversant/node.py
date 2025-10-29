"""ROS node implementing the Conversant dialog manager."""

from __future__ import annotations

import json
import random
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple
from urllib import request
from urllib.parse import urlparse, urlunparse

try:  # pragma: no cover - exercised when ROS 2 dependencies are present
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import Bool, Empty, String
except ImportError:  # pragma: no cover - fallback for test environments without ROS
    rclpy = None  # type: ignore[assignment]

    class Node:  # type: ignore[override]
        """Placeholder ``Node`` used when ROS 2 libraries are unavailable."""

        def __init__(self, *_: object, **__: object) -> None:
            raise RuntimeError("rclpy is required to instantiate ConversantNode")

    Bool = Empty = String = object  # type: ignore[assignment]
    QoSDurabilityPolicy = QoSProfile = QoSReliabilityPolicy = object  # type: ignore[assignment]

from .threads import (
    ConversationThread,
    ConversationTurn,
    ThreadStore,
    build_conversation_export,
    serialise_turn,
)

if TYPE_CHECKING:  # pragma: no cover - type-only imports
    from rclpy.publisher import Publisher

__all__ = ["ConversantNode", "main", "_enrich_metadata_with_topic"]


_DEFAULT_FILLERS = [
    "Hmm...",
    "Let me think about that.",
    "One moment...",
    "Just a second.",
]

_MODE_THRESHOLDS = {
    "aggressive": 0.4,
    "balanced": 0.9,
    "patient": 1.5,
}


@dataclass(slots=True)
class ConcernResponse:
    """Structured response returned by the concern responder."""

    speak: str
    intent: str
    escalate: bool


class ConcernResponder:
    """Optionally delegate concern handling to a lightweight local endpoint."""

    _OLLAMA_SUFFIXES = {"/api/generate", "/api/chat"}

    def __init__(
        self,
        node: Node,
        endpoint: Optional[str],
        *,
        model: Optional[str] = None,
        timeout: float = 2.5,
    ) -> None:
        self._node = node
        self._raw_endpoint = endpoint.strip() if endpoint else ""
        self._timeout = timeout
        self._model = (model or "").strip()
        self._mode = "disabled"
        self._http_endpoint: Optional[str] = None
        self._ollama_endpoint: Optional[str] = None
        self._ollama_timeout = max(timeout, 12.0)
        self._configure_endpoint()

    def _configure_endpoint(self) -> None:
        if not self._raw_endpoint:
            return

        parsed = urlparse(self._raw_endpoint)
        scheme = parsed.scheme.lower()

        if not scheme:
            parsed = urlparse(f"http://{self._raw_endpoint}")
            scheme = parsed.scheme.lower()

        if scheme.startswith("ollama"):
            base_scheme = "http"
            if "+" in scheme:
                _, base_scheme = scheme.split("+", 1)
            rebuilt = parsed._replace(scheme=base_scheme)
            target = urlunparse(rebuilt)
            self._configure_ollama_endpoint(target)
            return

        if scheme in {"http", "https"}:
            suffix = parsed.path.rstrip("/")
            if suffix in self._OLLAMA_SUFFIXES or suffix.endswith("/api/generate"):
                target = urlunparse(parsed)
                self._configure_ollama_endpoint(target)
                return

        self._mode = "http"
        self._http_endpoint = urlunparse(parsed)

    def _configure_ollama_endpoint(self, endpoint: str) -> None:
        endpoint = endpoint.rstrip("/")
        if not endpoint.endswith("/api/generate") and not endpoint.endswith("/api/chat"):
            endpoint = f"{endpoint}/api/generate"
        self._mode = "ollama"
        self._ollama_endpoint = endpoint
        if not self._model:
            self._model = "gemma3:latest"

    def enabled(self) -> bool:
        return self._mode != "disabled"

    def describe(self) -> str:
        if self._mode == "ollama":
            return f"ollama({self._model})"
        if self._mode == "http":
            return "http-endpoint"
        return "disabled"

    def generate(
        self,
        *,
        concern: str,
        thread: ConversationThread,
        state: Dict[str, Any],
    ) -> ConcernResponse:
        if not self.enabled():
            return self._fallback(concern, thread=thread)

        if self._mode == "ollama":
            return self._generate_with_ollama(concern=concern, thread=thread, state=state)
        return self._generate_with_http(concern=concern, thread=thread, state=state)

    def _generate_with_http(
        self,
        *,
        concern: str,
        thread: ConversationThread,
        state: Dict[str, Any],
    ) -> ConcernResponse:
        if not self._http_endpoint:
            return self._fallback(concern, thread=thread)

        payload = {
            "concern": concern,
            "thread_id": thread.thread_id,
            "history": [serialise_turn(turn) for turn in thread.turns[-6:]],
            "state": state,
        }
        body = json.dumps(payload).encode("utf-8")
        req = request.Request(
            self._http_endpoint,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with request.urlopen(req, timeout=self._timeout) as resp:
                response_text = resp.read().decode("utf-8")
        except Exception as exc:  # pragma: no cover - network failure path
            self._node.get_logger().warning(
                f"Local concern responder failed: {exc}"
            )
            return self._fallback(concern, thread=thread)

        try:
            data = json.loads(response_text)
        except json.JSONDecodeError:
            self._node.get_logger().warning(
                f"Concern responder returned invalid JSON: {response_text}"
            )
            return self._fallback(concern, thread=thread)

        return self._normalise_response(data, concern=concern, thread=thread)

    def _generate_with_ollama(
        self,
        *,
        concern: str,
        thread: ConversationThread,
        state: Dict[str, Any],
    ) -> ConcernResponse:
        if not self._ollama_endpoint:
            return self._fallback(concern, thread=thread)

        prompt = self._build_ollama_prompt(concern=concern, thread=thread, state=state)
        payload = {
            "model": self._model or "gemma3:latest",
            "prompt": prompt,
            "stream": False,
            "format": "json",
            "options": {
                "temperature": 0.2,
                "top_p": 0.9,
            },
        }
        body = json.dumps(payload).encode("utf-8")
        req = request.Request(
            self._ollama_endpoint,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with request.urlopen(req, timeout=self._ollama_timeout) as resp:
                outer_text = resp.read().decode("utf-8")
        except Exception as exc:  # pragma: no cover - network failure path
            self._node.get_logger().warning(
                f"Ollama concern responder failed: {exc}"
            )
            return self._fallback(concern, thread=thread)

        try:
            outer = json.loads(outer_text)
        except json.JSONDecodeError:
            self._node.get_logger().warning(
                f"Ollama concern responder returned invalid JSON: {outer_text}"
            )
            return self._fallback(concern, thread=thread)

        error_message = outer.get("error")
        if error_message:
            self._node.get_logger().warning(
                f"Ollama concern responder returned an error: {error_message}"
            )
            return self._fallback(concern, thread=thread)

        response_text = outer.get("response", "")
        if not isinstance(response_text, str):
            self._node.get_logger().warning(
                "Ollama concern responder missing response field"
            )
            return self._fallback(concern, thread=thread)

        response_text = response_text.strip()
        if not response_text:
            self._node.get_logger().warning(
                "Ollama concern responder returned empty response"
            )
            return self._fallback(concern, thread=thread)

        try:
            data = json.loads(response_text)
        except json.JSONDecodeError:
            self._node.get_logger().warning(
                f"Ollama concern responder produced non-JSON payload: {response_text}"
            )
            return self._fallback(concern, thread=thread)

        return self._normalise_response(data, concern=concern, thread=thread)

    def _build_ollama_prompt(
        self,
        *,
        concern: str,
        thread: ConversationThread,
        state: Dict[str, Any],
    ) -> str:
        turns = []
        for turn in thread.turns[-6:]:
            role = "Operator" if turn.role == "user" else "Pete"
            text = (turn.text or "").strip().replace("\n", " ")
            if text:
                turns.append(f"{role}: {text}")
        history = "\n".join(turns) if turns else "none"
        state_json = json.dumps(state, ensure_ascii=True)
        concern_text = concern.strip()

        prompt_lines = [
            "You are Pete's on-device concern responder.",
            "Thread history (most recent last):",
            history,
            "",
            f"Current concern: {concern_text}",
            "",
            "Respond with a JSON object containing keys 'speak', 'intent', and 'escalate'.",
            "- 'speak' is a short, empathetic sentence with 25 words or fewer.",
            "- 'intent' may be an empty string when no follow-up is required.",
            "- 'escalate' is true only for urgent or safety-critical issues.",
            f"Context state: {state_json}",
            "Return only the JSON object.",
        ]
        return "\n".join(prompt_lines)

    def _normalise_response(
        self,
        data: Any,
        *,
        concern: str,
        thread: ConversationThread,
    ) -> ConcernResponse:
        if not isinstance(data, dict):
            self._node.get_logger().warning(
                "Concern responder response must be an object"
            )
            return self._fallback(concern, thread=thread)

        speak = str(data.get("speak") or "").strip()
        intent = str(data.get("next_intent") or data.get("intent") or "").strip()

        escalate_raw = data.get("escalate_to_pilot")
        if escalate_raw is None:
            escalate_raw = data.get("escalate")
        if isinstance(escalate_raw, str):
            escalate = escalate_raw.strip().lower() in {"1", "true", "yes", "on"}
        else:
            escalate = bool(escalate_raw)

        if not speak:
            return self._fallback(concern, thread=thread)
        return ConcernResponse(speak=speak, intent=intent, escalate=escalate)

    def _fallback(self, concern: str, *, thread: ConversationThread) -> ConcernResponse:
        concern_text = concern.strip()
        if not concern_text:
            speak = "I'm here."
        elif concern_text.endswith("?"):
            speak = "Let me check on that."
        elif len(concern_text) <= 24:
            speak = f"Got it: {concern_text}"
        else:
            speak = "Give me a moment while I think."
        speak = speak.strip() or "One moment..."
        return ConcernResponse(speak=speak, intent="", escalate=False)


def _enrich_metadata_with_topic(
    metadata: Optional[Dict[str, str]],
    *,
    topic: str,
) -> Dict[str, str]:
    """Return *metadata* with the active conversation topic attached when present.

    Parameters
    ----------
    metadata:
        Optional metadata dictionary supplied by the caller. A shallow copy is
        returned to avoid mutating the original mapping.
    topic:
        Candidate topic text published by :class:`ConversantNode`. Whitespace is
        trimmed and empty topics are ignored.
    """

    enriched: Dict[str, str] = dict(metadata or {})
    topic_text = str(topic or "").strip()
    if topic_text and "topic" not in enriched:
        enriched["topic"] = topic_text
    return enriched


class ConversantNode(Node):
    """ROS node that manages conversational pacing and concerns."""

    def __init__(self) -> None:
        super().__init__("conversant")
        self._random = random.Random()
        self._mode = self._declare_str("mode", "balanced")
        self._silence_ms = self._declare_int("silence_ms", 900)
        self._thread_ttl_seconds = float(self._declare_int("thread_ttl_seconds", 180))
        filler_value = self.declare_parameter("filler_phrases", "").value
        phrases = self._parse_phrase_list(filler_value)
        self._filler_phrases: List[str] = phrases or list(_DEFAULT_FILLERS)
        self._local_llm_url = self._declare_str("local_llm_url", "")
        self._local_llm_model = self._declare_str("local_llm_model", "")
        self._memory_topic = self._declare_str("memory_topic", "/conversant/memory_event")
        self._vad_topic = self._declare_str("vad_topic", "/ear/speech_active")
        self._silence_topic = self._declare_str("silence_topic", "/ear/silence")
        self._speech_topic = self._declare_str("speech_topic", "/voice")
        self._pause_topic = self._declare_str("pause_topic", "/voice/pause")
        self._resume_topic = self._declare_str("resume_topic", "/voice/resume")
        self._clear_topic = self._declare_str("clear_topic", "/voice/clear")
        self._hesitate_topic = self._declare_str("hesitate_topic", "/conversant/hesitate")
        self._concern_topic = self._declare_str("concern_topic", "/conversant/concern")
        self._turn_control_topic = self._declare_str("turn_control_topic", "/conversant/turn_control")
        self._spoken_topic = self._declare_str("spoken_topic", "/voice/spoken")

        self._speech_pub = self.create_publisher(String, self._speech_topic, 10)
        self._pause_pub = self.create_publisher(Empty, self._pause_topic, 10)
        self._resume_pub = self.create_publisher(Empty, self._resume_topic, 10)
        self._clear_pub = self.create_publisher(Empty, self._clear_topic, 10)
        self._memory_pub = self.create_publisher(String, self._memory_topic, 10)

        self.create_subscription(Empty, self._hesitate_topic, self._handle_hesitate, 10)
        self.create_subscription(String, self._concern_topic, self._handle_concern, 20)
        self.create_subscription(String, self._turn_control_topic, self._handle_turn_control, 10)
        self.create_subscription(Bool, self._vad_topic, self._handle_voice_activity, 20)
        self.create_subscription(Bool, self._silence_topic, self._handle_silence, 20)
        self.create_subscription(String, self._spoken_topic, self._handle_spoken, 10)

        self._thread_store = ThreadStore(ttl_seconds=self._thread_ttl_seconds, max_turns=24)
        self._responder = ConcernResponder(
            self,
            self._local_llm_url or None,
            model=self._local_llm_model or None,
        )
        self._state = {
            "mood": "neutral",
            "situation": "",
            "participants": [],
            "topic": "",
        }

        self._topic_topic = self._declare_str("topic_topic", "/conversant/topic")
        self._topic_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._topic_pub = self.create_publisher(String, self._topic_topic, self._topic_qos)

        self._conversation_topic_prefix = self._declare_str(
            "conversation_topic_prefix",
            "/conversation",
        )
        self._default_thread_id = self._declare_str("default_thread_id", "default")
        self._default_user_id = self._declare_str("default_user_id", "operator")
        self._conversation_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._conversation_publishers: Dict[str, "Publisher"] = {}
        self._thread_users: Dict[str, str] = {}
        self._pending_conversant_turns: List[
            Tuple[ConversationThread, ConversationTurn]
        ] = []
        self._active_thread_id = self._default_thread_id

        default_thread = self._thread_store.get(self._default_thread_id)
        self._thread_users[self._default_thread_id] = self._default_user_id
        self._publish_conversation_snapshot(default_thread)

        self._last_thread_id: Optional[str] = self._default_thread_id
        self._voice_paused = False
        self._user_speaking = False
        self._silence_threshold = self._resolve_threshold_seconds(self._mode, self._silence_ms)
        self._last_user_speech_end = time.monotonic()
        self._last_user_voice = time.monotonic()
        self._last_silence_event = time.monotonic()
        self._last_spoken_text: Optional[str] = None

        self._cleanup_timer = self.create_timer(30.0, self._cleanup_threads)

        filler_summary = ", ".join(self._filler_phrases)
        responder_desc = self._responder.describe()
        self.get_logger().info(
            f"Conversant ready (mode={self._mode}, silence_ms={self._silence_ms}, "
            f"filler_phrases={filler_summary}, llm={responder_desc})"
        )
        self._broadcast_topic()

    # ------------------------------------------------------------------ helpers
    def _declare_int(self, name: str, default: int) -> int:
        parameter = self.declare_parameter(name, default)
        value = getattr(parameter, "value", default)
        try:
            return int(value)
        except (TypeError, ValueError):
            self.get_logger().warning(f"Invalid integer for {name}: {value!r}")
            return int(default)

    def _declare_str(self, name: str, default: str) -> str:
        parameter = self.declare_parameter(name, default)
        value = getattr(parameter, "value", default)
        if isinstance(value, str):
            return value.strip() or default
        return str(value).strip() or default

    def _parse_phrase_list(self, raw: Any) -> List[str]:
        phrases: List[str] = []
        if isinstance(raw, Sequence) and not isinstance(raw, str):
            for item in raw:
                text = str(item).strip()
                if text:
                    phrases.append(text)
        elif isinstance(raw, str):
            for part in raw.split(","):
                text = part.strip()
                if text:
                    phrases.append(text)
        return phrases

    def _resolve_threshold_seconds(self, mode: str, silence_ms: int) -> float:
        mode_key = mode.strip().lower()
        default_threshold = _MODE_THRESHOLDS.get(mode_key, _MODE_THRESHOLDS["balanced"])
        if silence_ms > 0:
            return max(silence_ms, 0) / 1000.0
        return default_threshold

    def _choose_filler(self) -> Optional[str]:
        if not self._filler_phrases:
            return None
        return self._random.choice(self._filler_phrases)

    def _cleanup_threads(self) -> None:
        removed = self._thread_store.prune()
        if not removed:
            return

        self.get_logger().debug("Expired threads: %s", ", ".join(removed))
        removed_set = set(removed)
        for thread_id in removed:
            publisher = self._conversation_publishers.pop(thread_id, None)
            if publisher is not None:
                try:
                    self.destroy_publisher(publisher)
                except Exception:  # pragma: no cover - ROS tear-down may vary
                    pass
            self._thread_users.pop(thread_id, None)

        self._pending_conversant_turns = [
            entry
            for entry in self._pending_conversant_turns
            if entry[0].thread_id not in removed_set
        ]

        if self._last_thread_id in removed_set:
            self._last_thread_id = self._default_thread_id
        if self._active_thread_id in removed_set:
            self._active_thread_id = self._default_thread_id

        default_thread = self._thread_store.get(self._default_thread_id)
        self._thread_users.setdefault(self._default_thread_id, self._default_user_id)
        self._publish_conversation_snapshot(default_thread)
        self._broadcast_topic()

    def _resolve_thread_id(self, requested: Optional[str]) -> str:
        candidate = str(requested or "").strip()
        if candidate:
            return candidate
        if self._last_thread_id:
            return self._last_thread_id
        return self._default_thread_id

    def _resolve_user_id(self, metadata: Optional[Dict[str, str]]) -> str:
        if metadata:
            candidate = metadata.get("user_id")
            if isinstance(candidate, str) and candidate.strip():
                return candidate.strip()

        participants = self._state.get("participants")
        if isinstance(participants, Sequence) and not isinstance(participants, str):
            for entry in participants:
                text = str(entry).strip()
                if text:
                    return text
        return self._default_user_id

    def _conversation_topic_name(self, thread_id: str) -> str:
        prefix = self._conversation_topic_prefix.rstrip("/") or "/conversation"
        thread_component = thread_id.strip() or self._default_thread_id
        return f"{prefix}/{thread_component}"

    def _publisher_for_thread(self, thread_id: str) -> "Publisher":
        publisher = self._conversation_publishers.get(thread_id)
        if publisher is not None:
            return publisher
        topic_name = self._conversation_topic_name(thread_id)
        publisher = self.create_publisher(String, topic_name, self._conversation_qos)
        self._conversation_publishers[thread_id] = publisher
        return publisher

    def _publish_conversation_snapshot(
        self,
        thread: ConversationThread,
        *,
        delivered_only: bool = True,
    ) -> None:
        publisher = self._publisher_for_thread(thread.thread_id)
        user_id = self._thread_users.get(thread.thread_id, self._default_user_id)
        payload = build_conversation_export(
            thread,
            user_id=user_id,
            delivered_only=delivered_only,
        )
        message = String()
        message.data = json.dumps(payload, ensure_ascii=True)
        publisher.publish(message)

    def _record_turn(
        self,
        *,
        thread_id: Optional[str],
        role: str,
        text: str,
        intent: str = "",
        metadata: Optional[Dict[str, str]] = None,
    ) -> tuple[ConversationThread, ConversationTurn]:
        resolved_thread_id = self._resolve_thread_id(thread_id)
        enriched_metadata = _enrich_metadata_with_topic(
            metadata,
            topic=self._state.get("topic", ""),
        )
        thread, turn = self._thread_store.append(
            thread_id=resolved_thread_id,
            role=role,
            text=text,
            intent=intent,
            metadata=enriched_metadata,
        )
        self._last_thread_id = thread.thread_id
        self._active_thread_id = thread.thread_id

        if role == "user":
            user_id = self._resolve_user_id(enriched_metadata)
            self._thread_users[thread.thread_id] = user_id
            self._publish_conversation_snapshot(thread)
        else:
            self._thread_users.setdefault(thread.thread_id, self._default_user_id)
            turn.metadata.setdefault("pending_delivery", "true")
            self._pending_conversant_turns.append((thread, turn))

        self._broadcast_topic()
        return thread, turn

    def _publish_memory_event(
        self,
        *,
        thread: ConversationThread,
        turn: ConversationTurn,
        origin: str,
    ) -> None:
        payload = {
            "kind": "conversation",
            "source": "conversant",
            "thread_id": thread.thread_id,
            "role": turn.role,
            "text": turn.text,
            "emoji": ":)" if turn.role == "conversant" else "",
            "intent": turn.intent,
            "timestamp": turn.timestamp.isoformat(),
            "origin": origin,
        }
        if turn.metadata:
            payload["metadata"] = dict(turn.metadata)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=True)
        self._memory_pub.publish(message)

    def _speak(self, text: str) -> None:
        cleaned = text.strip()
        if not cleaned:
            return
        msg = String()
        msg.data = cleaned
        self._speech_pub.publish(msg)

    def _maybe_pause_voice(self) -> None:
        if self._voice_paused:
            return
        self._pause_pub.publish(Empty())
        self._voice_paused = True
        self.get_logger().debug("Pausing voice queue (user speaking)")

    def _maybe_resume_voice(self) -> None:
        if not self._voice_paused:
            return
        elapsed = time.monotonic() - self._last_user_speech_end
        if elapsed >= self._silence_threshold:
            self._resume_pub.publish(Empty())
            self._voice_paused = False
            self.get_logger().debug(
                f"Resuming voice queue after {elapsed:.3f} s of silence"
            )

    def _handle_hesitate(self, _: Empty) -> None:
        phrase = self._choose_filler()
        if not phrase:
            return
        thread, turn = self._record_turn(
            thread_id=self._last_thread_id,
            role="conversant",
            text=phrase,
            metadata={"origin": "hesitation"},
        )
        self._publish_memory_event(thread=thread, turn=turn, origin="hesitation")
        self._speak(phrase)

    def _handle_concern(self, msg: String) -> None:
        payload = self._parse_concern_payload(msg.data)
        concern_text = payload.get("concern", "").strip()
        if not concern_text:
            self.get_logger().debug("Ignoring empty concern payload")
            return
        thread_id = payload.get("thread_id") or self._last_thread_id
        user_intent = payload.get("intent", "")
        escalate_hint = str(payload.get("escalate", "")).strip().lower()
        escalate_requested = escalate_hint in {"1", "true", "yes", "on"}
        metadata = {"origin": "user", "source": "concern", "escalate": str(escalate_requested)}
        user_id = str(payload.get("user_id") or "").strip()
        if user_id:
            metadata["user_id"] = user_id
        thread, user_turn = self._record_turn(
            thread_id=thread_id,
            role="user",
            text=concern_text,
            intent=user_intent,
            metadata=metadata,
        )
        self._publish_memory_event(thread=thread, turn=user_turn, origin="user")

        response = self._responder.generate(concern=concern_text, thread=thread, state=self._state)
        intent_text = response.intent
        escalate = response.escalate or escalate_requested
        if escalate and not intent_text:
            snippet = concern_text[:96]
            intent_text = f"<intend>follow up on concern: {snippet}</intend>"
        if escalate:
            # Drop any stale queued speech so urgent directives bubble up.
            self._clear_pub.publish(Empty())
        metadata = {"origin": "conversant", "source": "concern", "escalate": str(escalate)}
        thread, turn = self._record_turn(
            thread_id=thread.thread_id,
            role="conversant",
            text=response.speak,
            intent=intent_text,
            metadata=metadata,
        )
        self._publish_memory_event(thread=thread, turn=turn, origin="conversant")
        self._speak(response.speak)

    def _handle_turn_control(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        if raw.startswith("{"):
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                self.get_logger().warning(f"Invalid JSON turn control payload: {raw}")
                return
            if not isinstance(payload, dict):
                self.get_logger().warning("Turn control payload must be an object")
                return
            mode = str(payload.get("mode") or self._mode)
            silence_ms_payload = payload.get("silence_ms")
            silence_override: Optional[int] = None
            if silence_ms_payload is not None:
                try:
                    silence_override = int(silence_ms_payload)
                except (TypeError, ValueError):
                    self.get_logger().warning("silence_ms must be an integer")
            self._apply_mode(mode, silence_override)
            state = payload.get("state")
            if isinstance(state, dict):
                self._update_state(state)
            return
        self._apply_mode(raw, None)

    def _handle_voice_activity(self, msg: Bool) -> None:
        active = bool(msg.data)
        now = time.monotonic()
        if active:
            self._user_speaking = True
            self._last_user_voice = now
            self._maybe_pause_voice()
        else:
            if self._user_speaking:
                self._last_user_speech_end = now
            self._user_speaking = False

    def _handle_silence(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        self._last_silence_event = time.monotonic()
        self._maybe_resume_voice()

    def _handle_spoken(self, msg: String) -> None:
        self._last_spoken_text = msg.data.strip() or None
        if not self._last_spoken_text:
            return

        for index, (thread, turn) in enumerate(list(self._pending_conversant_turns)):
            if turn.text.strip() == self._last_spoken_text:
                turn.metadata.pop("pending_delivery", None)
                turn.metadata["delivered"] = datetime.now(timezone.utc).isoformat()
                del self._pending_conversant_turns[index]
                self._publish_conversation_snapshot(thread)
                self._last_thread_id = thread.thread_id
                self._active_thread_id = thread.thread_id
                self._broadcast_topic()
                return

        if self._pending_conversant_turns:
            thread, turn = self._pending_conversant_turns.pop(0)
            turn.metadata.pop("pending_delivery", None)
            turn.metadata["delivered"] = datetime.now(timezone.utc).isoformat()
            self._publish_conversation_snapshot(thread)
            self._last_thread_id = thread.thread_id
            self._active_thread_id = thread.thread_id
            self._broadcast_topic()

    def _apply_mode(self, mode: str, silence_override: Optional[int]) -> None:
        cleaned_mode = mode.strip().lower() or self._mode
        if cleaned_mode not in _MODE_THRESHOLDS:
            self.get_logger().warning(
                f"Unknown turn-taking mode '{cleaned_mode}'"
            )
            cleaned_mode = self._mode
        self._mode = cleaned_mode
        if silence_override is not None and silence_override >= 0:
            self._silence_ms = silence_override
        else:
            preset_ms = int(_MODE_THRESHOLDS[self._mode] * 1000)
            if self._silence_ms <= 0:
                self._silence_ms = preset_ms
        self._silence_threshold = self._resolve_threshold_seconds(self._mode, self._silence_ms)
        silence_ms = self._silence_threshold * 1000.0
        self.get_logger().info(
            f"Turn-taking mode -> {self._mode} (silence {silence_ms:.0f} ms)"
        )

    def _update_state(self, payload: Dict[str, Any]) -> None:
        mood = payload.get("mood")
        situation = payload.get("situation")
        participants = payload.get("participants")
        if isinstance(mood, str) and mood.strip():
            self._state["mood"] = mood.strip()
        if isinstance(situation, str):
            self._state["situation"] = situation.strip()
        if isinstance(participants, Sequence) and not isinstance(participants, str):
            self._state["participants"] = [str(item).strip() for item in participants if str(item).strip()]
        topic = payload.get("topic") or payload.get("conversation_topic")
        if isinstance(topic, str):
            cleaned = topic.strip()
            if cleaned != self._state.get("topic", ""):
                self._state["topic"] = cleaned
                self._broadcast_topic()

    def _broadcast_topic(self) -> None:
        """Publish the current conversation topic for cockpit consumers."""

        message = String()
        message.data = self._conversation_topic_name(self._active_thread_id)
        self._topic_pub.publish(message)

    def _parse_concern_payload(self, raw: str) -> Dict[str, str]:
        if not raw:
            return {}
        text = raw.strip()
        if text.startswith("{"):
            try:
                payload = json.loads(text)
            except json.JSONDecodeError:
                return {"concern": text}
            if isinstance(payload, dict):
                result = {"concern": str(payload.get("concern") or "").strip()}
                thread_id = payload.get("thread_id")
                if thread_id is not None:
                    result["thread_id"] = str(thread_id)
                intent = payload.get("intent")
                if intent is not None:
                    result["intent"] = str(intent)
                user_id = payload.get("user_id") or payload.get("user")
                if user_id is not None:
                    result["user_id"] = str(user_id)
                escalate = payload.get("escalate") or payload.get("escalate_to_pilot")
                if escalate is not None:
                    result["escalate"] = str(escalate)
                return result
            if isinstance(payload, str):
                return {"concern": payload}
        return {"concern": text}

    # ---------------------------------------------------------------- lifecycle
    def destroy_node(self) -> bool:
        try:
            self._cleanup_timer.cancel()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = ConversantNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Conversant interrupted; shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
