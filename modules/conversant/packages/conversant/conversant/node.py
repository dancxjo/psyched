"""ROS node implementing the Conversant dialog manager."""

from __future__ import annotations

import json
import random
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Sequence
from urllib import request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from .threads import ConversationThread, ConversationTurn, ThreadStore, serialise_turn

__all__ = ["ConversantNode", "main"]


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

    def __init__(self, node: Node, endpoint: Optional[str], *, timeout: float = 2.5) -> None:
        self._node = node
        self._endpoint = endpoint.strip() if endpoint else ""
        self._timeout = timeout

    def enabled(self) -> bool:
        return bool(self._endpoint)

    def generate(
        self,
        *,
        concern: str,
        thread: ConversationThread,
        state: Dict[str, Any],
    ) -> ConcernResponse:
        if not self._endpoint:
            return self._fallback(concern, thread=thread)
        payload = {
            "concern": concern,
            "thread_id": thread.thread_id,
            "history": [serialise_turn(turn) for turn in thread.turns[-6:]],
            "state": state,
        }
        body = json.dumps(payload).encode("utf-8")
        req = request.Request(
            self._endpoint,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with request.urlopen(req, timeout=self._timeout) as resp:
                response_text = resp.read().decode("utf-8")
        except Exception as exc:  # pragma: no cover - network failure path
            self._node.get_logger().warning(
                "Local concern responder failed: %s", exc
            )
            return self._fallback(concern, thread=thread)

        try:
            data = json.loads(response_text)
        except json.JSONDecodeError:
            self._node.get_logger().warning(
                "Concern responder returned invalid JSON: %s", response_text
            )
            return self._fallback(concern, thread=thread)

        if not isinstance(data, dict):
            self._node.get_logger().warning(
                "Concern responder response must be an object"
            )
            return self._fallback(concern, thread=thread)

        speak = str(data.get("speak") or "").strip()
        intent = str(data.get("next_intent") or data.get("intent") or "").strip()
        escalate = bool(data.get("escalate_to_pilot") or data.get("escalate"))

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
        self._responder = ConcernResponder(self, self._local_llm_url or None)
        self._state = {
            "mood": "neutral",
            "situation": "",
            "participants": [],
        }

        self._last_thread_id: Optional[str] = None
        self._voice_paused = False
        self._user_speaking = False
        self._silence_threshold = self._resolve_threshold_seconds(self._mode, self._silence_ms)
        self._last_user_speech_end = time.monotonic()
        self._last_user_voice = time.monotonic()
        self._last_silence_event = time.monotonic()
        self._last_spoken_text: Optional[str] = None

        self._cleanup_timer = self.create_timer(30.0, self._cleanup_threads)

        self.get_logger().info(
            "Conversant ready (mode=%s, silence_ms=%d, filler_phrases=%s, llm=%s)",
            self._mode,
            self._silence_ms,
            ", ".join(self._filler_phrases),
            "enabled" if self._responder.enabled() else "disabled",
        )

    # ------------------------------------------------------------------ helpers
    def _declare_int(self, name: str, default: int) -> int:
        parameter = self.declare_parameter(name, default)
        value = getattr(parameter, "value", default)
        try:
            return int(value)
        except (TypeError, ValueError):
            self.get_logger().warning("Invalid integer for %s: %r", name, value)
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
        if removed:
            self.get_logger().debug("Expired threads: %s", ", ".join(removed))
            if self._last_thread_id in removed:
                self._last_thread_id = None

    def _record_turn(
        self,
        *,
        thread_id: Optional[str],
        role: str,
        text: str,
        intent: str = "",
        metadata: Optional[Dict[str, str]] = None,
    ) -> tuple[ConversationThread, ConversationTurn]:
        thread, turn = self._thread_store.append(
            thread_id=thread_id,
            role=role,
            text=text,
            intent=intent,
            metadata=metadata,
        )
        self._last_thread_id = thread.thread_id
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
            self.get_logger().debug("Resuming voice queue after %.3f s of silence", elapsed)

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
                self.get_logger().warning("Invalid JSON turn control payload: %s", raw)
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

    def _apply_mode(self, mode: str, silence_override: Optional[int]) -> None:
        cleaned_mode = mode.strip().lower() or self._mode
        if cleaned_mode not in _MODE_THRESHOLDS:
            self.get_logger().warning("Unknown turn-taking mode '%s'", cleaned_mode)
            cleaned_mode = self._mode
        self._mode = cleaned_mode
        if silence_override is not None and silence_override >= 0:
            self._silence_ms = silence_override
        else:
            preset_ms = int(_MODE_THRESHOLDS[self._mode] * 1000)
            if self._silence_ms <= 0:
                self._silence_ms = preset_ms
        self._silence_threshold = self._resolve_threshold_seconds(self._mode, self._silence_ms)
        self.get_logger().info(
            "Turn-taking mode -> %s (silence %.0f ms)",
            self._mode,
            self._silence_threshold * 1000.0,
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
