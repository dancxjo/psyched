#!/usr/bin/env python3
from __future__ import annotations

"""
A lean, explicit rewrite of the chat node to avoid "AI-slop" behaviors:
- No dynamic API probing; use Ollama's official client in one clear way.
- No hidden HTTP fallbacks; fail fast with a crisp error.
- Deterministic streaming: we either stream or we don't, controlled by a flag.
- Minimal history handling and turn-taking, no duplicated assistant echoes.
- Clean logging and small, testable helpers.

ROS graph (topics):
  In:
    - conversation (psyched_msgs/Message)  # expects user turns
    - voice/done (std_msgs/String)         # what TTS actually spoke
  Out:
    - voice/say (std_msgs/String)          # TTS input (assistant's line)
    - chat/stream (std_msgs/String)        # live streaming buffer (optional)
    - conversation (psyched_msgs/Message)  # assistant turn after TTS confirms

Env / Params:
  - OLLAMA_HOST (env) default "http://127.0.0.1:11434"
  - OLLAMA_MODEL (env) default "gpt-oss:20b"
  - declare_parameter("max_history", 20)
  - declare_parameter("stream", True)        # enable streaming to chat/stream
  - declare_parameter("first_sentence_only", True)
  - declare_parameter("system_prompt", "You are a helpful assistant.")

Notes:
  - Keep responsibilities tight: compose -> call LLM -> speak -> confirm -> log.
  - No Pilot scraping, no sys.modules magic, no unused transcript glue.
"""

import logging
import os
from typing import Any, Dict, List, Optional, Callable

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from psyched_msgs.msg import Message as MsgMessage

try:  # pragma: no cover - requires ROS runtime
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
except Exception:  # pragma: no cover - exercised in tests
    QoSDurabilityPolicy = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    QoSReliabilityPolicy = None  # type: ignore[assignment]

try:
    # Official client: https://pypi.org/project/ollama/
    import ollama
except Exception as e:  # pragma: no cover
    raise RuntimeError(
        "The 'ollama' package is required. Install with: pip install ollama"
    ) from e


# -------------------------- helpers -------------------------- #

def normalize_whitespace(s: str) -> str:
    return " ".join((s or "").split())


def first_sentence(s: str, fallback_chars: int = 200) -> str:
    s = (s or "").strip()
    if not s:
        return s
    for i, ch in enumerate(s):
        if ch in ".!?":
            return s[: i + 1].strip()
    # No terminal punctuation—truncate politely
    return (s[:fallback_chars] + ("…" if len(s) > fallback_chars else "")).strip()


# ---------------------------- LLM ---------------------------- #

class OllamaChat:
    def __init__(self, host: str, model: str) -> None:
        self._client = ollama.Client(host=host)
        self._model = model

    def chat(self,
             messages: List[Dict[str, str]],
             stream: bool,
             on_token: Optional[Callable[[str], None]] = None) -> str:
        """Call Ollama in a single, explicit way.
        If stream=True, on_token will be called with incremental content.
        Returns the final full assistant text either way.
        """
        if stream:
            full = []
            for chunk in self._client.chat(model=self._model, messages=messages, stream=True):
                # chunk shape: { 'message': { 'role': 'assistant', 'content': '...' } }
                token = (chunk.get('message', {}) or {}).get('content', '')
                if token:
                    full.append(token)
                    if on_token:
                        on_token(token)
            return "".join(full).strip()
        else:
            resp = self._client.chat(model=self._model, messages=messages)
            return (resp.get('message', {}) or {}).get('content', '').strip()


# ---------------------------- Node --------------------------- #

class ChatNode(Node):
    def __init__(self) -> None:
        super().__init__("chat_node")

        # pubs
        # Use best-effort QoS for conversational short-lived messages so
        # it matches other nodes (for example the voice node) which publish
        # with BEST_EFFORT reliability. This avoids the "incompatible QoS"
        # warning where RELIABILITY policies don't match.
        def _best_effort_qos(*, depth: int = 10) -> Any:
            if (
                QoSProfile is None
                or QoSHistoryPolicy is None
                or QoSReliabilityPolicy is None
                or QoSDurabilityPolicy is None
            ):
                return depth
            return QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=depth,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

        self.pub_conversation = self.create_publisher(MsgMessage, "conversation", _best_effort_qos(depth=10))
        self.pub_voice = self.create_publisher(String, "voice/say", 10)
        self.pub_stream = self.create_publisher(String, "chat/stream", 10)

        # subs
        self.create_subscription(MsgMessage, "conversation", self._on_conversation, _best_effort_qos(depth=10))
        self.create_subscription(String, "voice/done", self._on_voice_done, 10)

        # params
        self.max_history: int = int(self.declare_parameter("max_history", 20).value)
        self.stream_enabled: bool = bool(self.declare_parameter("stream", True).value)
        self.first_sentence_only: bool = bool(self.declare_parameter("first_sentence_only", True).value)
        self.system_prompt: str = str(self.declare_parameter("system_prompt", "You are a helpful assistant.").value)

        # env
        host = os.environ.get("OLLAMA_HOST", "http://forebrain.local:11434").rstrip("/")
        model = os.environ.get("OLLAMA_MODEL", "gpt-oss:20b").strip()

        # llm client
        self._llm = OllamaChat(host=host, model=model)

        # state
        self._history: List[Dict[str, Any]] = []
        self._pending_utterance: Optional[str] = None

        # log
        self.get_logger().info(f"chat_node online | model={model} host={host} stream={self.stream_enabled}")

    # ------------------------ callbacks ----------------------- #

    def _on_conversation(self, msg: MsgMessage) -> None:
        # Track every message, but only act on user turns.
        record = {
            "role": msg.role,
            "content": msg.content,
            "speaker": getattr(msg, "speaker", ""),
            "confidence": float(getattr(msg, "confidence", 0.0) or 0.0),
        }
        self._history.append(record)
        if len(self._history) > self.max_history:
            self._history = self._history[-self.max_history :]

        if (msg.role or "").strip().lower() != "user":
            return

        # Compose messages with a single system message at the front.
        messages: List[Dict[str, str]] = [{"role": "system", "content": self.system_prompt}]
        # Keep only role+content for the model—strip extras.
        messages.extend({"role": h["role"], "content": h["content"]} for h in self._history)

        stream_buf: List[str] = []

        def on_token(tok: str) -> None:
            stream_buf.append(tok)
            out = String()
            out.data = "".join(stream_buf)
            self.pub_stream.publish(out)

        # Call the model
        try:
            full_text = self._llm.chat(messages, stream=self.stream_enabled, on_token=on_token if self.stream_enabled else None)
        except Exception as e:
            self.get_logger().error(f"LLM error: {e}")
            return

        if not full_text:
            self.get_logger().warning("empty assistant text")
            return

        say_text = first_sentence(full_text) if self.first_sentence_only else full_text
        self._pending_utterance = say_text

        speak = String(); speak.data = say_text
        self.pub_voice.publish(speak)

    def _on_voice_done(self, msg: String) -> None:
        if not self._pending_utterance:
            return
        spoken = (msg.data or "").strip()
        if not spoken:
            return

        expected = self._pending_utterance
        if normalize_whitespace(spoken) != normalize_whitespace(expected):
            self.get_logger().warn("voice/done mismatch; using spoken text anyway")

        # Publish assistant turn to conversation exactly once, then clear pending.
        out = MsgMessage()
        out.role = "assistant"
        out.content = spoken
        out.speaker = "assistant"
        out.confidence = 1.0
        self.pub_conversation.publish(out)

        self._history.append({"role": "assistant", "content": spoken, "speaker": "assistant", "confidence": 1.0})
        if len(self._history) > self.max_history:
            self._history = self._history[-self.max_history :]

        self._pending_utterance = None


# --------------------------- entry --------------------------- #

def main(argv: Optional[List[str]] = None) -> None:
    rclpy.init(args=argv)
    node = ChatNode()
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
