#!/usr/bin/env python3
from __future__ import annotations

import os
from typing import Any, Callable, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from psyched_msgs.msg import Message as MsgMessage, Transcript

# LangChain is required (no HTTP fallback). Use Ollama chat model via LangChain.
from langchain.chat_models import Ollama
from langchain.schema import HumanMessage, SystemMessage, AIMessage


def _normalize_voice_text(text: str) -> str:
    return " ".join(text.split())


def first_sentence(text: str) -> str:
    s = text.strip()
    if not s:
        return s
    for i, ch in enumerate(s):
        if ch in ".!?":
            return s[: i + 1].strip()
    return (s[:200] + ("…" if len(s) > 200 else "")).strip()


def transcript_to_message(transcript: Transcript) -> Optional[MsgMessage]:
    text = str(getattr(transcript, "text", "")).strip()
    if not text:
        return None
    msg = MsgMessage()
    msg.content = text
    msg.speaker = getattr(transcript, "speaker", "") or ""
    try:
        msg.confidence = float(getattr(transcript, "confidence", 0.0) or 0.0)
    except Exception:
        msg.confidence = 0.0
    msg.segments = getattr(transcript, "segments", []) or []
    msg.words = getattr(transcript, "words", []) or []
    return msg


def _normalise_ollama_host(host: str) -> str:
    h = (host or "").strip()
    if not h:
        return ""
    h = h.rstrip("/")
    if h.startswith("http://") or h.startswith("https://"):
        return h
    return "http://" + h


class ChatNode(Node):
    def __init__(self) -> None:
        super().__init__("chat_node")

        self.pub_conversation = self.create_publisher(MsgMessage, "conversation", 10)
        self.pub_voice = self.create_publisher(String, "voice/say", 10)
        self.pub_stream = self.create_publisher(String, "chat/stream", 10)

        self.history: List[Dict[str, Any]] = []
        self.pending_to_confirm: List[str] = []
        self.max_history: int = int(self.declare_parameter("max_history", 20).get_parameter_value().integer_value)

        self.model: str = os.environ.get("OLLAMA_MODEL", "gpt-oss:20b")
        self.ollama_host: str = _normalise_ollama_host(os.environ.get("OLLAMA_HOST", ""))

        # Initialize LangChain Ollama chat model. No fallback — fail fast if
        # langchain/Ollama are not available or misconfigured.
        try:
            # Prefer `base_url` kwarg; some langchain versions accept `url`.
            self._llm = Ollama(model=self.model, base_url=self.ollama_host)  # type: ignore[arg-type]
        except TypeError:
            self._llm = Ollama(model=self.model, url=self.ollama_host)  # type: ignore[arg-type]

        self._serve_proc = None
        self._supports_stream = False
        self._llm_is_chat_model = True

        # wrapper used by _generate_response; supports optional stream_callback
        self._ollama_chat = lambda messages, stream_callback=None: self._call_langchain(messages, stream_callback=stream_callback)

    def _compose_system_message(self) -> str:
        base = "You are a helpful assistant."
        try:
            pilot_base = os.environ.get("PILOT_BASE", "http://localhost:8080")
            url = pilot_base.rstrip("/") + "/api/modules"
            # Querying Pilot's control surface is useful, but the HTTP path was
            # removed. If PILOT_BASE is set and reachable, the LangChain model
            # prompt may still include relevant info via other integrations.
            # Keep this try/except to avoid bringing back direct HTTP calls.
            pass
        except Exception:
            pass
        return base

    def _call_langchain(self, messages: List[Dict[str, str]], stream_callback: Optional[Callable[[str], None]] = None) -> str:
        """Call the LangChain Ollama chat model with a list of message dicts.

        messages is a list of {"role": "system|user|assistant", "content": "..."}.
        Streaming callbacks are not implemented here because LangChain streaming
        APIs differ by provider/version. We still accept a stream_callback
        parameter to keep the call signature identical.
        """
        if not self._llm:
            raise RuntimeError("LangChain Ollama model is not initialized")

        lc_messages = []
        for m in messages:
            role = (m.get("role") or "user").lower()
            content = str(m.get("content", "") or "")
            if role == "system":
                lc_messages.append(SystemMessage(content=content))
            elif role == "user":
                lc_messages.append(HumanMessage(content=content))
            else:
                lc_messages.append(AIMessage(content=content))

        try:
            ai_msg = self._llm.predict_messages(lc_messages)  # type: ignore[attr-defined]
            return str(getattr(ai_msg, "content", "") or "").strip()
        except Exception as exc:
            # No silent fallback — log the error and return empty string so
            # the node can decide how to respond downstream.
            self.get_logger().error(f"LangChain call failed: {exc}")
            return ""

    def _generate_response(self, messages: List[Dict[str, str]], stream_callback: Optional[Callable[[str], None]] = None) -> str:
        try:
            return str(self._ollama_chat(messages, stream_callback=stream_callback) or "").strip()
        except Exception as exc:
            self.get_logger().error(f"_generate_response error: {exc}")
            return ""

    def _publish_user_turn(self, msg: MsgMessage) -> None:
        try:
            self.pub_conversation.publish(msg)
        except Exception:
            pass

    def on_conversation(self, msg: MsgMessage) -> None:
        self.history.append(
            {
                "role": msg.role,
                "content": msg.content,
                "speaker": getattr(msg, "speaker", ""),
                "confidence": float(getattr(msg, "confidence", 0.0) or 0.0),
            }
        )
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history :]

        if msg.role.lower().strip() != "user":
            return

        system_message = self._compose_system_message()
        messages: List[Dict[str, str]] = [{"role": "system", "content": system_message}]
        messages.extend(self.history)

        token_buffer: List[str] = []

        def _on_token(token: str) -> None:
            token_buffer.append(token)
            stream_msg = String()
            stream_msg.data = "".join(token_buffer)
            try:
                self.pub_stream.publish(stream_msg)
            except Exception:
                pass

        assistant_text = self._generate_response(messages, stream_callback=_on_token)
        if not assistant_text:
            self.get_logger().warning("Empty response from language model")
            return

        assistant_text = first_sentence(assistant_text)

        speak = String()
        speak.data = assistant_text
        try:
            self.pub_voice.publish(speak)
        except Exception:
            pass
        self.pending_to_confirm.append(assistant_text)

    def on_voice_done(self, msg: String) -> None:
        if not self.pending_to_confirm:
            return
        spoken = str(getattr(msg, "data", "") or "").strip()
        if not spoken:
            return
        expected = self.pending_to_confirm.pop(0)
        if _normalize_voice_text(spoken) != _normalize_voice_text(expected):
            self.get_logger().warning("voice_done did not match pending utterance; publishing spoken text anyway")

        recent_match = False
        if self.history:
            last = self.history[-1]
            if last.get("role") == "assistant" and _normalize_voice_text(str(last.get("content", ""))) == _normalize_voice_text(spoken):
                recent_match = True

        if not recent_match:
            out = MsgMessage()
            out.role = "assistant"
            out.content = spoken
            out.speaker = "assistant"
            out.confidence = 1.0
            try:
                self.pub_conversation.publish(out)
            except Exception:
                pass
            self.history.append({"role": "assistant", "content": spoken, "speaker": "assistant", "confidence": 1.0})
        else:
            if not self.history or self.history[-1].get("content") != spoken:
                self.history.append({"role": "assistant", "content": spoken, "speaker": "assistant", "confidence": 1.0})
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history :]

    def destroy_node(self) -> None:  # type: ignore[override]
        try:
            if self._serve_proc is not None:
                self._serve_proc.terminate()
        except Exception:
            pass
        super().destroy_node()


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


if __name__ == '__main__':  # pragma: no cover - entrypoint behaviour
    main()

