#!/usr/bin/env python3
from __future__ import annotations

import logging
import os
import sys
from typing import Any, Callable, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from psyched_msgs.msg import Message as MsgMessage, Transcript

logger = logging.getLogger("psyched.chat.node")
if not logger.handlers:
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [chat.node] %(message)s"))
    logger.addHandler(handler)
logger.setLevel(logging.DEBUG)

import requests  # type: ignore

from .ollama_client import OllamaServiceClient, OllamaServiceError


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
    msg.role = "user"
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

        model_param = self.declare_parameter("model", os.environ.get("OLLAMA_MODEL", "gpt-oss:20b"))
        self.model = (self._extract_parameter_str(model_param, "gpt-oss:20b") or "gpt-oss:20b").strip()
        if not self.model:
            self.model = "gpt-oss:20b"

        host_param = self.declare_parameter("ollama_host", os.environ.get("OLLAMA_HOST", ""))
        configured_host = self._extract_parameter_str(host_param, "").strip()
        env_host = str(os.environ.get("OLLAMA_HOST", "")).strip()
        candidate_host = _normalise_ollama_host(configured_host or env_host)
        self.ollama_host = candidate_host or "http://127.0.0.1:11434"
        logger.info("Using Ollama service at %s", self.ollama_host)

        self._ollama_client = OllamaServiceClient(host=self.ollama_host, logger=logger)
        self._ollama_chat = lambda messages, stream_callback=None: self._call_ollama(  # noqa: E731
            messages,
            stream_callback=stream_callback,
        )

    @staticmethod
    def _extract_parameter_str(parameter: Any, default: str) -> str:
        try:
            if hasattr(parameter, "value"):
                raw = getattr(parameter, "value")
            elif hasattr(parameter, "get_parameter_value"):
                value = parameter.get_parameter_value()
                for attr in ("string_value", "double_value", "integer_value"):
                    if hasattr(value, attr):
                        candidate = getattr(value, attr)
                        if isinstance(candidate, (str, int, float)):
                            return str(candidate)
                return str(value)
            else:
                raw = parameter
            if raw is None:
                return default
            return str(raw)
        except Exception:
            return default

    def _compose_system_message(self) -> str:
        base = "You are a helpful assistant."
        try:
            cockpit_base = os.environ.get("COCKPIT_BASE", "http://localhost:8080")
            url = cockpit_base.rstrip("/") + "/api/modules"
            # Query Cockpit's control surface (optional). Tests inject a fake
            # `requests` module into sys.modules; prefer that at runtime so
            # tests remain hermetic even if import-time resolution differs.
            requests_mod = sys.modules.get("requests")
            if requests_mod is not None:
                resp = requests_mod.get(url, timeout=2)
                if resp and getattr(resp, "status_code", 200) == 200:
                    data = resp.json()
                    modules = data.get("modules", []) if isinstance(data, dict) else []
                    for mod in modules:
                        if mod.get("name") == "cockpit" or mod.get("display_name", "").lower().startswith("cockpit"):
                            display = mod.get("display_name", "Cockpit Control Surface")
                            desc = mod.get("description", "")
                            base += f"\n\nCockpit Control Surface:\n{display}\n{desc}\n"
                            break
        except Exception:
            pass
        return base

    def _call_ollama(
        self,
        messages: List[Dict[str, str]],
        stream_callback: Optional[Callable[[str], None]] = None,
    ) -> str:
        try:
            return self._ollama_client.chat(
                model=self.model,
                messages=messages,
                stream_callback=stream_callback,
            )
        except OllamaServiceError as exc:
            self.get_logger().error(f"Ollama service unavailable: {exc}")
            return ""
        except Exception as exc:
            self.get_logger().error(f"Unexpected Ollama error: {exc}")
            return ""

    def _generate_response(self, messages: List[Dict[str, str]], stream_callback: Optional[Callable[[str], None]] = None) -> str:
        try:
            # Call the configured chat wrapper. Some tests monkeypatch
            # _ollama_chat with a single-argument callable (messages), while
            # production wrapper accepts (messages, stream_callback). Try the
            # full signature first and fall back to the single-arg form.
            try:
                result = self._ollama_chat(messages, stream_callback=stream_callback)
            except TypeError:
                result = self._ollama_chat(messages)
            return str(result or "").strip()
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

        try:
            assistant_text = self._generate_response(messages, stream_callback=_on_token)
        except TypeError:
            # Some tests replace _generate_response with a single-arg callable.
            assistant_text = self._generate_response(messages)
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

