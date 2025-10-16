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

# Use the official `ollama` python client directly. Fail fast if missing so
# deployments know to install it. We'll use streaming if the client exposes it.
logger = logging.getLogger("psyched.chat.node")
if not logger.handlers:
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [chat.node] %(message)s"))
    logger.addHandler(handler)
logger.setLevel(logging.DEBUG)

logger.info("Importing ollama client")
try:  # pragma: no cover - import compatibility shim
    from ollama import Ollama  # type: ignore
except ImportError:  # pragma: no cover
    import ollama  # type: ignore

    class Ollama:  # type: ignore[override]
        """Compatibility wrapper for older code expecting ``ollama.Ollama``."""

        def __init__(self, *args, **kwargs) -> None:  # noqa: ANN003, D401
            self._client = ollama.Client(*args, **kwargs)

        def __getattr__(self, name: str):
            return getattr(self._client, name)

import requests  # type: ignore


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

        self.model: str = os.environ.get("OLLAMA_MODEL", "gpt-oss:20b")
        self.ollama_host: str = _normalise_ollama_host(os.environ.get("OLLAMA_HOST", ""))

        # Initialize Ollama client (direct). Ollama client API may accept a
        # host/base_url; set up a client instance and detect streaming support.
        self._client = None
        try:
            # The `ollama` package exposes an Ollama client which accepts a
            # `host` kwarg in some versions. Try constructing it; if it fails
            # the exception will propagate (we want fast failure).
            try:
                self._client = Ollama(host=self.ollama_host or None)
            except TypeError:
                # older/newer client variants may differ; try no-arg
                self._client = Ollama()
            logger.info(f"Ollama client initialized, model={self.model}, host={self.ollama_host}")
        except Exception as exc:
            logger.exception("Failed to initialize Ollama client")
            raise

        self._serve_proc = None
        # detect streaming capability (we'll attempt to call `stream` if present)
        self._supports_stream = hasattr(self._client, "stream") or hasattr(self._client, "create_stream")
        logger.debug(f"Ollama streaming supported: {self._supports_stream}")

        # wrapper used by _generate_response
        self._ollama_chat = lambda messages, stream_callback=None: self._call_ollama(messages, stream_callback=stream_callback)

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

    def _call_ollama(self, messages: List[Dict[str, str]], stream_callback: Optional[Callable[[str], None]] = None) -> str:
        """Call Ollama directly using the `ollama` client.

        messages is a list of {"role":"system|user|assistant","content":"..."}.
        If streaming is supported and a stream_callback is given, emit tokens
        to the callback as they arrive; otherwise return the full response.
        """
        if not self._client:
            raise RuntimeError("Ollama client not initialized")

        # Build a simple prompt for the Ollama chat model. Ollama's Python
        # client APIs vary; many environments provide a `chat` or `generate`
        # method. We'll try a few common names and raise if none are present.
        payload = {"model": self.model, "messages": messages}

        logger.debug("Calling Ollama with payload: %s", payload)

        # Streaming path
        try:
            if stream_callback and self._supports_stream:
                # Try client.stream(...) signature
                if hasattr(self._client, "stream"):
                    for chunk in self._client.stream(model=self.model, messages=messages):
                        token = str(chunk or "")
                        stream_callback(token)
                    return ""
                # some clients may expose create_stream
                if hasattr(self._client, "create_stream"):
                    for chunk in self._client.create_stream(model=self.model, messages=messages):
                        token = str(chunk or "")
                        stream_callback(token)
                    return ""
        except Exception as exc:
            logger.exception("Ollama streaming failed: %s", exc)

        # Non-streaming path: try client.chat/generate/predict
        try:
            if hasattr(self._client, "chat"):
                resp = self._client.chat(model=self.model, messages=messages)
            elif hasattr(self._client, "generate"):
                resp = self._client.generate(model=self.model, messages=messages)
            elif hasattr(self._client, "predict"):
                resp = self._client.predict(model=self.model, messages=messages)
            else:
                # Last resort: call the local Ollama HTTP API directly via requests
                if not self.ollama_host:
                    raise RuntimeError("No ollama host configured and client lacks chat API")
                url = self.ollama_host.rstrip("/") + "/api/chat"
                r = requests.post(url, json={"model": self.model, "messages": messages}, timeout=30)
                r.raise_for_status()
                data = r.json()
                # try common response shapes
                if isinstance(data, dict):
                    if isinstance(data.get("message"), dict):
                        return str(data["message"].get("content", "")).strip()
                    choices = data.get("choices") or []
                    if choices and isinstance(choices[0], dict):
                        return str(choices[0].get("message", {}).get("content", "")).strip()
                return ""

            # Extract text depending on response shape
            if isinstance(resp, str):
                return resp.strip()
            if hasattr(resp, "text"):
                return str(resp.text or "").strip()
            # some clients return an object with 'content' attribute
            if hasattr(resp, "content"):
                return str(getattr(resp, "content", "") or "").strip()
            # fallback to stringifying
            return str(resp or "").strip()
        except Exception as exc:
            logger.exception("Ollama call failed: %s", exc)
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

