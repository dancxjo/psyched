#!/usr/bin/env python3
from __future__ import annotations

import importlib
import os
import time
import threading
import subprocess
import json
from types import ModuleType
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from psyched_msgs.msg import Message as MsgMessage, Transcript


def _normalize_voice_text(text: str) -> str:
    """Collapse whitespace for reliable voice acknowledgement matching."""

    return " ".join(text.split())

def first_sentence(text: str) -> str:
    s = text.strip()
    if not s:
        return s
    # Find first sentence terminator
    for i, ch in enumerate(s):
        if ch in ".!?":
            # Include the terminator
            return s[: i + 1].strip()
    # Fallback: return up to ~200 chars max to stay concise
    return (s[:200] + ("…" if len(s) > 200 else "")).strip()


def transcript_to_message(transcript: Transcript) -> Optional[MsgMessage]:
    """Convert a transcript into a chat ``Message`` while preserving metadata."""

    text = str(getattr(transcript, 'text', '')).strip()
    if not text:
        return None

    msg = MsgMessage()
    msg.role = 'user'
    msg.content = text
    speaker = getattr(transcript, 'speaker', '') or 'user'
    msg.speaker = str(speaker)
    try:
        msg.confidence = float(getattr(transcript, 'confidence', 0.0) or 0.0)
    except Exception:
        msg.confidence = 0.0
    return msg


class ChatNode(Node):

    def __init__(self) -> None:
        super().__init__('chat')

        # Parameters (no environment fallback, only YAML or launch params)
        self.declare_parameter('system_prompt', 'You are a helpful assistant. Always answer in one concise sentence.')
        self.declare_parameter('conversation_topic', '/conversation')
        self.declare_parameter('voice_topic', '/voice')
        self.declare_parameter('transcript_topic', '/audio/transcription')
        self.declare_parameter('model', 'llama3.2')
        self.declare_parameter('ollama_host', 'http://localhost:11434')
        self.declare_parameter('max_history', 20)

        # Resolve parameters
        self.system_prompt: str = self.get_parameter('system_prompt').get_parameter_value().string_value
        self.conversation_topic: str = self.get_parameter('conversation_topic').get_parameter_value().string_value
        self.voice_topic: str = self.get_parameter('voice_topic').get_parameter_value().string_value
        self.transcript_topic: str = self.get_parameter('transcript_topic').get_parameter_value().string_value
        self.model: str = self.get_parameter('model').get_parameter_value().string_value
        self.ollama_host: str = self.get_parameter('ollama_host').get_parameter_value().string_value.rstrip('/')
        self.max_history: int = int(self.get_parameter('max_history').get_parameter_value().integer_value or 20)

        # Publishers/subscribers
        self.pub_voice = self.create_publisher(String, self.voice_topic, 10)
        self.pub_conversation = self.create_publisher(MsgMessage, self.conversation_topic, 10)
        self.sub_conversation = self.create_subscription(MsgMessage, self.conversation_topic, self.on_conversation, 10)
        self.sub_voice_done = self.create_subscription(String, 'voice_done', self.on_voice_done, 10)
        self.sub_transcript = self.create_subscription(Transcript, self.transcript_topic, self._handle_transcript, 10)

        # State
        self.history: List[Dict[str, Any]] = []  # list of {role, content, ...}
        self.pending_to_confirm: List[str] = []  # queue of assistant texts awaiting voice_done
        self._serve_proc: subprocess.Popen | None = None
        self._http_missing_warned = False
        self._http: ModuleType | None = self._load_http_client()
        self._http_lock = threading.Lock()

        # Ensure Ollama service is reachable or try to start it
        self._ensure_ollama()

        self.get_logger().info(f"Chat node started. Model={self.model}, conversation={self.conversation_topic}, voice={self.voice_topic}")

    # --- Ollama helpers ---
    def _load_http_client(self) -> ModuleType | None:
        """Return the ``requests`` module if available, logging when missing."""
        try:
            return importlib.import_module('requests')
        except ModuleNotFoundError:
            self._http_missing_warned = True
            self.get_logger().warning(
                'Python requests not available; install it to enable HTTP calls to Ollama'
            )
        except Exception as exc:  # pragma: no cover - defensive logging
            self._http_missing_warned = True
            self.get_logger().warning(f'Failed to import requests: {exc}')
        return None

    def _ensure_ollama(self) -> None:
        if self._http is None:
            if not self._http_missing_warned:
                self.get_logger().warning(
                    'Python requests not available; install it to enable HTTP calls to Ollama'
                )
                self._http_missing_warned = True
            return
        if self._check_ollama():
            return
        # Try to start local server
        try:
            self.get_logger().info('Attempting to start local ollama serve...')
            self._serve_proc = subprocess.Popen(['ollama', 'serve'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # Wait briefly for port to come up
            for _ in range(20):
                if self._check_ollama():
                    self.get_logger().info('Ollama is now reachable')
                    break
                time.sleep(0.25)
            else:
                self.get_logger().warning('Ollama did not become reachable in time. Ensure the ollama service is running.')
        except FileNotFoundError:
            self.get_logger().warning('ollama binary not found in PATH. Install it and run `ollama serve`.')
        except Exception as e:
            self.get_logger().warning(f'Could not start ollama serve: {e}')

    def _check_ollama(self) -> bool:
        try:
            with self._http_lock:
                r = self._http.get(self.ollama_host + '/api/tags', timeout=1)
            return r.status_code == 200
        except Exception:
            return False

    def _ollama_chat(self, messages: List[Dict[str, str]]) -> str:
        if self._http is None:
            if not self._http_missing_warned:
                self.get_logger().warning(
                    'Python requests not available; install it to enable HTTP calls to Ollama'
                )
                self._http_missing_warned = True
            return ''
        # Try chat endpoint first, fallback to generate
        payload_chat = {
            'model': self.model,
            'messages': messages,
            'stream': False,
        }
        try:
            with self._http_lock:
                r = self._http.post(self.ollama_host + '/api/chat', json=payload_chat, timeout=60)
            if r.status_code == 200:
                data = r.json()
                # Newer responses: {'message': {'role':'assistant','content':'...'}} or {'choices':[{'message':{...}}]}
                if isinstance(data, dict):
                    if 'message' in data and isinstance(data['message'], dict):
                        return str(data['message'].get('content', '')).strip()
                    if 'choices' in data and data['choices']:
                        return str(data['choices'][0].get('message', {}).get('content', '')).strip()
        except Exception as e:
            self.get_logger().warning(f'/api/chat failed: {e}')

        # Fallback: concatenate into a prompt for generate
        prompt = []
        for m in messages:
            role = m.get('role', 'user')
            content = m.get('content', '')
            if role == 'system':
                prompt.append(f"[SYSTEM]\n{content}\n")
            elif role == 'user':
                prompt.append(f"User: {content}\n")
            else:
                prompt.append(f"Assistant: {content}\n")
        prompt.append('Assistant:')
        payload_gen = {
            'model': self.model,
            'prompt': "\n".join(prompt),
            'stream': False,
        }
        try:
            with self._http_lock:
                r = self._http.post(self.ollama_host + '/api/generate', json=payload_gen, timeout=60)
            if r.status_code == 200:
                data = r.json()
                return str(data.get('response', '')).strip()
        except Exception as e:
            self.get_logger().error(f'Ollama generate failed: {e}')
        return ''

    # --- ROS callbacks ---
    def _publish_user_turn(self, msg: MsgMessage) -> None:
        self.pub_conversation.publish(msg)

    def _handle_transcript(self, transcript: Transcript) -> None:
        msg = transcript_to_message(transcript)
        if msg is None:
            return
        if not msg.speaker:
            msg.speaker = transcript.speaker or 'user'
        self._publish_user_turn(msg)

    def on_conversation(self, msg: MsgMessage) -> None:
        # Append to history
        self.history.append({
            'role': msg.role,
            'content': msg.content,
            'speaker': getattr(msg, 'speaker', ''),
            'confidence': float(getattr(msg, 'confidence', 0.0) or 0.0),
        })
        # Trim
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history :]

        if msg.role.lower().strip() != 'user':
            return

        # Build messages with system on top
        messages: List[Dict[str, str]] = [{'role': 'system', 'content': self.system_prompt}]
        messages.extend(self.history)

        # Query model
        assistant_text = self._ollama_chat(messages)
        if not assistant_text:
            self.get_logger().warning('Empty response from model')
            return

        # Enforce one sentence
        assistant_text = first_sentence(assistant_text)

        # Publish to voice for speaking
        speak = String()
        speak.data = assistant_text
        self.pub_voice.publish(speak)
        # Queue for confirmation
        self.pending_to_confirm.append(assistant_text)

    def on_voice_done(self, msg: String) -> None:
        if not self.pending_to_confirm:
            return
        spoken_raw = str(getattr(msg, 'data', '') or '')
        spoken = spoken_raw.strip()
        if not spoken:
            return
        head = self.pending_to_confirm.pop(0)
        if _normalize_voice_text(spoken) != _normalize_voice_text(head):
            self.get_logger().warning(
                'voice_done payload did not match pending utterance; publishing spoken text anyway'
            )
        # Avoid duplicating assistant messages if another publisher (the voice node)
        # already published the assistant turn with identical content. Check the
        # most recent history item for a matching assistant entry. If present,
        # do not republish but ensure history is consistent.
        recent_match = False
        if self.history:
            last = self.history[-1]
            if last.get('role') == 'assistant' and _normalize_voice_text(str(last.get('content', ''))) == _normalize_voice_text(spoken):
                recent_match = True

        if not recent_match:
            out = MsgMessage()
            out.role = 'assistant'
            out.content = spoken
            out.speaker = 'assistant'
            out.confidence = 1.0
            self.pub_conversation.publish(out)
            # Also keep history consistent
            self.history.append({'role': 'assistant', 'content': spoken, 'speaker': 'assistant', 'confidence': 1.0})
        else:
            # Ensure history contains this assistant turn (it should already), but
            # if for some reason it wasn't appended, add it now.
            if not self.history or self.history[-1].get('content') != spoken:
                self.history.append({'role': 'assistant', 'content': spoken, 'speaker': 'assistant', 'confidence': 1.0})
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history :]


def main(argv=None):
    rclpy.init(args=argv)
    node = ChatNode()
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node._serve_proc is not None:
                node._serve_proc.terminate()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
