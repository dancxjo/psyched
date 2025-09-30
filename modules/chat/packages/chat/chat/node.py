#!/usr/bin/env python3
from __future__ import annotations

import importlib
import os
import time
import threading
import subprocess
from types import ModuleType
from typing import Any, Dict, List, Optional, Mapping


def _normalise_ollama_host(raw_host: str) -> str:
    """Return a canonical Ollama host string suitable for HTTP requests.

    The fallback accepts configuration from ROS parameters or ``OLLAMA_HOST``.
    Operators frequently export ``OLLAMA_HOST=forebrain.local:11434`` (without a
    scheme or trailing slash). The chat node should tolerate those variants so
    that failing back to Ollama Just Works™, even when the websocket LLM is
    offline. Examples::

        >>> _normalise_ollama_host('http://forebrain.local:11434/')
        'http://forebrain.local:11434'
        >>> _normalise_ollama_host('forebrain.local:11434')
        'http://forebrain.local:11434'

    An empty string disables the fallback and is preserved as-is.
    """

    host = (raw_host or "").strip()
    if not host:
        return ""
    if not host.startswith(("http://", "https://")):
        host = f"http://{host}"
    return host.rstrip("/")

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
    try:
        msg.segments = list(getattr(transcript, 'segments', []) or [])
    except Exception:
        msg.segments = []  # type: ignore[assignment]
    try:
        msg.words = list(getattr(transcript, 'words', []) or [])
    except Exception:
        msg.words = []  # type: ignore[assignment]
    return msg


class ChatNode(Node):

    def __init__(self) -> None:
        super().__init__('chat')

        # Parameters (no environment fallback, only YAML or launch params)
        self.declare_parameter('system_prompt', 'You are a helpful assistant. Always answer in one concise sentence.')
        self.declare_parameter('conversation_topic', '/conversation')
        self.declare_parameter('voice_topic', '/voice')
        self.declare_parameter('transcript_topic', '/audio/transcription')
        # Default to a tiny local Ollama model so the fallback stays responsive.
        # The remote speech stack supplies the heavier GPT-OSS:20B model.
        default_ollama_host = _normalise_ollama_host(
            os.getenv('CHAT_OLLAMA_HOST', os.getenv('OLLAMA_HOST', 'http://forebrain.local:11434'))
        )
        if not default_ollama_host:
            default_ollama_host = 'http://forebrain.local:11434'
        self.declare_parameter('model', 'gpt-oss:20b')
        self.declare_parameter('ollama_host', default_ollama_host)
        self.declare_parameter('max_history', 20)
        self.declare_parameter('pilot_base_url', 'http://forebrain.local:8080')
        self.declare_parameter('pilot_text_cache_ttl', 5.0)

        # Resolve parameters
        self.system_prompt: str = self.get_parameter('system_prompt').get_parameter_value().string_value
        self.conversation_topic: str = self.get_parameter('conversation_topic').get_parameter_value().string_value
        self.voice_topic: str = self.get_parameter('voice_topic').get_parameter_value().string_value
        self.transcript_topic: str = self.get_parameter('transcript_topic').get_parameter_value().string_value
        self.model: str = self.get_parameter('model').get_parameter_value().string_value
        resolved_ollama_host = self.get_parameter('ollama_host').get_parameter_value().string_value
        self.ollama_host: str = _normalise_ollama_host(resolved_ollama_host)
        if not self.ollama_host:
            self.get_logger().warning('Ollama host not configured; fallback disabled.')
        self.max_history: int = int(self.get_parameter('max_history').get_parameter_value().integer_value or 20)
        pilot_base = (
            self.get_parameter('pilot_base_url').get_parameter_value().string_value.strip()
        )
        self._pilot_modules_url = pilot_base.rstrip('/') + '/api/modules' if pilot_base else ''
        ttl_value = self.get_parameter('pilot_text_cache_ttl').get_parameter_value().double_value or 5.0
        self._pilot_text_cache_ttl = max(1.0, float(ttl_value))
        self._pilot_text_cache = ''
        self._pilot_text_expiry = 0.0
        self._pilot_text_warned = False

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

    # --- Pilot helpers ---
    def _compose_system_message(self) -> str:
        """Return the system prompt with a pilot text digest appended."""

        parts: List[str] = []
        base_prompt = str(self.system_prompt or '').strip()
        if base_prompt:
            parts.append(base_prompt)
        pilot_summary = self._get_pilot_text()
        if pilot_summary:
            parts.append(pilot_summary.strip())
        return '\n\n'.join(parts) if parts else ''

    def _get_pilot_text(self) -> str:
        """Fetch and cache a text-only summary of the pilot UI."""

        if not self._pilot_modules_url or self._http is None:
            return self._pilot_text_cache
        now = time.monotonic()
        if now < self._pilot_text_expiry:
            return self._pilot_text_cache
        try:
            with self._http_lock:
                response = self._http.get(self._pilot_modules_url, timeout=2.0)
            if response.status_code != 200:
                if not self._pilot_text_warned:
                    self.get_logger().warning(
                        f'Pilot summary unavailable ({response.status_code}) from {self._pilot_modules_url}'
                    )
                    self._pilot_text_warned = True
                self._pilot_text_cache = ''
            else:
                payload = response.json()
                self._pilot_text_cache = self._render_pilot_text(payload)
                self._pilot_text_warned = False
        except Exception as exc:  # pragma: no cover - network dependent
            if not self._pilot_text_warned:
                self.get_logger().warning(f'Failed to fetch pilot summary: {exc}')
                self._pilot_text_warned = True
            self._pilot_text_cache = ''
        self._pilot_text_expiry = now + self._pilot_text_cache_ttl
        return self._pilot_text_cache

    def _render_pilot_text(self, payload: Any) -> str:
        """Convert the pilot module payload into a readable text digest.

        The digest mirrors the Pilot control surface layout so the language
        model receives the same module ordering and context operators see in
        the browser UI. Each module entry includes its description, regimes,
        supported commands, and declared topics. For example, a payload with
        a single Pilot module produces output resembling::

            --- Pilot Control Surface (text-only digest) ---
            Module: Pilot
              Description: Teleoperation dashboard
              Regimes: system
              Module commands: setup, restart
              Topics:
                - /cmd_vel | geometry_msgs/msg/Twist | access=rw | view=joystick

        """

        header = '--- Pilot Control Surface (text-only digest) ---'
        modules: List[Mapping[str, Any]] = []
        if isinstance(payload, Mapping):
            raw_modules = payload.get('modules')
            if isinstance(raw_modules, list):
                modules = [m for m in raw_modules if isinstance(m, Mapping)]

        if not modules:
            return header + '\nPilot status: unavailable.'

        lines: List[str] = [header]
        for module in modules:
            name = str(module.get('display_name') or module.get('name') or 'Module').strip()
            lines.append(f'Module: {name}')
            description = str(module.get('description') or '').strip()
            if description:
                lines.append(f'  Description: {description}')
            regimes = module.get('regimes')
            if isinstance(regimes, list):
                regimes_text = ', '.join(str(item).strip() for item in regimes if str(item).strip())
                if regimes_text:
                    lines.append(f'  Regimes: {regimes_text}')
            commands = module.get('commands')
            if isinstance(commands, Mapping):
                mod_cmds = commands.get('mod')
                if isinstance(mod_cmds, list) and mod_cmds:
                    lines.append('  Module commands: ' + ', '.join(str(cmd).strip() for cmd in mod_cmds if str(cmd).strip()))
                sys_cmds = commands.get('system')
                if isinstance(sys_cmds, list) and sys_cmds:
                    lines.append('  System commands: ' + ', '.join(str(cmd).strip() for cmd in sys_cmds if str(cmd).strip()))
            topics = module.get('topics')
            if isinstance(topics, list) and topics:
                lines.append('  Topics:')
                for topic in topics:
                    if not isinstance(topic, Mapping):
                        continue
                    topic_name = str(topic.get('topic') or topic.get('name') or 'topic').strip() or 'topic'
                    type_name = str(topic.get('type') or topic.get('message_type') or '').strip()
                    access = str(topic.get('access') or 'ro').strip()
                    presentation = str(topic.get('presentation') or '').strip()
                    details = [topic_name]
                    if type_name:
                        details.append(type_name)
                    details.append(f'access={access}')
                    if presentation:
                        details.append(f'view={presentation}')
                    lines.append('    - ' + ' | '.join(details))
        return '\n'.join(lines)

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
        if not self.ollama_host:
            return
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

    def _generate_response(self, messages: List[Dict[str, str]]) -> str:
        return self._ollama_chat(messages)

    def _check_ollama(self) -> bool:
        if not self.ollama_host:
            return False
        try:
            with self._http_lock:
                r = self._http.get(self.ollama_host + '/api/tags', timeout=1)
            return r.status_code == 200
        except Exception:
            return False

    def _ollama_chat(self, messages: List[Dict[str, str]]) -> str:
        if not self.ollama_host:
            self.get_logger().warning('Ollama host not configured; skipping fallback response')
            return ''
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
        system_message = self._compose_system_message()
        messages: List[Dict[str, str]] = [{'role': 'system', 'content': system_message}]
        messages.extend(self.history)

        # Query model
        assistant_text = self._generate_response(messages)
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
