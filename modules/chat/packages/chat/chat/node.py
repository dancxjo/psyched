#!/usr/bin/env python3
from __future__ import annotations

import os
import time
import threading
from typing import Any, Callable, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from psyched_msgs.msg import Message as MsgMessage, Transcript


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
    text = str(getattr(transcript, 'text', '')).strip()
    if not text:
        return None
    msg = MsgMessage()


    def _publish_user_turn(self, msg: MsgMessage) -> None:
        self.pub_conversation.publish(msg)

    def _handle_transcript(self, transcript: Transcript) -> None:
        msg = transcript_to_message(transcript)
        if msg is None:
            return
        if not msg.speaker:
            msg.speaker = transcript.speaker or 'user'
        self._publish_user_turn(msg)

    # ------------------------------------------------------------------
    # LangChain helpers
    def _build_chat_messages(self, messages: Sequence[Dict[str, str]]) -> Optional[List[BaseMessage]]:
        if BaseMessage is None:
            return None
        chat_messages: List[BaseMessage] = []
        for message in messages:
            role = str(message.get('role', 'user')).strip().lower() or 'user'
            content = str(message.get('content', '')).strip()
            if not content:
                continue
            if role == 'system':
                chat_messages.append(SystemMessage(content=content))
            elif role == 'assistant':
                chat_messages.append(AIMessage(content=content))
            else:
                chat_messages.append(HumanMessage(content=content))
        return chat_messages

    def _messages_to_prompt(self, messages: Sequence[Dict[str, str]]) -> str:
        prompt_lines: List[str] = []
        for message in messages:
            role = message.get('role', 'user')
            content = message.get('content', '')
            if role == 'system':
                prompt_lines.append(f"[SYSTEM]\n{content}\n")
            elif role == 'assistant':
                prompt_lines.append(f"Assistant: {content}\n")
            else:
                prompt_lines.append(f"User: {content}\n")
        prompt_lines.append('Assistant:')
        return "\n".join(prompt_lines)

    def _chunk_text(self, chunk: Any) -> str:
        if isinstance(chunk, str):
            return chunk
        for attr in ('content', 'text'):
            value = getattr(chunk, attr, None)
            if isinstance(value, str):
                return value
        message = getattr(chunk, 'message', None)
        if message is not None:
            for attr in ('content', 'text'):
                value = getattr(message, attr, None)
                if isinstance(value, str):
                    return value
        return ''

    def _extract_text(self, result: Any) -> str:
        if isinstance(result, str):
            return result.strip()
        for attr in ('content', 'text'):
            value = getattr(result, attr, None)
            if isinstance(value, str):
                return value.strip()
        message = getattr(result, 'message', None)
        if message is not None:
            for attr in ('content', 'text'):
                value = getattr(message, attr, None)
                if isinstance(value, str):
                    return value.strip()
        return ''

    def _call_chat_model(
        self,
        messages: Sequence[Dict[str, str]],
        stream_callback: Optional[Callable[[str], None]],
    ) -> str:
        if self._llm is None:
            return ''

        lc_messages = self._build_chat_messages(messages)
        if not lc_messages:
            return ''

        collected: List[str] = []
        if stream_callback and self._supports_stream and hasattr(self._llm, 'stream'):
            try:
                for chunk in self._llm.stream(lc_messages):  # type: ignore[call-arg]
                    token = self._chunk_text(chunk)
                    if not token:
                        continue
                    collected.append(token)
                    stream_callback(token)
                if collected:
                    return ''.join(collected).strip()
            except Exception as exc:
                self.get_logger().warning(f'Chat streaming failed; retrying without stream: {exc}')
                collected.clear()

        try:
            if hasattr(self._llm, 'invoke'):
                result = self._llm.invoke(lc_messages)  # type: ignore[call-arg]
            else:
                result = self._llm(lc_messages)  # type: ignore[call-arg]
        except Exception as exc:
            self.get_logger().warning(f'LangChain chat invocation failed: {exc}')
            return ''

        text = self._extract_text(result)
        if text and stream_callback and not collected:
            stream_callback(text)
        return text

    def _call_text_model(
        self,
        messages: Sequence[Dict[str, str]],
        stream_callback: Optional[Callable[[str], None]],
    ) -> str:
        if self._llm is None:
            return ''
        prompt = self._messages_to_prompt(messages)
        collected: List[str] = []
        if stream_callback and self._supports_stream and hasattr(self._llm, 'stream'):
            try:
                for chunk in self._llm.stream(prompt):  # type: ignore[call-arg]
                    token = self._chunk_text(chunk)
                    if not token:
                        continue
                    collected.append(token)
                    stream_callback(token)
                if collected:
                    return ''.join(collected).strip()
            except Exception as exc:
                self.get_logger().warning(f'LLM streaming failed; retrying without stream: {exc}')
                collected.clear()
        try:
            if hasattr(self._llm, 'invoke'):
                result = self._llm.invoke(prompt)  # type: ignore[call-arg]
            else:
                result = self._llm(prompt)  # type: ignore[call-arg]
        except Exception as exc:
            self.get_logger().warning(f'LangChain LLM invocation failed: {exc}')
            return ''
        text = self._extract_text(result)
        if text and stream_callback and not collected:
            stream_callback(text)
        return text

    def _check_ollama(self) -> bool:
        if not self.ollama_host or self._http is None:
            return False
        try:
            with self._http_lock:
                response = self._http.get(self.ollama_host + '/api/tags', timeout=1)
            return response.status_code == 200
        except Exception:
            return False

    def _ensure_ollama(self) -> None:
        if not self.ollama_host or self._http is None:
            return
        if self._check_ollama():
            return
        try:
            self.get_logger().info('Attempting to start local Ollama service...')
            self._serve_proc = subprocess.Popen(['ollama', 'serve'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            for _ in range(20):
                if self._check_ollama():
                    self.get_logger().info('Ollama service became reachable')
                    return
                time.sleep(0.25)
            self.get_logger().warning('Ollama did not become reachable in time; ensure the service is running.')
        except FileNotFoundError:
            self.get_logger().warning('ollama binary not found in PATH. Install it and run `ollama serve`.')
        except Exception as exc:
            self.get_logger().warning(f'Could not start Ollama: {exc}')

    def _ollama_http(self, messages: Sequence[Dict[str, str]]) -> str:
        if not self.ollama_host:
            self.get_logger().warning('Ollama host not configured; skipping HTTP fallback response')
            return ''
        if self._http is None:
            if not self._http_missing_warned:
                self.get_logger().warning('Install python-requests to enable the HTTP fallback path.')
                self._http_missing_warned = True
            return ''

        payload = {'model': self.model, 'messages': list(messages), 'stream': False}
        try:
            with self._http_lock:
                response = self._http.post(self.ollama_host + '/api/chat', json=payload, timeout=60)
            if response.status_code == 200:
                data = response.json()
                if isinstance(data, dict):
                    if isinstance(data.get('message'), dict):
                        return str(data['message'].get('content', '')).strip()
                    choices = data.get('choices') or []
                    if choices and isinstance(choices[0], dict):
                        return str(choices[0].get('message', {}).get('content', '')).strip()
        except Exception as exc:
            self.get_logger().warning(f'Ollama chat endpoint failed: {exc}')

        prompt = self._messages_to_prompt(messages)
        try:
            payload = {'model': self.model, 'prompt': prompt, 'stream': False}
            with self._http_lock:
                response = self._http.post(self.ollama_host + '/api/generate', json=payload, timeout=60)
            if response.status_code == 200:
                data = response.json()
                return str(data.get('response', '')).strip()
        except Exception as exc:
            self.get_logger().error(f'Ollama generate failed: {exc}')
        return ''

    def _generate_response(
        self,
        messages: Sequence[Dict[str, str]],
        stream_callback: Optional[Callable[[str], None]] = None,
    ) -> str:
        if self._llm is not None:
            if self._llm_is_chat_model:
                response = self._call_chat_model(messages, stream_callback)
            else:
                response = self._call_text_model(messages, stream_callback)
            if response:
                return response.strip()
        return self._ollama_http(messages)

    # ------------------------------------------------------------------
    # ROS callbacks
    def on_conversation(self, msg: MsgMessage) -> None:
        self.history.append(
            {
                'role': msg.role,
                'content': msg.content,
                'speaker': getattr(msg, 'speaker', ''),
                'confidence': float(getattr(msg, 'confidence', 0.0) or 0.0),
            }
        )
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history :]

        if msg.role.lower().strip() != 'user':
            return

        system_message = self._compose_system_message()
        messages: List[Dict[str, str]] = [{'role': 'system', 'content': system_message}]
        messages.extend(self.history)

        token_buffer: List[str] = []

        def _on_token(token: str) -> None:
            token_buffer.append(token)
            stream_msg = String()
            stream_msg.data = ''.join(token_buffer)
            try:
                self.pub_stream.publish(stream_msg)
            except Exception:
                pass

        assistant_text = self._generate_response(messages, stream_callback=_on_token)
        if not assistant_text:
            self.get_logger().warning('Empty response from language model')
            return

        assistant_text = first_sentence(assistant_text)

        speak = String()
        speak.data = assistant_text
        self.pub_voice.publish(speak)
        self.pending_to_confirm.append(assistant_text)

    def on_voice_done(self, msg: String) -> None:
        if not self.pending_to_confirm:
            return
        spoken = str(getattr(msg, 'data', '') or '').strip()
        if not spoken:
            return
        expected = self.pending_to_confirm.pop(0)
        if _normalize_voice_text(spoken) != _normalize_voice_text(expected):
            self.get_logger().warning('voice_done did not match pending utterance; publishing spoken text anyway')

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
            self.history.append({'role': 'assistant', 'content': spoken, 'speaker': 'assistant', 'confidence': 1.0})
        else:
            if not self.history or self.history[-1].get('content') != spoken:
                self.history.append({'role': 'assistant', 'content': spoken, 'speaker': 'assistant', 'confidence': 1.0})
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history :]

    # ------------------------------------------------------------------
    # Shutdown helpers
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
