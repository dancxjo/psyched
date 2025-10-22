"""Transcription backend implementations for the ear module."""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import queue
import sys
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Protocol, TextIO, runtime_checkable

_LOGGER = logging.getLogger(__name__)

PublishCallback = Callable[[str], None]


class BackendUnavailableError(RuntimeError):
    """Raised when a backend cannot connect to an upstream dependency."""


class EarBackend(Protocol):
    """Protocol implemented by all ear transcription backends."""

    def run(self, publish: PublishCallback, stop_event: threading.Event) -> None:
        """Start consuming input and invoke ``publish`` for recognised text."""


@runtime_checkable
class AudioAwareBackend(EarBackend, Protocol):
    """Protocol for backends that accept PCM audio input."""

    def submit_audio(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Supply raw PCM audio data to the backend."""

    def close(self) -> None:  # pragma: no cover - optional shutdown hook
        """Release backend resources. Implementations may override."""


@dataclass(slots=True)
class ConsoleEarBackend:
    """Simple backend that reads lines from a text stream.

    Parameters
    ----------
    input_stream:
        Stream from which user input is read. Defaults to :data:`sys.stdin`.
    output_stream:
        Optional stream used to render a coloured prompt. Defaults to
        :data:`sys.stdout`.
    prompt:
        Prompt rendered before waiting for user input. Defaults to ``"🦻  "``.
    colour:
        ANSI escape sequence applied to the prompt. Defaults to cyan.

    Example
    -------
    >>> backend = ConsoleEarBackend()
    >>> stop_event = threading.Event()
    >>> backend.run(print, stop_event)  # doctest: +SKIP
    """

    input_stream: TextIO = sys.stdin
    output_stream: TextIO | None = sys.stdout
    prompt: str = "🦻  "
    colour: str = "\x1b[36m"
    _reset: str = field(default="\x1b[0m", init=False, repr=False)

    def run(self, publish: PublishCallback, stop_event: threading.Event) -> None:
        """Continuously read lines and publish non-empty entries."""

        while not stop_event.is_set():
            if self.output_stream is not None:
                self.output_stream.write(f"{self.colour}{self.prompt}{self._reset}")
                self.output_stream.flush()
            try:
                line = self.input_stream.readline()
            except Exception:  # pragma: no cover - defensive logging
                _LOGGER.exception("Failed to read from console input")
                break
            if line == "":
                break
            text = line.strip()
            if not text:
                continue
            publish(text)


class FasterWhisperEarBackend(AudioAwareBackend):
    """Backend that performs offline transcription via `faster-whisper`.

    Notes
    -----
    The backend lazily instantiates :class:`faster_whisper.WhisperModel` to
    avoid importing heavy dependencies when unused. Audio chunks must be
    supplied via :meth:`submit_audio` using 16-bit PCM samples.
    """

    def __init__(
        self,
        model_size: str = "base",
        *,
        device: str = "cpu",
        compute_type: str = "int8",
        language: str | None = None,
        beam_size: int | None = None,
    ) -> None:
        self._model_size = model_size
        self._device = device
        self._compute_type = compute_type
        self._language = language
        self._beam_size = beam_size
        self._queue: "queue.Queue[tuple[bytes, int, int] | None]" = queue.Queue()
        self._model = None

    def submit_audio(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Queue audio for transcription."""

        self._queue.put((pcm, sample_rate, channels))

    def close(self) -> None:
        """Wake the worker so it can terminate promptly."""

        self._queue.put(None)

    def run(self, publish: PublishCallback, stop_event: threading.Event) -> None:
        """Continuously transcribe queued audio segments."""

        try:
            from faster_whisper import WhisperModel  # type: ignore[import-not-found]
            import numpy as np  # type: ignore[import-not-found]
        except ImportError as error:  # pragma: no cover - optional dependency
            _LOGGER.error(
                "faster-whisper backend requested but dependencies are missing. "
                "Install them with `psh mod pip ear` (preferred) or "
                "`python3 -m pip install --break-system-packages faster-whisper` before retrying.",
            )
            raise RuntimeError(
                "faster-whisper backend requested but dependencies are missing; install via "
                "`psh mod pip ear` or `python3 -m pip install --break-system-packages faster-whisper`"
            ) from error

        if self._model is None:
            self._model = WhisperModel(
                self._model_size,
                device=self._device,
                compute_type=self._compute_type,
            )

        while not stop_event.is_set():
            try:
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            pcm, sample_rate, channels = item
            if channels != 1:
                _LOGGER.warning(
                    f"faster-whisper backend currently expects mono audio; received {channels} channels",
                )
            audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
            segments, _ = self._model.transcribe(
                audio,
                language=self._language,
                beam_size=self._beam_size,
                temperature=0.0,
                vad_filter=True,
                vad_parameters={"min_speech_duration_ms": 250},
            )
            transcript_parts = [segment.text.strip() for segment in segments if segment.text]
            transcript = " ".join(filter(None, transcript_parts)).strip()
            if transcript:
                publish(transcript)

        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break


class ServiceASREarBackend(AudioAwareBackend):
    """Backend that streams audio frames to the `/service/asr` websocket with optional fallback."""

    def __init__(
        self,
        uri: str = "ws://127.0.0.1:8089/ws",
        *,
        loop: asyncio.AbstractEventLoop | None = None,
        connect_timeout: float = 5.0,
        fallback_factory: Callable[[], AudioAwareBackend] | None = None,
    ) -> None:
        self._uri = uri
        self._loop = loop
        self._connect_timeout = connect_timeout
        self._queue: "queue.Queue[tuple[bytes, int, int] | None]" = queue.Queue()
        self._fallback_factory = fallback_factory
        self._delegate: AudioAwareBackend | None = None
        self._last_text: str = ""

    def submit_audio(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Queue audio to forward to the websocket service or fallback backend."""

        if self._delegate is not None:
            self._delegate.submit_audio(pcm, sample_rate, channels)
            return
        self._queue.put((pcm, sample_rate, channels))

    def close(self) -> None:
        """Signal the active backend to finish processing."""

        if self._delegate is not None:
            self._delegate.close()
        self._queue.put(None)

    def _ensure_fallback(self) -> AudioAwareBackend:
        """Instantiate the fallback backend and replay queued audio chunks."""

        if self._delegate is not None:
            return self._delegate
        if self._fallback_factory is None:
            raise BackendUnavailableError(
                f"ASR service at {self._uri} is unavailable and no fallback is configured"
            )
        fallback = self._fallback_factory()
        self._delegate = fallback
        while not self._queue.empty():
            try:
                item = self._queue.get_nowait()
            except queue.Empty:  # pragma: no cover - race guard
                break
            if item is None:
                continue
            pcm, sample_rate, channels = item
            fallback.submit_audio(pcm, sample_rate, channels)
        return fallback

    async def _transcribe(self, publish: PublishCallback, stop_event: threading.Event) -> None:
        import websockets
        from websockets.exceptions import WebSocketException

        try:
            async with websockets.connect(self._uri, open_timeout=self._connect_timeout) as ws:
                self._last_text = ""
                tasks = [
                    asyncio.create_task(self._pump_audio_chunks(ws, stop_event)),
                    asyncio.create_task(self._receive_service_messages(ws, publish, stop_event)),
                ]
                results = await asyncio.gather(*tasks, return_exceptions=True)
                for result in results:
                    if isinstance(result, Exception):
                        raise result
        except (OSError, asyncio.TimeoutError, WebSocketException) as error:
            raise BackendUnavailableError(
                f"Failed to connect to ASR service at {self._uri}"
            ) from error

    def run(self, publish: PublishCallback, stop_event: threading.Event) -> None:
        """Bridge the websocket transcription service to the publish callback."""

        loop = self._loop or asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        fallback_error: BackendUnavailableError | None = None
        try:
            loop.run_until_complete(self._transcribe(publish, stop_event))
        except BackendUnavailableError as error:
            fallback_error = error
        finally:
            pending = asyncio.all_tasks(loop=loop)
            for task in pending:
                task.cancel()
            if pending:
                loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            loop.stop()
            loop.close()
        if fallback_error is not None:
            fallback = self._ensure_fallback()
            _LOGGER.warning(
                f"ASR service unavailable at {self._uri}; using {fallback.__class__.__name__} fallback",
            )
            fallback.run(publish, stop_event)

    async def _pump_audio_chunks(self, ws, stop_event: threading.Event) -> None:
        from websockets.exceptions import WebSocketException

        while not stop_event.is_set():
            try:
                item = await asyncio.to_thread(self._queue.get, True, 0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            pcm, sample_rate, channels = item
            payload = {
                "type": "chunk",
                "sample_rate": sample_rate,
                "channels": channels,
                "pcm": base64.b64encode(pcm).decode("ascii"),
            }
            try:
                await ws.send(json.dumps(payload))
            except WebSocketException as error:
                raise error

    async def _receive_service_messages(self, ws, publish: PublishCallback, stop_event: threading.Event) -> None:
        from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK

        while not stop_event.is_set():
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            except (ConnectionClosedError, ConnectionClosedOK):
                break
            if isinstance(response, bytes):
                continue
            self._process_service_message(response, publish)

    def _process_service_message(self, message: str, publish: PublishCallback) -> None:
        text = self._extract_text_from_message(message)
        if not text:
            return
        if text == self._last_text:
            return
        self._last_text = text
        publish(text)

    def _extract_text_from_message(self, message: str) -> str:
        trimmed = message.strip()
        if not trimmed:
            return ""
        if trimmed.startswith("{"):
            try:
                payload = json.loads(trimmed)
            except json.JSONDecodeError:
                return trimmed
            event = str(payload.get("event", "")).lower()
            if event == "ready":
                sample_rate = payload.get("sample_rate")
                if sample_rate:
                    _LOGGER.debug("ASR service ready (sample_rate=%s)", sample_rate)
                return ""
            if event == "error":
                message_text = str(payload.get("message", "")).strip()
                if message_text:
                    _LOGGER.error("ASR service error: %s", message_text)
                return ""
            if event in {"partial", "final"}:
                segments = payload.get("segments") or []
                parts: list[str] = []
                for segment in segments:
                    if not isinstance(segment, dict):
                        continue
                    part = str(segment.get("text", "")).strip()
                    if part:
                        parts.append(part)
                segment_text = " ".join(parts).strip()
                if event == "partial":
                    return segment_text
                text_value = str(payload.get("text", "")).strip()
                return text_value or segment_text
            text_value = str(payload.get("text", "")).strip()
            return text_value
        return trimmed
