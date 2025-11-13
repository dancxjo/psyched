"""Transcription backend implementations for the ear module."""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import queue
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Literal, Protocol, Sequence, TextIO, runtime_checkable

_LOGGER = logging.getLogger(__name__)

PublishCallback = Callable[["TranscriptionEvent"], None]


@dataclass(slots=True)
class TranscriptWord:
    """Word-level timing metadata associated with a transcript segment."""

    text: str
    start_ms: int | None = None
    end_ms: int | None = None

    def to_dict(self) -> dict[str, object]:
        """Serialize the word into a JSON-friendly mapping."""

        payload: dict[str, object] = {"text": self.text}
        if self.start_ms is not None:
            payload["start_ms"] = self.start_ms
        if self.end_ms is not None:
            payload["end_ms"] = self.end_ms
        return payload


@dataclass(slots=True)
class TranscriptSegment:
    """Sentence or phrase recognised during transcription."""

    text: str
    start_ms: int | None = None
    end_ms: int | None = None
    words: list[TranscriptWord] = field(default_factory=list)

    def to_dict(self) -> dict[str, object]:
        """Serialize the segment into a JSON-friendly mapping."""

        payload: dict[str, object] = {"text": self.text}
        if self.start_ms is not None:
            payload["start_ms"] = self.start_ms
        if self.end_ms is not None:
            payload["end_ms"] = self.end_ms
        if self.words:
            payload["words"] = [word.to_dict() for word in self.words]
        return payload


@dataclass(slots=True)
class TranscriptionEvent:
    """Event emitted by a transcription backend."""

    kind: Literal["partial", "final"]
    text: str
    segments: list[TranscriptSegment] = field(default_factory=list)
    start_ms: int | None = None
    end_ms: int | None = None
    audio_base64: str | None = None

    @classmethod
    def partial(
        cls,
        text: str,
        *,
        segments: Sequence[TranscriptSegment] | None = None,
    ) -> "TranscriptionEvent":
        """Construct a partial transcription event."""

        return cls(
            kind="partial",
            text=text,
            segments=list(segments) if segments is not None else [],
        )

    @classmethod
    def final(
        cls,
        text: str,
        *,
        segments: Sequence[TranscriptSegment] | None = None,
        start_ms: int | None = None,
        end_ms: int | None = None,
        audio_base64: str | None = None,
    ) -> "TranscriptionEvent":
        """Construct a final transcription event."""

        return cls(
            kind="final",
            text=text,
            segments=list(segments) if segments is not None else [],
            start_ms=start_ms,
            end_ms=end_ms,
            audio_base64=audio_base64,
        )

    @property
    def is_partial(self) -> bool:
        """Return ``True`` when the event represents an interim transcript."""

        return self.kind == "partial"

    def to_dict(self) -> dict[str, object]:
        """Serialize the event into a JSON-friendly mapping."""

        payload: dict[str, object] = {
            "event": self.kind,
            "text": self.text,
        }
        if self.segments:
            payload["segments"] = [segment.to_dict() for segment in self.segments]
        if self.start_ms is not None:
            payload["start_ms"] = self.start_ms
        if self.end_ms is not None:
            payload["end_ms"] = self.end_ms
        if self.audio_base64:
            payload["audio_base64"] = self.audio_base64
        return payload


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
        Optional stream used to render a colored prompt. Defaults to
        :data:`sys.stdout`.
    prompt:
        Prompt rendered before waiting for user input. Defaults to ``"🦻  "``.
    color:
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
    color: str = "\x1b[36m"
    _reset: str = field(default="\x1b[0m", init=False, repr=False)

    def run(self, publish: PublishCallback, stop_event: threading.Event) -> None:
        """Continuously read lines and publish non-empty entries."""

        while not stop_event.is_set():
            if self.output_stream is not None:
                self.output_stream.write(f"{self.color}{self.prompt}{self._reset}")
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
            publish(TranscriptionEvent.final(text))


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
                    "faster-whisper backend currently expects mono audio; received %s channels",
                    channels,
                )
            audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
            segment_results, _ = self._model.transcribe(
                audio,
                language=self._language,
                beam_size=self._beam_size,
                temperature=0.0,
                vad_filter=True,
                vad_parameters={"min_speech_duration_ms": 250},
                word_timestamps=True,
            )
            segments: list[TranscriptSegment] = []
            for segment in segment_results:
                text = getattr(segment, "text", "").strip()
                if not text:
                    continue
                start = getattr(segment, "start", None)
                end = getattr(segment, "end", None)
                words: list[TranscriptWord] = []
                for word in getattr(segment, "words", []) or []:
                    word_text = getattr(word, "word", "").strip()
                    if not word_text:
                        continue
                    word_start = getattr(word, "start", None)
                    word_end = getattr(word, "end", None)
                    words.append(
                        TranscriptWord(
                            text=word_text,
                            start_ms=int(word_start * 1000) if isinstance(word_start, (int, float)) else None,
                            end_ms=int(word_end * 1000) if isinstance(word_end, (int, float)) else None,
                        )
                    )
                segments.append(
                    TranscriptSegment(
                        text=text,
                        start_ms=int(start * 1000) if isinstance(start, (int, float)) else None,
                        end_ms=int(end * 1000) if isinstance(end, (int, float)) else None,
                        words=words,
                    )
                )
            transcript = " ".join(segment.text for segment in segments if segment.text).strip()
            if transcript:
                start_ms = segments[0].start_ms if segments else None
                end_ms = segments[-1].end_ms if segments else None
                publish(
                    TranscriptionEvent.final(
                        transcript,
                        segments=segments,
                        start_ms=start_ms,
                        end_ms=end_ms,
                    )
                )

        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break


class ServiceASREarBackend(AudioAwareBackend):
    """Backend that streams audio frames to the `/service/asr` websocket with optional fallback.

    Parameters
    ----------
    uri:
        Websocket endpoint providing transcription results.
    loop:
        Optional event loop reused for asynchronous tasks.
    connect_timeout:
        Maximum number of seconds to wait while establishing the websocket connection.
    fallback_factory:
        Callable that returns an :class:`AudioAwareBackend` instance to process audio when the
        websocket is unavailable.
    debug_hook:
        Optional callback invoked with structured debug payloads for observability.
    partial_event_timeout:
        Number of seconds a partial transcript may remain unchanged before it is promoted to a
        synthetic final event. Set to ``0`` to disable the automatic promotion.
    """

    def __init__(
        self,
        uri: str = "ws://127.0.0.1:5003/asr",
        *,
        loop: asyncio.AbstractEventLoop | None = None,
        connect_timeout: float = 5.0,
        fallback_factory: Callable[[], AudioAwareBackend] | None = None,
        debug_hook: Callable[[dict[str, object]], None] | None = None,
        partial_event_timeout: float = 2.0,
    ) -> None:
        self._uri = uri
        self._loop = loop
        self._connect_timeout = connect_timeout
        self._queue: "queue.Queue[tuple[bytes, int, int] | None]" = queue.Queue()
        self._fallback_factory = fallback_factory
        self._delegate: AudioAwareBackend | None = None
        self._last_partial_signature: str | None = None
        self._last_final_signature: str | None = None
        self._debug_hook = debug_hook
        self._debug_sequence = 0
        self._silence_profile: tuple[int, int, int] | None = None  # (sample_rate, channels, samples_per_chunk)
        self._last_chunk_time = time.monotonic()
        self._state_lock = threading.Lock()
        self._partial_timeout = max(partial_event_timeout, 0.0)
        self._last_partial_event: TranscriptionEvent | None = None
        self._last_partial_time: float | None = None

    def submit_audio(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Queue audio to forward to the websocket service or fallback backend."""

        if self._delegate is not None:
            self._delegate.submit_audio(pcm, sample_rate, channels)
            return
        self._queue.put((pcm, sample_rate, channels))
        self._record_silence_profile(len(pcm), sample_rate, channels)

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

        self._emit_debug(
            "connecting",
            direction="status",
            uri=self._uri,
        )
        try:
            async with websockets.connect(self._uri, open_timeout=self._connect_timeout) as ws:
                self._emit_debug(
                    "connected",
                    direction="status",
                    uri=self._uri,
                )
                self._last_partial_signature = None
                self._last_final_signature = None
                tasks = [
                    asyncio.create_task(self._pump_audio_chunks(ws, stop_event)),
                    asyncio.create_task(self._receive_service_messages(ws, publish, stop_event)),
                ]
                results = await asyncio.gather(*tasks, return_exceptions=True)
                for result in results:
                    if isinstance(result, Exception):
                        raise result
                self._emit_debug(
                    "disconnected",
                    direction="status",
                    uri=self._uri,
                )
        except (OSError, asyncio.TimeoutError, WebSocketException) as error:
            self._emit_debug(
                "error",
                direction="status",
                uri=self._uri,
                message=str(error),
            )
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
                item = await asyncio.to_thread(self._get_next_audio_chunk, 0.1)
            except queue.Empty:
                continue
            if item is None:
                self._emit_debug(
                    "queue_drained",
                    direction="status",
                )
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
                self._emit_debug(
                    "chunk_sent",
                    direction="outbound",
                    sample_rate=sample_rate,
                    channels=channels,
                    byte_length=len(pcm),
                    queue_depth=self._queue.qsize(),
                )
            except WebSocketException as error:
                raise error

    def _get_next_audio_chunk(self, timeout: float) -> tuple[bytes, int, int] | None:
        try:
            item = self._queue.get(True, timeout)
        except queue.Empty as error:
            silence = self._build_silence_chunk()
            if silence is None:
                raise error
            return silence
        return item

    async def _receive_service_messages(self, ws, publish: PublishCallback, stop_event: threading.Event) -> None:
        from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK

        while not stop_event.is_set():
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=0.5)
            except asyncio.TimeoutError:
                if self._finalize_stale_partial(publish):
                    continue
                continue
            except (ConnectionClosedError, ConnectionClosedOK):
                break
            if isinstance(response, bytes):
                continue
            self._emit_debug(
                "message_received",
                direction="inbound",
                payload=response,
            )
            self._process_service_message(response, publish)

    def _process_service_message(self, message: str, publish: PublishCallback) -> None:
        event = self._parse_service_message(message)
        if event is None:
            self._finalize_stale_partial(publish)
            return
        signature = self._event_signature(event)
        if event.is_partial:
            if signature == self._last_partial_signature:
                if self._finalize_stale_partial(publish):
                    return
                return
            self._last_partial_signature = signature
            self._last_partial_event = event
            self._last_partial_time = time.monotonic()
        else:
            if signature == self._last_final_signature:
                self._clear_partial_state()
                return
            self._last_final_signature = signature
            self._clear_partial_state()
        publish(event)

    def _clear_partial_state(self) -> None:
        """Reset cached partial transcript tracking state."""

        self._last_partial_signature = None
        self._last_partial_event = None
        self._last_partial_time = None

    def _finalize_stale_partial(self, publish: PublishCallback) -> bool:
        """Emit the cached partial transcript as a final event when it stagnates."""

        event = self._last_partial_event
        if event is None:
            return False
        if self._partial_timeout <= 0:
            return False
        last_update = self._last_partial_time
        if last_update is None:
            return False
        elapsed = time.monotonic() - last_update
        if elapsed < self._partial_timeout:
            return False
        final_event = TranscriptionEvent.final(event.text, segments=event.segments)
        final_signature = self._event_signature(final_event)
        if final_signature == self._last_final_signature:
            self._clear_partial_state()
            return False
        self._last_final_signature = final_signature
        self._clear_partial_state()
        publish(final_event)
        return True

    def _emit_debug(self, kind: str, **fields) -> None:
        if not self._debug_hook:
            return
        self._debug_sequence += 1
        payload = {
            "kind": kind,
            "sequence": self._debug_sequence,
            "timestamp": time.time(),
            **fields,
        }
        try:
            self._debug_hook(payload)
        except Exception:  # pragma: no cover - debug hooks should not break transcription
            _LOGGER.debug("ASR service debug hook failed", exc_info=True)

    def _parse_service_message(self, message: str) -> TranscriptionEvent | None:
        trimmed = message.strip()
        if not trimmed:
            return None
        if trimmed.startswith("{"):
            try:
                payload = json.loads(trimmed)
            except json.JSONDecodeError:
                return TranscriptionEvent.final(trimmed)

            event_name = str(payload.get("event", "")).strip().lower()
            if event_name == "ready":
                sample_rate = payload.get("sample_rate")
                if sample_rate:
                    _LOGGER.debug("ASR service ready (sample_rate=%s)", sample_rate)
                return None
            if event_name == "error":
                message_text = str(payload.get("message", "")).strip()
                if message_text:
                    _LOGGER.error("ASR service error: %s", message_text)
                return None

            segments = self._normalise_segments(payload.get("segments"))
            text_value = payload.get("text")
            text = text_value.strip() if isinstance(text_value, str) else ""
            if not text:
                text = self._segments_to_text(segments)

            if event_name == "partial":
                if not text and not segments:
                    return None
                return TranscriptionEvent.partial(text, segments=segments)

            if event_name == "final":
                if not text and not segments:
                    return None
                start_ms = self._coerce_int(payload.get("start_ms"))
                end_ms = self._coerce_int(payload.get("end_ms"))
                audio_base64 = payload.get("audio_base64")
                encoded = audio_base64.strip() if isinstance(audio_base64, str) else None
                return TranscriptionEvent.final(
                    text,
                    segments=segments,
                    start_ms=start_ms,
                    end_ms=end_ms,
                    audio_base64=encoded or None,
                )

            if text:
                return TranscriptionEvent.final(text, segments=segments)
            return None

        return TranscriptionEvent.final(trimmed)

    def _normalise_segments(self, raw_segments: object) -> list[TranscriptSegment]:
        segments: list[TranscriptSegment] = []
        if not isinstance(raw_segments, list):
            return segments
        for entry in raw_segments:
            if not isinstance(entry, dict):
                continue
            text = str(entry.get("text", "")).strip()
            if not text:
                continue
            words: list[TranscriptWord] = []
            raw_words = entry.get("words")
            if isinstance(raw_words, list):
                for word in raw_words:
                    if not isinstance(word, dict):
                        continue
                    word_text = str(word.get("text", "")).strip()
                    if not word_text:
                        continue
                    words.append(
                        TranscriptWord(
                            text=word_text,
                            start_ms=self._coerce_int(word.get("start_ms")),
                            end_ms=self._coerce_int(word.get("end_ms")),
                        )
                    )
            segments.append(
                TranscriptSegment(
                    text=text,
                    start_ms=self._coerce_int(entry.get("start_ms")),
                    end_ms=self._coerce_int(entry.get("end_ms")),
                    words=words,
                )
            )
        return segments

    @staticmethod
    def _segments_to_text(segments: Sequence[TranscriptSegment]) -> str:
        return " ".join(segment.text for segment in segments if segment.text).strip()

    @staticmethod
    def _coerce_int(value: object) -> int | None:
        if isinstance(value, (int, float)):
            return int(value)
        if isinstance(value, str):
            candidate = value.strip()
            if not candidate:
                return None
            try:
                return int(float(candidate))
            except ValueError:
                return None
        return None

    def _event_signature(self, event: TranscriptionEvent) -> str:
        payload = {
            "kind": event.kind,
            "text": event.text,
            "start_ms": event.start_ms,
            "end_ms": event.end_ms,
            "segments": [segment.to_dict() for segment in event.segments],
        }
        return json.dumps(payload, sort_keys=True)

    def _record_silence_profile(self, byte_length: int, sample_rate: int, channels: int) -> None:
        """Remember the latest chunk metadata so silence padding matches the live stream cadence."""

        bytes_per_sample = 2 * max(channels, 1)
        if bytes_per_sample <= 0:
            return
        samples_per_chunk = byte_length // bytes_per_sample
        if samples_per_chunk <= 0:
            return
        with self._state_lock:
            self._silence_profile = (sample_rate, channels, samples_per_chunk)
            self._last_chunk_time = time.monotonic()

    def _build_silence_chunk(self) -> tuple[bytes, int, int] | None:
        """Construct a synthetic silence chunk when the recorder is idle."""

        with self._state_lock:
            profile = self._silence_profile
            if profile is None:
                return None
            sample_rate, channels, samples_per_chunk = profile
            if samples_per_chunk <= 0:
                return None
            rate = max(sample_rate, 1)
            chunk_duration = samples_per_chunk / float(rate)
            now = time.monotonic()
            if (now - self._last_chunk_time) < chunk_duration:
                return None
            bytes_per_sample = 2 * max(channels, 1)
            byte_length = samples_per_chunk * bytes_per_sample
            self._last_chunk_time = now
        return (bytes(byte_length), sample_rate, channels)
