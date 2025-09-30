"""ASR backend implementations used by the ear module."""
from __future__ import annotations

import asyncio
import base64
import importlib
import json
import math
import time
import uuid
from typing import Callable, List, Mapping, Optional, Sequence, Tuple, Type

from .transcription_types import (
    SegmentData,
    TranscriptionResult,
    WordData,
    coalesce_words,
    looks_like_placeholder_transcript,
    result_is_usable,
    segments_and_words_from_payload,
    words_from_sequence,
)

try:
    import numpy as np
except ImportError:  # pragma: no cover - numpy is required for runtime but optional for tests
    np = None  # type: ignore

try:  # Optional ROS imports for runtime usage.
    from rclpy.node import Node  # noqa: F401
except ImportError:  # pragma: no cover - unit tests stub out ROS.
    Node = object  # type: ignore


__all__ = [
    "FasterWhisperBackend",
    "WhisperBackend",
    "load_backend",
    "RemoteAsrBackend",
    "initialise_remote_backend",
    "ChainedTranscriptionBackend",
    "_load_websocket_dependencies",
]


def _load_websocket_dependencies(
    import_module: Callable[[str], object] = importlib.import_module,
) -> Tuple[Optional[Callable[..., object]], Type[BaseException]]:
    """Resolve websocket client entry points with broad version support."""

    candidates = [
        ("websockets.asyncio.client", "connect"),
        ("websockets.client", "connect"),
        ("websockets", "connect"),
    ]

    connect: Optional[Callable[..., object]] = None
    for module_name, attribute in candidates:
        try:
            module = import_module(module_name)
        except ImportError:
            continue
        try:
            candidate = getattr(module, attribute)
        except AttributeError:
            continue
        if candidate is not None:
            connect = candidate
            break

    if connect is None:
        return None, RuntimeError

    try:
        exceptions_module = import_module("websockets.exceptions")
        connection_closed: Type[BaseException] = getattr(
            exceptions_module, "ConnectionClosed"
        )
    except (ImportError, AttributeError):
        connection_closed = RuntimeError

    return connect, connection_closed


websocket_connect, ConnectionClosed = _load_websocket_dependencies()


def _logprob_to_confidence(logprob: float) -> float:
    """Convert a log-probability into a loose 0..1 confidence score."""

    try:
        probability = math.exp(float(logprob))
    except Exception:
        return 0.0
    return max(0.0, min(1.0, probability))


class FasterWhisperBackend:
    """Wrapper around :mod:`faster_whisper` for online decoding."""

    def __init__(
        self, model_name: str, device: str, compute_type: str, language: Optional[str], beam_size: int
    ) -> None:
        from faster_whisper import WhisperModel

        self._model = WhisperModel(model_name, device=device, compute_type=compute_type)
        self._language = language or None
        self._beam_size = max(1, int(beam_size))

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[TranscriptionResult]:
        if np is None:
            raise RuntimeError("numpy is required for transcription")
        audio = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32)
        if audio.size == 0:
            return None
        audio = audio / 32768.0

        segments, info = self._model.transcribe(
            audio,
            beam_size=self._beam_size,
            language=self._language,
            word_timestamps=True,
        )
        texts = []
        confidences = []
        segments_out: List[SegmentData] = []
        words_out: List[WordData] = []
        for segment in segments:
            text = segment.text.strip()
            if text:
                texts.append(text)
            start = float(getattr(segment, "start", getattr(segment, "t0", 0.0)) or 0.0)
            end = float(getattr(segment, "end", getattr(segment, "t1", start)) or start)
            if end < start:
                end = start
            if text:
                segments_out.append(
                    SegmentData(start=start, end=end, text=text, speaker=None)
                )
            words_out.extend(words_from_sequence(getattr(segment, "words", []), start, end))
            if segment.avg_logprob is not None:
                confidences.append(_logprob_to_confidence(segment.avg_logprob))
            elif segment.no_speech_prob is not None:
                confidences.append(
                    max(0.0, min(1.0, 1.0 - float(segment.no_speech_prob)))
                )
        if not texts:
            return None
        if confidences:
            confidence = float(sum(confidences) / len(confidences))
        else:
            confidence = float(getattr(info, "language_probability", 0.0) or 0.0)
        text_out = " ".join(texts).strip()
        words_coalesced = coalesce_words(text_out, segments_out, words_out)
        return TranscriptionResult(
            text=text_out,
            confidence=confidence,
            segments=segments_out,
            words=words_coalesced,
        )


class WhisperBackend:
    """Fallback backend using OpenAI's reference Whisper implementation."""

    def __init__(self, model_name: str, device: str, language: Optional[str]) -> None:
        import whisper

        self._model = whisper.load_model(model_name, device=device)
        self._language = language or None
        self._use_fp16 = device != "cpu"

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[TranscriptionResult]:
        if np is None:
            raise RuntimeError("numpy is required for transcription")
        audio = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32)
        if audio.size == 0:
            return None
        audio = audio / 32768.0

        result = self._model.transcribe(
            audio,
            language=self._language,
            fp16=self._use_fp16,
            word_timestamps=True,
        )
        text = str(result.get("text", "")).strip()
        if not text:
            return None
        avg_logprob = result.get("avg_logprob")
        if avg_logprob is not None:
            confidence = _logprob_to_confidence(float(avg_logprob))
        else:
            confidence = float(result.get("language_probability") or 0.0)
        segments_raw = result.get("segments")
        segments_out, words_out = segments_and_words_from_payload(segments_raw)
        words_coalesced = coalesce_words(text, segments_out, words_out)
        return TranscriptionResult(
            text=text,
            confidence=confidence,
            segments=segments_out,
            words=words_coalesced,
        )


def load_backend(
    model_name: str,
    device: str,
    compute_type: str,
    language: Optional[str],
    beam_size: int,
    logger=None,
):
    """Attempt to construct an ASR backend."""

    try:
        return FasterWhisperBackend(model_name, device, compute_type, language, beam_size)
    except ImportError:
        if logger:
            logger.info("faster_whisper not available; falling back to whisper")
    except Exception as exc:  # pragma: no cover - defensive logging
        if logger:
            logger.error(f"Failed to load faster_whisper backend: {exc}")
    try:
        return WhisperBackend(model_name, device, language)
    except ImportError:
        if logger:
            logger.warning(
                "Neither faster_whisper nor whisper packages are installed; ASR is disabled. "
                "Install faster-whisper for best performance."
            )
    except Exception as exc:  # pragma: no cover - defensive logging
        if logger:
            logger.error(f"Failed to load whisper backend: {exc}")
    return None


class RemoteAsrBackend:
    """Client for the websocket-based ASR microservice."""

    def __init__(
        self,
        *,
        uri: str,
        language: Optional[str],
        connect_timeout: float,
        response_timeout: float,
        logger=None,
        connector=None,
        monotonic: Callable[[], float] | None = None,
    ) -> None:
        if not uri:
            raise ValueError("remote ASR URI must be non-empty")
        if connector is None and websocket_connect is None:
            raise RuntimeError("websockets dependency is not available")
        self._uri = uri
        self._language = language
        self._connect_timeout = max(0.1, float(connect_timeout))
        self._response_timeout = max(0.1, float(response_timeout))
        self._logger = logger
        self._connector = connector or websocket_connect
        self._monotonic = monotonic or time.monotonic

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[TranscriptionResult]:
        if not pcm_bytes:
            return None
        payload_b64 = base64.b64encode(pcm_bytes).decode("ascii")
        stream_id = f"ear-{uuid.uuid4().hex}"
        chunk_id = f"chunk-{uuid.uuid4().hex}"

        try:
            return asyncio.run(
                self._transcribe_async(
                    stream_id=stream_id,
                    chunk_id=chunk_id,
                    payload_b64=payload_b64,
                    sample_rate=sample_rate,
                )
            )
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError(f"remote ASR error: {exc}") from exc

    async def _transcribe_async(
        self,
        *,
        stream_id: str,
        chunk_id: str,
        payload_b64: str,
        sample_rate: int,
    ) -> Optional[TranscriptionResult]:
        assert self._connector is not None
        async with self._connector(
            self._uri,
            open_timeout=self._connect_timeout,
            close_timeout=self._connect_timeout,
            max_size=5_000_000,
        ) as websocket:
            init_payload = {
                "type": "init",
                "stream_id": stream_id,
                "content_type": f"audio/pcm; rate={sample_rate}",
                "sample_rate": sample_rate,
                "extras": {"chunk_id": chunk_id},
            }
            if self._language:
                init_payload["lang"] = self._language
            await websocket.send(json.dumps(init_payload))

            audio_payload = {
                "type": "audio",
                "stream_id": stream_id,
                "seq": 0,
                "payload_b64": payload_b64,
            }
            await websocket.send(json.dumps(audio_payload))

            commit_payload = {
                "type": "commit",
                "stream_id": stream_id,
                "chunk_id": chunk_id,
            }
            await websocket.send(json.dumps(commit_payload))

            partial_result: Optional[TranscriptionResult] = None
            final_result: Optional[TranscriptionResult] = None

            deadline = self._monotonic() + self._response_timeout
            while True:
                timeout = deadline - self._monotonic()
                if timeout <= 0.0:
                    raise RuntimeError("Timed out waiting for ASR response")
                try:
                    response_raw = await asyncio.wait_for(websocket.recv(), timeout=timeout)
                except asyncio.TimeoutError as exc:
                    raise RuntimeError("Timed out waiting for ASR response") from exc
                except ConnectionClosed as exc:  # pragma: no cover - runtime only
                    raise RuntimeError(f"ASR connection closed: {exc}") from exc

                if not response_raw:
                    continue
                try:
                    payload = json.loads(response_raw)
                except json.JSONDecodeError as exc:
                    raise RuntimeError(f"Invalid ASR response: {exc}") from exc
                response_type = str(payload.get("type", "")).strip().lower()

                # Extend the response window while the server keeps streaming updates.
                deadline = self._monotonic() + self._response_timeout

                if response_type == "stats":
                    continue
                if response_type == "error":
                    message = payload.get("message", "remote ASR error")
                    raise RuntimeError(str(message))

                result = self._result_from_payload(payload)

                if response_type == "partial":
                    avg_logprob = payload.get("avg_logprob")
                    if avg_logprob is not None:
                        confidence = _logprob_to_confidence(float(avg_logprob))
                        result = TranscriptionResult(
                            text=result.text,
                            confidence=confidence,
                            segments=result.segments,
                            words=result.words,
                        )
                    partial_result = result if result_is_usable(result) else partial_result
                    continue

                if response_type == "final":
                    confidence_raw = payload.get("confidence")
                    if confidence_raw is not None:
                        result = TranscriptionResult(
                            text=result.text,
                            confidence=float(confidence_raw or 0.0),
                            segments=result.segments,
                            words=result.words,
                        )
                    if result_is_usable(result):
                        final_result = result
                        break
                    continue

                if response_type == "refine":
                    if result_is_usable(result):
                        return result
                    continue

                if self._logger:
                    self._logger.warning(
                        f"Unknown ASR response type {payload.get('type')}"
                    )

            if result_is_usable(final_result):
                return final_result
            if result_is_usable(partial_result):
                return partial_result
        return None

    def _result_from_payload(self, payload: Mapping[str, object]) -> TranscriptionResult:
        text = str(payload.get("text", "") or "").strip()
        segments_payload = payload.get("segments")
        words_payload = payload.get("words")
        segments_out, words_out = segments_and_words_from_payload(segments_payload)
        if words_payload:
            words_out.extend(words_from_sequence(words_payload, 0.0, 0.0))
        words_coalesced = coalesce_words(text, segments_out, words_out)
        confidence_raw = payload.get("confidence")
        confidence = float(confidence_raw or 0.0) if confidence_raw is not None else 0.0
        return TranscriptionResult(
            text=text,
            confidence=confidence,
            segments=segments_out,
            words=words_coalesced,
        )


class ChainedTranscriptionBackend:
    """Backend that prefers a primary decoder but falls back when unavailable."""

    def __init__(
        self,
        *,
        primary,
        fallback,
        cooldown_seconds: float,
        logger=None,
        monotonic: Callable[[], float] | None = None,
    ) -> None:
        self._primary = primary
        self._fallback = fallback
        self._cooldown = max(0.0, float(cooldown_seconds))
        self._logger = logger
        self._monotonic = monotonic or time.monotonic
        self._next_retry = 0.0

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[TranscriptionResult]:
        now = self._monotonic()
        primary_result: Optional[TranscriptionResult] = None
        if self._primary is not None and now >= self._next_retry:
            try:
                candidate = self._primary.transcribe(pcm_bytes, sample_rate)
                if result_is_usable(candidate):
                    return candidate
                primary_result = candidate
                self._next_retry = now + self._cooldown
                if self._logger:
                    reason = "placeholder transcript" if candidate and looks_like_placeholder_transcript(candidate.text) else "blank transcript"
                    if candidate is None:
                        reason = "empty transcript"
                    self._logger.warning(
                        f"Remote ASR returned {reason}; falling back to onboard decoder."
                    )
            except Exception as exc:
                if self._logger:
                    self._logger.warning(
                        f"Remote ASR failed: {exc}. Falling back to onboard decoder."
                    )
                self._next_retry = now + self._cooldown

        if self._fallback is None:
            return primary_result

        try:
            return self._fallback.transcribe(pcm_bytes, sample_rate)
        except Exception as exc:
            if self._logger:
                self._logger.error(f"Fallback ASR failed: {exc}")
            return None


def initialise_remote_backend(
    *,
    uri: str,
    language: Optional[str],
    connect_timeout: float,
    response_timeout: float,
    logger=None,
) -> Optional[RemoteAsrBackend]:
    """Construct the remote backend when dependencies are available."""

    if not uri:
        return None
    if websocket_connect is None:
        if logger:
            logger.warning("websockets package is not installed; remote ASR disabled")
        return None
    try:
        return RemoteAsrBackend(
            uri=uri,
            language=language,
            connect_timeout=connect_timeout,
            response_timeout=response_timeout,
            logger=logger,
        )
    except Exception as exc:
        if logger:
            logger.warning(f"Failed to initialise remote ASR backend: {exc}")
        return None
