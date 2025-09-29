"""ROS 2 bridge that converts voiced audio segments into transcripts."""
from __future__ import annotations

import asyncio
import importlib
import base64
import json
import math
import os
import queue
import re
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Callable, List, Mapping, Optional, Sequence, Tuple, Type

from .audio_utils import coerce_pcm_bytes

try:
    import numpy as np
except ImportError:  # pragma: no cover - numpy is required for runtime but optional for tests
    np = None  # type: ignore

try:  # Optional ROS imports for runtime usage.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import ByteMultiArray

    from psyched_msgs.msg import Transcript, TranscriptSegment as MsgTranscriptSegment, TranscriptWord as MsgTranscriptWord
except ImportError:  # pragma: no cover - unit tests stub out ROS.
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    SingleThreadedExecutor = object  # type: ignore
    ByteMultiArray = object  # type: ignore
    Transcript = object  # type: ignore
    MsgTranscriptSegment = object  # type: ignore
    MsgTranscriptWord = object  # type: ignore


@dataclass(frozen=True)
class SegmentData:
    """Lightweight representation of a transcript segment."""

    start: float
    end: float
    text: str
    speaker: Optional[str] = None


@dataclass(frozen=True)
class WordData:
    """Word-level timing metadata."""

    start: float
    end: float
    text: str


@dataclass(frozen=True)
class TranscriptionResult:
    """Bundle of recognised text and its alignment metadata."""

    text: str
    confidence: float
    segments: List[SegmentData]
    words: List[WordData]


_PLACEHOLDER_TRANSCRIPT_RE = re.compile(r"^samples=\d+(?:\.\d+)?\s+sum=-?\d+(?:\.\d+)?$")


def _looks_like_placeholder_transcript(text: str) -> bool:
    """Return ``True`` when the recognised text matches known placeholder patterns."""

    candidate = text.strip().lower()
    if not candidate:
        return False
    return _PLACEHOLDER_TRANSCRIPT_RE.match(candidate) is not None


def _result_is_usable(result: Optional[TranscriptionResult]) -> bool:
    """Determine whether a transcription result contains meaningful text."""

    if result is None:
        return False
    text = str(getattr(result, 'text', '') or '').strip()
    if not text:
        return False
    if _looks_like_placeholder_transcript(text):
        return False
    return True


def _coerce_float(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _word_from_mapping(word: Mapping[str, object], fallback_start: float, fallback_end: float) -> Optional[WordData]:
    text_raw = word.get('text', word.get('word', ''))
    text = str(text_raw or '').strip()
    if not text:
        return None
    start = _coerce_float(word.get('start', word.get('t0')), fallback_start)
    end = _coerce_float(word.get('end', word.get('t1')), start)
    if end < start:
        end = start
    return WordData(start=start, end=end, text=text)


def _words_from_sequence(words: object, fallback_start: float, fallback_end: float) -> List[WordData]:
    if not isinstance(words, Sequence) or isinstance(words, (str, bytes, bytearray)):
        return []
    result: List[WordData] = []
    last_end = fallback_start
    for word in words:
        if isinstance(word, Mapping):
            candidate = _word_from_mapping(word, last_end, fallback_end)
        else:
            text = str(getattr(word, 'word', getattr(word, 'text', '')) or '').strip()
            if not text:
                continue
            start = _coerce_float(getattr(word, 'start', getattr(word, 't0', last_end)), last_end)
            end = _coerce_float(getattr(word, 'end', getattr(word, 't1', start)), start)
            if end < start:
                end = start
            candidate = WordData(start=start, end=end, text=text)
        if candidate is None:
            continue
        result.append(candidate)
        last_end = max(last_end, candidate.end)
    return result


def _segments_and_words_from_payload(
    raw_segments: object,
    *,
    default_speaker: Optional[str] = None,
) -> Tuple[List[SegmentData], List[WordData]]:
    if not isinstance(raw_segments, Sequence) or isinstance(raw_segments, (str, bytes, bytearray)):
        return [], []

    segments: List[SegmentData] = []
    words: List[WordData] = []
    for entry in raw_segments:
        if isinstance(entry, Mapping):
            text_raw = entry.get('text', '')
            speaker_raw = entry.get('speaker', default_speaker)
            start = _coerce_float(entry.get('start', entry.get('t0')), 0.0)
            end = _coerce_float(entry.get('end', entry.get('t1')), start)
            if end < start:
                end = start
            text = str(text_raw or '').strip()
            speaker = str(speaker_raw) if speaker_raw else None
            segments.append(SegmentData(start=start, end=end, text=text, speaker=speaker))
            words.extend(_words_from_sequence(entry.get('words'), start, end))
        else:
            text = str(getattr(entry, 'text', '') or '').strip()
            start = _coerce_float(getattr(entry, 'start', getattr(entry, 't0', 0.0)), 0.0)
            end = _coerce_float(getattr(entry, 'end', getattr(entry, 't1', start)), start)
            if end < start:
                end = start
            speaker_attr = getattr(entry, 'speaker', default_speaker)
            speaker = str(speaker_attr) if speaker_attr else None
            segments.append(SegmentData(start=start, end=end, text=text, speaker=speaker))
            words.extend(_words_from_sequence(getattr(entry, 'words', []), start, end))
    return segments, words


def _coalesce_words(text: str, segments: Sequence[SegmentData], words: Sequence[WordData]) -> List[WordData]:
    collected = [WordData(start=w.start, end=w.end, text=w.text) for w in words if w.text]
    if collected:
        return collected
    return _approximate_words_from_segments(text, segments)


def _approximate_words_from_segments(text: str, segments: Sequence[SegmentData]) -> List[WordData]:
    words: List[WordData] = []
    for segment in segments:
        tokens = [token for token in segment.text.split() if token]
        if not tokens:
            continue
        duration = max(0.0, segment.end - segment.start)
        if duration <= 0.0:
            cursor = segment.start
            for token in tokens:
                words.append(WordData(start=cursor, end=cursor, text=token))
            continue
        step = duration / max(1, len(tokens))
        cursor = segment.start
        for index, token in enumerate(tokens):
            start = cursor
            if index == len(tokens) - 1:
                end = segment.end
            else:
                end = min(segment.end, cursor + step)
            words.append(WordData(start=start, end=end, text=token))
            cursor = end
    if words:
        return words
    tokens = [token for token in text.split() if token]
    cursor = 0.0
    for token in tokens:
        start = cursor
        end = cursor + 0.4
        words.append(WordData(start=start, end=end, text=token))
        cursor = end
    return words

def _load_websocket_dependencies(
    import_module: Callable[[str], object] = importlib.import_module,
) -> Tuple[Optional[Callable[..., object]], Type[BaseException]]:
    """Resolve the websocket client entry points with broad version support.

    Parameters
    ----------
    import_module:
        Import callable, injectable to simplify unit testing.

    Returns
    -------
    tuple
        Pair containing the websocket ``connect`` coroutine (or ``None`` if the
        dependency cannot be imported) and the ``ConnectionClosed`` exception
        class to catch during reconnect loops.

    Examples
    --------
    >>> connect, closed = _load_websocket_dependencies()
    >>> connect is None or callable(connect)
    True
    """

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

    def __init__(self, model_name: str, device: str, compute_type: str, language: Optional[str], beam_size: int) -> None:
        from faster_whisper import WhisperModel

        self._model = WhisperModel(model_name, device=device, compute_type=compute_type)
        self._language = language or None
        self._beam_size = max(1, int(beam_size))

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[TranscriptionResult]:
        if np is None:
            raise RuntimeError('numpy is required for transcription')
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
            start = _coerce_float(getattr(segment, 'start', getattr(segment, 't0', 0.0)), 0.0)
            end = _coerce_float(getattr(segment, 'end', getattr(segment, 't1', start)), start)
            if end < start:
                end = start
            if text:
                segments_out.append(SegmentData(start=start, end=end, text=text, speaker=None))
            words_out.extend(_words_from_sequence(getattr(segment, 'words', []), start, end))
            if segment.avg_logprob is not None:
                confidences.append(_logprob_to_confidence(segment.avg_logprob))
            elif segment.no_speech_prob is not None:
                confidences.append(max(0.0, min(1.0, 1.0 - float(segment.no_speech_prob))))
        if not texts:
            return None
        if confidences:
            confidence = float(sum(confidences) / len(confidences))
        else:
            confidence = float(info.language_probability or 0.0)
        text_out = " ".join(texts).strip()
        words_coalesced = _coalesce_words(text_out, segments_out, words_out)
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
        self._use_fp16 = device != 'cpu'

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[TranscriptionResult]:
        if np is None:
            raise RuntimeError('numpy is required for transcription')
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
        text = str(result.get('text', '')).strip()
        if not text:
            return None
        avg_logprob = result.get('avg_logprob')
        if avg_logprob is not None:
            confidence = _logprob_to_confidence(float(avg_logprob))
        else:
            confidence = float(result.get('language_probability') or 0.0)
        segments_raw = result.get('segments')
        segments_out, words_out = _segments_and_words_from_payload(segments_raw)
        words_coalesced = _coalesce_words(text, segments_out, words_out)
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
            logger.info('faster_whisper not available; falling back to whisper')
    except Exception as exc:  # pragma: no cover - defensive logging
        if logger:
            logger.error(f'Failed to load faster_whisper backend: {exc}')
    try:
        return WhisperBackend(model_name, device, language)
    except ImportError:
        if logger:
            logger.warning(
                'Neither faster_whisper nor whisper packages are installed; ASR is disabled. '
                'Install faster-whisper for best performance.'
            )
    except Exception as exc:  # pragma: no cover - defensive logging
        if logger:
            logger.error(f'Failed to load whisper backend: {exc}')
    return None


@dataclass
class TranscriptionWorker:
    """Synchronous helper that wraps a backend ASR engine."""

    backend: Optional[object]
    sample_rate: int
    speaker: str
    on_result: Callable[[TranscriptionResult], None]
    logger: Optional[object] = None

    def handle_segment(self, pcm_bytes: bytes) -> None:
        if not pcm_bytes:
            return
        if self.backend is None:
            if self.logger:
                self.logger.warning('No ASR backend configured; dropping audio segment')
            return
        try:
            result = self.backend.transcribe(pcm_bytes, self.sample_rate)
        except Exception as exc:
            if self.logger:
                self.logger.error(f'ASR backend raised an error: {exc}')
            return
        normalised = self._normalise_result(result)
        if normalised is None:
            return
        self.on_result(normalised)

    def _normalise_result(self, result: object) -> Optional[TranscriptionResult]:
        if result is None:
            return None
        if isinstance(result, TranscriptionResult):
            if not result.text.strip():
                return None
            return result
        if isinstance(result, tuple) and len(result) >= 2:
            text = str(result[0] or '').strip()
            if not text:
                return None
            try:
                confidence = float(result[1])
            except Exception:
                confidence = 0.0
            segment = SegmentData(start=0.0, end=0.0, text=text, speaker=str(self.speaker))
            words = _approximate_words_from_segments(text, [segment])
            return TranscriptionResult(text=text, confidence=confidence, segments=[segment], words=words)
        if self.logger:
            self.logger.warning(f'Unexpected transcription result type: {type(result)!r}')
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
            raise ValueError('remote ASR URI must be non-empty')
        if connector is None and websocket_connect is None:
            raise RuntimeError('websockets dependency is not available')
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
        payload_b64 = base64.b64encode(pcm_bytes).decode('ascii')
        stream_id = f'ear-{uuid.uuid4().hex}'
        chunk_id = f'chunk-{uuid.uuid4().hex}'

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
            raise RuntimeError(f'remote ASR error: {exc}') from exc

    async def _transcribe_async(
        self,
        *,
        stream_id: str,
        chunk_id: str,
        payload_b64: str,
        sample_rate: int,
    ) -> Optional[TranscriptionResult]:
        connector = self._connector
        if connector is None:
            raise RuntimeError('websocket connector unavailable')

        init_payload = {
            'type': 'init',
            'stream_id': stream_id,
            'lang': self._language,
            'content_type': f'audio/pcm; rate={sample_rate}',
            'sample_rate': sample_rate,
            'extras': {},
        }
        audio_payload = {
            'type': 'audio',
            'stream_id': stream_id,
            'seq': 1,
            'payload_b64': payload_b64,
        }
        commit_payload = {
            'type': 'commit',
            'stream_id': stream_id,
            'chunk_id': chunk_id,
        }

        try:
            async with connector(
                self._uri,
                open_timeout=self._connect_timeout,
                close_timeout=self._connect_timeout,
                ping_interval=None,
                ping_timeout=self._response_timeout,
                max_size=None,
            ) as websocket:
                await websocket.send(json.dumps(init_payload))
                await websocket.send(json.dumps(audio_payload))
                await websocket.send(json.dumps(commit_payload))

                deadline = self._monotonic() + self._response_timeout
                text: Optional[str] = None
                confidence: Optional[float] = None
                segments_payload: object = None
                words_payload: object = None

                while True:
                    remaining = deadline - self._monotonic()
                    if remaining <= 0:
                        break
                    try:
                        message = await asyncio.wait_for(websocket.recv(), timeout=remaining)
                    except asyncio.TimeoutError:
                        break
                    except ConnectionClosed as exc:  # pragma: no cover - depends on runtime failures.
                        raise RuntimeError(f'connection closed: {exc}') from exc

                    if isinstance(message, bytes):
                        continue
                    try:
                        payload = json.loads(message)
                    except json.JSONDecodeError:
                        continue
                    msg_type = str(payload.get('type', '')).lower()
                    if msg_type == 'error':
                        raise RuntimeError(str(payload.get('message', 'remote asr error')))
                    if msg_type == 'final':
                        text = str(payload.get('text', '')).strip()
                        conf = payload.get('confidence')
                        confidence = float(conf) if conf is not None else confidence
                        segments_payload = payload.get('segments')
                        words_payload = payload.get('words')
                        break
                    if msg_type == 'refine':
                        text = str(payload.get('text', '')).strip()
                        segments_payload = payload.get('segments')
                        words_payload = payload.get('words')
                        break
                    if msg_type == 'partial' and not text:
                        text = str(payload.get('text', '')).strip()
                        conf = payload.get('avg_logprob')
                        if conf is not None:
                            try:
                                confidence = float(conf)
                            except Exception:
                                confidence = confidence

                if not text:
                    return None
                segments_out, words_out = _segments_and_words_from_payload(segments_payload)
                if words_payload:
                    words_out.extend(_words_from_sequence(words_payload, 0.0, 0.0))
                words_coalesced = _coalesce_words(text, segments_out, words_out)
                return TranscriptionResult(
                    text=text,
                    confidence=float(confidence or 0.0),
                    segments=segments_out,
                    words=words_coalesced,
                )
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError(str(exc)) from exc


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
                if _result_is_usable(candidate):
                    return candidate
                primary_result = candidate
                self._next_retry = now + self._cooldown
                if self._logger:
                    reason = 'blank transcript' if candidate and not str(candidate.text or '').strip() else 'placeholder transcript'
                    if candidate is None:
                        reason = 'empty transcript'
                    self._logger.warning(
                        f'Remote ASR returned {reason}; falling back to onboard decoder.'
                    )
            except Exception as exc:
                if self._logger:
                    self._logger.warning(f'Remote ASR failed: {exc}. Falling back to onboard decoder.')
                self._next_retry = now + self._cooldown

        if self._fallback is None:
            return primary_result

        try:
            return self._fallback.transcribe(pcm_bytes, sample_rate)
        except Exception as exc:
            if self._logger:
                self._logger.error(f'Fallback ASR failed: {exc}')
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
            logger.warning('websockets package is not installed; remote ASR disabled')
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
            logger.warning(f'Failed to initialise remote ASR backend: {exc}')
        return None


class TranscriberNode(Node):  # type: ignore[misc]
    """ROS 2 node that fans audio buffers to the remote ASR tiers."""

    def __init__(self, backend: Optional[object] = None) -> None:  # pragma: no cover - requires ROS
        super().__init__('transcriber')

        self._segment_topic = self.declare_parameter('segment_topic', '/audio/speech_segment').value
        self._segment_accum_topic = self.declare_parameter(
            'segment_accumulating_topic', '/audio/speech_segment_accumulating'
        ).value
        self._speech_accum_topic = self.declare_parameter(
            'speech_accumulating_topic', '/audio/speech_accumulating'
        ).value
        self._transcript_topic = self.declare_parameter('transcript_topic', '/audio/transcription').value
        self._transcript_short_topic = self.declare_parameter(
            'transcript_short_topic', '/audio/transcript/short'
        ).value
        self._transcript_medium_topic = self.declare_parameter(
            'transcript_medium_topic', '/audio/transcript/medium'
        ).value
        self._transcript_long_topic = self.declare_parameter(
            'transcript_long_topic', '/audio/transcript/long'
        ).value
        self._speaker_label = self.declare_parameter('speaker', 'user').value
        self._sample_rate = int(self.declare_parameter('segment_sample_rate', 16000).value)
        self._model_name = self.declare_parameter('model', 'base').value
        self._device = self.declare_parameter('device', 'cpu').value
        self._compute_type = self.declare_parameter('compute_type', 'int8').value
        self._language = self.declare_parameter('language', '').value or None
        self._beam_size = int(self.declare_parameter('beam_size', 5).value)

        default_fast_ws = os.getenv('EAR_ASR_FAST_WS_URL', 'ws://forebrain.local:8082/ws')
        default_medium_ws = os.getenv('EAR_ASR_MEDIUM_WS_URL', 'ws://forebrain.local:8083/ws')
        default_long_ws = os.getenv('EAR_ASR_LONG_WS_URL', 'ws://forebrain.local:8084/ws')
        self._fast_ws_url = self.declare_parameter('fast_remote_ws_url', default_fast_ws).value
        self._medium_ws_url = self.declare_parameter('medium_remote_ws_url', default_medium_ws).value
        self._long_ws_url = self.declare_parameter('long_remote_ws_url', default_long_ws).value
        self._remote_connect_timeout = float(self.declare_parameter('remote_connect_timeout', 0.6).value)
        self._remote_response_timeout = float(self.declare_parameter('remote_response_timeout', 1.5).value)

        self._fast_backend = initialise_remote_backend(
            uri=str(self._fast_ws_url or '').strip(),
            language=self._language,
            connect_timeout=self._remote_connect_timeout,
            response_timeout=self._remote_response_timeout,
            logger=self.get_logger(),
        )
        self._medium_backend = initialise_remote_backend(
            uri=str(self._medium_ws_url or '').strip(),
            language=self._language,
            connect_timeout=self._remote_connect_timeout,
            response_timeout=self._remote_response_timeout,
            logger=self.get_logger(),
        )
        self._long_backend = initialise_remote_backend(
            uri=str(self._long_ws_url or '').strip(),
            language=self._language,
            connect_timeout=self._remote_connect_timeout,
            response_timeout=self._remote_response_timeout,
            logger=self.get_logger(),
        )

        if backend is not None:
            if self._medium_backend is None:
                self._medium_backend = backend
            if self._long_backend is None:
                self._long_backend = backend
            if self._fast_backend is None:
                self._fast_backend = backend

        self.transcript_pub = self.create_publisher(Transcript, self._transcript_topic, 10)
        self.transcript_short_pub = self.create_publisher(Transcript, self._transcript_short_topic, 10)
        self.transcript_medium_pub = self.create_publisher(Transcript, self._transcript_medium_topic, 10)
        self.transcript_long_pub = self.create_publisher(Transcript, self._transcript_long_topic, 10)

        self._stop_evt = threading.Event()
        self._backend_warnings: dict[str, bool] = {'short': False, 'medium': False, 'long': False}

        self._short_queue: 'queue.Queue[Optional[bytes]]' = queue.Queue(maxsize=2)
        self._medium_queue: 'queue.Queue[Optional[bytes]]' = queue.Queue()
        self._long_queue: 'queue.Queue[Optional[bytes]]' = queue.Queue()

        self._threads: list[threading.Thread] = []
        self._threads.append(
            threading.Thread(
                target=self._worker_loop,
                args=('short', self._fast_backend, self._short_queue, self._publish_short),
                daemon=True,
            )
        )
        self._threads.append(
            threading.Thread(
                target=self._worker_loop,
                args=('medium', self._medium_backend, self._medium_queue, self._publish_medium),
                daemon=True,
            )
        )
        self._threads.append(
            threading.Thread(
                target=self._worker_loop,
                args=('long', self._long_backend, self._long_queue, self._publish_long),
                daemon=True,
            )
        )
        for thread in self._threads:
            thread.start()

        self._segment_sub = self.create_subscription(
            ByteMultiArray, self._segment_topic, self._on_segment, 10
        )
        self._segment_accum_sub = self.create_subscription(
            ByteMultiArray, self._segment_accum_topic, self._on_segment_accumulating, 10
        )
        self._speech_accum_sub = self.create_subscription(
            ByteMultiArray, self._speech_accum_topic, self._on_speech_accumulating, 10
        )

        self.get_logger().info(
            f'Transcriber ready: short={bool(self._fast_backend)} medium={bool(self._medium_backend)} long={bool(self._long_backend)}'
        )

    def destroy_node(self) -> None:  # pragma: no cover - requires ROS
        self._stop_evt.set()
        for q in (self._short_queue, self._medium_queue, self._long_queue):
            try:
                q.put_nowait(None)
            except Exception:
                pass
        for thread in self._threads:
            try:
                thread.join(timeout=2.0)
            except Exception:
                pass
        return super().destroy_node()

    def _worker_loop(
        self,
        tier: str,
        backend: Optional[object],
        audio_queue: 'queue.Queue[Optional[bytes]]',
        publisher: Callable[[TranscriptionResult], None],
    ) -> None:
        logger = self.get_logger()
        while not self._stop_evt.is_set():
            try:
                item = audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            if backend is None:
                if not self._backend_warnings.get(tier, False):
                    logger.warning('No ASR backend configured for %s tier; dropping audio', tier)
                    self._backend_warnings[tier] = True
                continue
            try:
                result = backend.transcribe(item, self._sample_rate)  # type: ignore[attr-defined]
            except Exception as exc:  # pragma: no cover - runtime behaviour
                logger.warning('Backend %s raised error: %s', tier, exc)
                continue
            if not _result_is_usable(result):
                continue
            try:
                publisher(result)  # type: ignore[arg-type]
            except Exception as exc:  # pragma: no cover - runtime behaviour
                logger.error('Failed to publish %s transcript: %s', tier, exc)

    def _on_segment(self, msg: ByteMultiArray) -> None:  # pragma: no cover - requires ROS
        pcm = coerce_pcm_bytes(msg.data)
        if not pcm:
            return
        self._medium_queue.put(pcm)

    def _on_segment_accumulating(self, msg: ByteMultiArray) -> None:  # pragma: no cover - requires ROS
        pcm = coerce_pcm_bytes(msg.data)
        if not pcm:
            return
        self._enqueue_latest(self._short_queue, pcm)

    def _on_speech_accumulating(self, msg: ByteMultiArray) -> None:  # pragma: no cover - requires ROS
        pcm = coerce_pcm_bytes(msg.data)
        if not pcm:
            return
        self._long_queue.put(pcm)

    def _enqueue_latest(self, queue_ref: 'queue.Queue[Optional[bytes]]', pcm: bytes) -> None:
        try:
            queue_ref.put_nowait(pcm)
        except queue.Full:
            try:
                queue_ref.get_nowait()
            except queue.Empty:
                pass
            queue_ref.put_nowait(pcm)

    def _publish_short(self, result: TranscriptionResult) -> None:
        msg = self._build_transcript(result, include_timing=False, include_words=False)
        self.transcript_short_pub.publish(msg)

    def _publish_medium(self, result: TranscriptionResult) -> None:
        msg = self._build_transcript(result, include_timing=True, include_words=True)
        self.transcript_medium_pub.publish(msg)
        self.transcript_pub.publish(msg)

    def _publish_long(self, result: TranscriptionResult) -> None:
        msg = self._build_transcript(result, include_timing=True, include_words=True)
        self.transcript_long_pub.publish(msg)

    def _build_transcript(
        self,
        result: TranscriptionResult,
        *,
        include_timing: bool,
        include_words: bool,
    ) -> Transcript:
        msg = Transcript()
        msg.text = result.text
        segment_speaker = next((segment.speaker for segment in result.segments if segment.speaker), None)
        msg.speaker = segment_speaker or str(self._speaker_label)
        msg.confidence = float(result.confidence)

        if include_timing:
            msg.segments = []
            for segment in result.segments:
                seg_msg = MsgTranscriptSegment()
                seg_msg.start = float(segment.start)
                seg_msg.end = float(segment.end)
                seg_msg.text = segment.text
                seg_msg.speaker = segment.speaker or str(self._speaker_label)
                msg.segments.append(seg_msg)
        else:
            msg.segments = []  # type: ignore[assignment]

        if include_words:
            msg.words = []
            for word in result.words:
                word_msg = MsgTranscriptWord()
                word_msg.start = float(word.start)
                word_msg.end = float(word.end)
                word_msg.text = word.text
                msg.words.append(word_msg)
        else:
            msg.words = []  # type: ignore[assignment]
        return msg


def main(args=None):  # pragma: no cover - requires ROS
    rclpy.init(args=args)
    node = TranscriberNode()
    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = [
    'TranscriberNode',
    'TranscriptionWorker',
    'TranscriptionResult',
    'SegmentData',
    'WordData',
    'load_backend',
    'FasterWhisperBackend',
    'WhisperBackend',
    'RemoteAsrBackend',
    'ChainedTranscriptionBackend',
]
