"""ROS 2 bridge that converts voiced audio segments into transcripts."""
from __future__ import annotations

import asyncio
import importlib
import base64
import json
import math
import os
import queue
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Callable, Optional, Tuple, Type

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

    from psyched_msgs.msg import Transcript
except ImportError:  # pragma: no cover - unit tests stub out ROS.
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    SingleThreadedExecutor = object  # type: ignore
    ByteMultiArray = object  # type: ignore
    Transcript = object  # type: ignore

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

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[Tuple[str, float]]:
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
        )
        texts = []
        confidences = []
        for segment in segments:
            text = segment.text.strip()
            if text:
                texts.append(text)
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
        return " ".join(texts).strip(), confidence


class WhisperBackend:
    """Fallback backend using OpenAI's reference Whisper implementation."""

    def __init__(self, model_name: str, device: str, language: Optional[str]) -> None:
        import whisper

        self._model = whisper.load_model(model_name, device=device)
        self._language = language or None
        self._use_fp16 = device != 'cpu'

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[Tuple[str, float]]:
        if np is None:
            raise RuntimeError('numpy is required for transcription')
        audio = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32)
        if audio.size == 0:
            return None
        audio = audio / 32768.0

        result = self._model.transcribe(audio, language=self._language, fp16=self._use_fp16)
        text = str(result.get('text', '')).strip()
        if not text:
            return None
        avg_logprob = result.get('avg_logprob')
        if avg_logprob is not None:
            confidence = _logprob_to_confidence(float(avg_logprob))
        else:
            confidence = float(result.get('language_probability') or 0.0)
        return text, confidence


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
    on_result: Callable[[str, float], None]
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
        if not result:
            return
        text, confidence = result
        text = (text or '').strip()
        if not text:
            return
        self.on_result(text, float(confidence))


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

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[Tuple[str, float]]:
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
    ) -> Optional[Tuple[str, float]]:
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
                return text, float(confidence or 0.0)
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

    def transcribe(self, pcm_bytes: bytes, sample_rate: int) -> Optional[Tuple[str, float]]:
        now = self._monotonic()
        if self._primary is not None and now >= self._next_retry:
            try:
                return self._primary.transcribe(pcm_bytes, sample_rate)
            except Exception as exc:
                if self._logger:
                    self._logger.warning(f'Remote ASR failed: {exc}. Falling back to onboard decoder.')
                self._next_retry = now + self._cooldown

        if self._fallback is None:
            return None

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
    """ROS 2 node that feeds VAD segments into an ASR backend."""

    def __init__(self, backend: Optional[object] = None) -> None:  # pragma: no cover - requires ROS
        super().__init__('transcriber')

        self._segment_topic = self.declare_parameter('segment_topic', '/audio/speech_segment').value
        self._transcript_topic = self.declare_parameter('transcript_topic', '/audio/transcription').value
        self._speaker_label = self.declare_parameter('speaker', 'user').value
        self._sample_rate = int(self.declare_parameter('segment_sample_rate', 16000).value)
        self._model_name = self.declare_parameter('model', 'base').value
        self._device = self.declare_parameter('device', 'cpu').value
        self._compute_type = self.declare_parameter('compute_type', 'int8').value
        self._language = self.declare_parameter('language', '').value or None
        self._beam_size = int(self.declare_parameter('beam_size', 5).value)

        default_remote_ws = os.getenv('EAR_ASR_WS_URL', 'ws://localhost:8082/ws')
        self._remote_ws_url = self.declare_parameter('remote_ws_url', default_remote_ws).value
        self._remote_connect_timeout = float(self.declare_parameter('remote_connect_timeout', 0.6).value)
        self._remote_response_timeout = float(self.declare_parameter('remote_response_timeout', 1.5).value)
        self._remote_retry_cooldown = float(self.declare_parameter('remote_retry_cooldown', 15.0).value)

        fallback_backend = backend or load_backend(
            self._model_name,
            self._device,
            self._compute_type,
            self._language,
            self._beam_size,
            logger=self.get_logger(),
        )
        remote_backend: Optional[RemoteAsrBackend] = None
        remote_uri = str(self._remote_ws_url or '').strip()
        if backend is None and remote_uri:
            remote_backend = initialise_remote_backend(
                uri=remote_uri,
                language=self._language,
                connect_timeout=self._remote_connect_timeout,
                response_timeout=self._remote_response_timeout,
                logger=self.get_logger(),
            )

        if remote_backend:
            self._backend = ChainedTranscriptionBackend(
                primary=remote_backend,
                fallback=fallback_backend,
                cooldown_seconds=max(0.0, self._remote_retry_cooldown),
                logger=self.get_logger(),
            )
        else:
            self._backend = fallback_backend

        self.transcript_pub = self.create_publisher(Transcript, self._transcript_topic, 10)
        self._segment_sub = self.create_subscription(ByteMultiArray, self._segment_topic, self._on_segment, 10)

        self._queue: 'queue.Queue[Optional[bytes]]' = queue.Queue()
        self._stop_evt = threading.Event()
        self._worker = TranscriptionWorker(
            backend=self._backend,
            sample_rate=self._sample_rate,
            speaker=str(self._speaker_label),
            on_result=self._publish_transcript,
            logger=self.get_logger(),
        )
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        backend_name = self._backend.__class__.__name__ if self._backend else 'None'
        self.get_logger().info(
            f'Transcriber ready: backend={backend_name} segments={self._segment_topic} transcripts={self._transcript_topic}'
        )

    def destroy_node(self) -> None:  # pragma: no cover - requires ROS
        self._stop_evt.set()
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass
        try:
            self._thread.join(timeout=2.0)
        except Exception:
            pass
        return super().destroy_node()

    def _on_segment(self, msg: ByteMultiArray) -> None:  # pragma: no cover - requires ROS
        data = coerce_pcm_bytes(msg.data)
        if not data:
            return
        self._queue.put(data)

    def _loop(self) -> None:  # pragma: no cover - requires ROS
        while not self._stop_evt.is_set():
            try:
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                self._queue.task_done()
                break
            try:
                self._worker.handle_segment(item)
            finally:
                self._queue.task_done()

    def _publish_transcript(self, text: str, confidence: float) -> None:  # pragma: no cover - requires ROS
        msg = Transcript()
        msg.text = text
        msg.speaker = str(self._speaker_label)
        msg.confidence = float(confidence)
        self.transcript_pub.publish(msg)


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
    'load_backend',
    'FasterWhisperBackend',
    'WhisperBackend',
    'RemoteAsrBackend',
    'ChainedTranscriptionBackend',
]
