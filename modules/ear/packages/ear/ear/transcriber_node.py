"""ROS 2 bridge that converts voiced audio segments into transcripts."""
from __future__ import annotations

import math
import queue
import threading
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

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

        self._backend = backend or load_backend(
            self._model_name,
            self._device,
            self._compute_type,
            self._language,
            self._beam_size,
            logger=self.get_logger(),
        )

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
]
