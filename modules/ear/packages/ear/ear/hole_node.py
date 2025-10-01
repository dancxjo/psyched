"""Unified audio capture and transcription node for the ear module."""
from __future__ import annotations

try:  # pragma: no cover - dependency availability varies across Python builds
    import audioop
except ImportError:  # pragma: no cover - allow importing the module when audioop is missing
    class _AudioOpCompat:
        def __getattr__(self, name: str):
            raise RuntimeError("audioop module is required for runtime audio processing")

    audioop = _AudioOpCompat()  # type: ignore[assignment]
import io
import subprocess
import threading
import time
from collections import deque
from contextlib import suppress
from dataclasses import dataclass, field
from typing import Callable, Deque, Iterable, List, Optional, Sequence, Tuple

import webrtcvad

from .qos import best_effort_qos, sensor_data_qos
from .transcription_backends import (
    ChainedTranscriptionBackend,
    initialise_remote_backend,
    load_backend,
)
from .transcription_types import (
    TranscriptionResult,
    WordData,
    coalesce_words,
    result_is_usable,
)

try:  # pragma: no cover - runtime-only ROS imports
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import ByteMultiArray, String
    from psyched_msgs.msg import Transcript, TranscriptSegment, TranscriptWord
except ImportError:  # pragma: no cover - allow unit tests without ROS
    rclpy = None  # type: ignore[assignment]

    class _LoggerStub:
        def info(self, msg: str) -> None:
            pass

        def warning(self, msg: str) -> None:
            pass

        def error(self, msg: str) -> None:
            pass

    class _PublisherStub:
        def __init__(self) -> None:
            self.published: List[object] = []

        def publish(self, msg: object) -> None:
            self.published.append(msg)

    class Node:  # type: ignore[override]
        def __init__(self, name: str) -> None:
            self._name = name
            self._parameters = {}
            self._logger = _LoggerStub()

        def declare_parameter(self, name: str, default_value: object) -> object:
            self._parameters[name] = default_value
            return type("_Param", (), {"value": default_value})()

        def get_logger(self) -> _LoggerStub:
            return self._logger

        def create_publisher(self, *_args, **_kwargs) -> _PublisherStub:
            return _PublisherStub()

        def destroy_node(self) -> None:
            pass

    class ByteMultiArray:  # type: ignore[override]
        def __init__(self, data: Iterable[int] | bytes | bytearray | None = None) -> None:
            self.data = bytes(data or b"")

    class String:  # type: ignore[override]
        def __init__(self) -> None:
            self.data: str = ""

    @dataclass
    class TranscriptWord:  # type: ignore[override]
        start: float = 0.0
        end: float = 0.0
        text: str = ""

    @dataclass
    class TranscriptSegment:  # type: ignore[override]
        start: float = 0.0
        end: float = 0.0
        text: str = ""
        speaker: str = ""

    @dataclass
    class Transcript:  # type: ignore[override]
        text: str = ""
        speaker: str = ""
        confidence: float = 0.0
        segments: List[TranscriptSegment] = field(default_factory=list)
        words: List[TranscriptWord] = field(default_factory=list)


PopenFactory = Callable[..., subprocess.Popen]


@dataclass
class StableHead:
    """Container describing the stabilised portion of a transcript."""

    words: List[WordData]
    duration: float

    @property
    def text(self) -> str:
        return _words_to_text(self.words)

    @property
    def end_time(self) -> float:
        return self.words[-1].end if self.words else 0.0


class HeadStabilityTracker:
    """Detect when the leading words of successive transcripts stabilise."""

    def __init__(self, window: int, min_words: int, min_duration: float) -> None:
        self._window = max(1, int(window))
        self._min_words = max(0, int(min_words))
        self._min_duration = max(0.0, float(min_duration))
        self._history: Deque[List[WordData]] = deque(maxlen=self._window)

    def reset(self) -> None:
        self._history.clear()

    def seed(self, words: Sequence[WordData]) -> None:
        if not words:
            return
        self._history.append([WordData(start=w.start, end=w.end, text=w.text) for w in words])

    def update(self, words: Sequence[WordData]) -> Optional[StableHead]:
        if not words:
            return None
        self._history.append([WordData(start=w.start, end=w.end, text=w.text) for w in words])
        if len(self._history) < self._window:
            return None
        candidate = self._common_prefix()
        if not candidate:
            return None
        duration = candidate[-1].end - candidate[0].start if len(candidate) > 1 else candidate[0].end - candidate[0].start
        if len(candidate) < self._min_words or duration < self._min_duration:
            return None
        return StableHead(words=candidate, duration=max(0.0, duration))

    def _common_prefix(self) -> List[WordData]:
        if not self._history:
            return []
        length = min(len(entry) for entry in self._history)
        prefix: List[WordData] = []
        for index in range(length):
            exemplar = self._history[0][index]
            if any(entry[index].text != exemplar.text for entry in self._history):
                break
            start = min(entry[index].start for entry in self._history)
            end = max(entry[index].end for entry in self._history)
            prefix.append(WordData(start=start, end=end, text=exemplar.text))
        return prefix


def _words_to_text(words: Sequence[WordData]) -> str:
    if not words:
        return ""
    tokens: List[str] = []
    for word in words:
        token = word.text.strip()
        if not token:
            continue
        if tokens and token in {".", ",", "!", "?", ":", ";", "'s"}:
            tokens[-1] = f"{tokens[-1]}{token}"
        elif tokens and token.startswith("'"):
            tokens[-1] = f"{tokens[-1]}{token}"
        else:
            tokens.append(token)
    return " ".join(tokens).strip()


def _build_transcript_message(
    *,
    text: str,
    words: Sequence[WordData],
    speaker: str,
    confidence: float,
    offset: float,
) -> Transcript:
    msg = Transcript()
    msg.text = text
    msg.speaker = speaker
    msg.confidence = float(confidence)
    msg.segments = []
    if words:
        segment = TranscriptSegment()
        segment.start = float(offset + words[0].start)
        segment.end = float(offset + words[-1].end)
        segment.text = text
        segment.speaker = speaker
        msg.segments.append(segment)
    msg.words = []
    for word in words:
        word_msg = TranscriptWord()
        word_msg.start = float(offset + word.start)
        word_msg.end = float(offset + word.end)
        word_msg.text = word.text
        msg.words.append(word_msg)
    return msg


def _pcm_to_wav_bytes(pcm: bytes, sample_rate: int) -> bytes:
    if not pcm:
        return b""
    buffer = io.BytesIO()
    with suppress(Exception):
        import wave

        with wave.open(buffer, "wb") as handle:
            handle.setnchannels(1)
            handle.setsampwidth(2)
            handle.setframerate(sample_rate)
            handle.writeframes(pcm)
        return buffer.getvalue()
    return pcm


class HoleNode(Node):  # type: ignore[misc]
    """Single-node audio capture and transcription pipeline."""

    def __init__(
        self,
        *,
        popen_factory: Optional[PopenFactory] = None,
        parameter_overrides: Optional[dict[str, object]] = None,
    ) -> None:
        super().__init__("hole")
        overrides = parameter_overrides or {}
        self._popen_factory = popen_factory or subprocess.Popen
        self._logger = self.get_logger()

        self._chunk_size = self._get_int_param("chunk_size", 2048, overrides)
        self._sample_rate = self._get_int_param("sample_rate", 44100, overrides)
        self._channels = self._get_int_param("channels", 1, overrides)
        self._device_id = self._get_int_param("device_id", 0, overrides)
        self._alsa_device = self._get_str_param("alsa_device", "", overrides)

        self._segment_sample_rate = self._get_int_param("segment_sample_rate", 16000, overrides)
        self._speaker_label = self._get_str_param("speaker", "user", overrides)
        self._model_name = self._get_str_param("model", "small", overrides)
        self._device_type = self._get_str_param("device", "gpu", overrides)
        self._compute_type = self._get_str_param("compute_type", "int8", overrides)
        self._language = self._get_str_param("language", "", overrides).strip() or None
        self._beam_size = self._get_int_param("beam_size", 5, overrides)

        remote_default = self._get_str_param("remote_ws_url", "", overrides)
        self._remote_ws_url = remote_default
        self._remote_connect_timeout = self._get_float_param("remote_connect_timeout", 0.6, overrides)
        self._remote_response_timeout = self._get_float_param("remote_response_timeout", 1.5, overrides)
        self._remote_audio_dump_dir = self._get_str_param("remote_audio_dump_dir", "", overrides)

        self._raw_topic = self._get_str_param("raw_topic", "/audio/raw", overrides)
        self._partial_text_topic = self._get_str_param("partial_text_topic", "/audio/transcript/partial", overrides)
        self._partial_detail_topic = self._get_str_param("partial_detailed_topic", "/audio/transcript/partial/timing", overrides)
        self._final_text_topic = self._get_str_param("final_text_topic", "/audio/transcript/final", overrides)
        self._final_detail_topic = self._get_str_param("final_detailed_topic", "/audio/transcript/final/timing", overrides)
        self._final_audio_topic = self._get_str_param("final_audio_topic", "/audio/transcript/final/audio", overrides)

        silence_flush_ms = self._get_float_param("silence_flush_ms", 1200.0, overrides)
        silence_padding_ms = self._get_float_param("silence_padding_ms", 240.0, overrides)
        head_window = self._get_int_param("head_stability_window", 3, overrides)
        head_min_words = self._get_int_param("head_min_words", 3, overrides)
        head_min_duration_ms = self._get_float_param("head_min_duration_ms", 450.0, overrides)
        request_interval_ms = self._get_float_param("request_interval_ms", 250.0, overrides)

        self._silence_flush_seconds = max(0.1, silence_flush_ms / 1000.0)
        self._silence_padding_seconds = max(0.0, silence_padding_ms / 1000.0)
        self._request_interval = max(0.05, request_interval_ms / 1000.0)

        self._audio_pub = self.create_publisher(ByteMultiArray, self._raw_topic, sensor_data_qos())
        self._partial_text_pub = self.create_publisher(String, self._partial_text_topic, best_effort_qos(depth=10))
        self._partial_detail_pub = self.create_publisher(Transcript, self._partial_detail_topic, best_effort_qos(depth=5))
        self._final_text_pub = self.create_publisher(String, self._final_text_topic, best_effort_qos(depth=10))
        self._final_detail_pub = self.create_publisher(Transcript, self._final_detail_topic, best_effort_qos(depth=5))
        self._final_audio_pub = self.create_publisher(ByteMultiArray, self._final_audio_topic, sensor_data_qos())

        self._vad = webrtcvad.Vad()
        self._vad.set_mode(2)

        self._frame_ms = 30
        self._frame_bytes = max(1, int(self._segment_sample_rate * (self._frame_ms / 1000.0)) * 2)
        self._ratecv_state: Optional[Tuple[object, ...]] = None
        self._resample_buffer = bytearray()

        self._buffer = bytearray()
        self._buffer_lock = threading.Lock()
        self._buffer_has_new_data = False
        self._force_finalize_pending = False
        self._timeline_offset = 0.0

        self._pending_silence = bytearray()
        self._pending_silence_duration = 0.0
        self._last_request_time = 0.0

        self._tracker = HeadStabilityTracker(
            window=head_window,
            min_words=head_min_words,
            min_duration=head_min_duration_ms / 1000.0,
        )

        self._backend = self._build_backend()

        self._stop_evt = threading.Event()
        self._new_data_evt = threading.Event()

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self._asr_thread = threading.Thread(target=self._asr_loop, daemon=True)
        self._asr_thread.start()

        self._proc: Optional[subprocess.Popen] = None

        self._logger.info(
            "Hole node initialised: device=%s rate=%sHz channels=%s chunk=%s partial_topic=%s",
            self._resolve_device(),
            self._sample_rate,
            self._channels,
            self._chunk_size,
            self._partial_text_topic,
        )

    # ------------------------------------------------------------------
    # Parameter helpers
    def _get_param(self, name: str, default: object, overrides: dict[str, object]) -> object:
        declared = self.declare_parameter(name, default)
        value = getattr(declared, "value", default)
        if name in overrides:
            value = overrides[name]
        return value

    def _get_int_param(self, name: str, default: int, overrides: dict[str, object]) -> int:
        value = self._get_param(name, default, overrides)
        try:
            return int(value)
        except Exception:
            return default

    def _get_float_param(self, name: str, default: float, overrides: dict[str, object]) -> float:
        value = self._get_param(name, default, overrides)
        try:
            return float(value)
        except Exception:
            return default

    def _get_str_param(self, name: str, default: str, overrides: dict[str, object]) -> str:
        value = self._get_param(name, default, overrides)
        try:
            return str(value)
        except Exception:
            return default

    # ------------------------------------------------------------------
    # Backend management
    def _build_backend(self):
        remote = None
        if self._remote_ws_url:
            remote = initialise_remote_backend(
                uri=self._remote_ws_url,
                language=self._language,
                connect_timeout=self._remote_connect_timeout,
                response_timeout=self._remote_response_timeout,
                logger=self._logger,
                dump_audio_dir=self._remote_audio_dump_dir or None,
            )
        fallback = load_backend(
            self._model_name,
            self._device_type,
            self._compute_type,
            self._language,
            self._beam_size,
            logger=self._logger,
        )
        if remote and fallback:
            return ChainedTranscriptionBackend(
                primary=remote,
                fallback=fallback,
                cooldown_seconds=3.0,
                logger=self._logger,
            )
        return remote or fallback

    # ------------------------------------------------------------------
    # Capture loop
    def _resolve_device(self) -> str:
        if self._alsa_device:
            return self._alsa_device
        return f"hw:{self._device_id},0"

    def _spawn_arecord(self) -> Optional[subprocess.Popen]:
        cmd = [
            "arecord",
            "-q",
            "-D",
            self._resolve_device(),
            "-f",
            "S16_LE",
            "-r",
            str(self._sample_rate),
            "-c",
            str(self._channels),
            "-t",
            "raw",
        ]
        try:
            proc = self._popen_factory(cmd, stdout=subprocess.PIPE)
            self._logger.info("spawned arecord: %s", " ".join(cmd))
            return proc
        except FileNotFoundError:
            self._logger.error("arecord not available; install `alsa-utils`.")
        except Exception as exc:
            self._logger.error("failed to spawn arecord: %s", exc)
        return None

    def _reader_loop(self) -> None:
        backoff = 0.5
        while not self._stop_evt.is_set():
            if self._proc is None or self._proc.poll() is not None:
                self._proc = self._spawn_arecord()
                if self._proc is None:
                    time.sleep(min(backoff, 5.0))
                    backoff = min(backoff * 2, 5.0)
                    continue
                backoff = 0.5
            assert self._proc.stdout is not None
            try:
                data = self._proc.stdout.read(self._chunk_size)
            except Exception as exc:  # pragma: no cover - runtime-only
                self._logger.error("error reading from arecord: %s", exc)
                self._restart_process()
                time.sleep(0.5)
                continue
            if self._stop_evt.is_set():
                break
            if not data:
                self._logger.warning("arecord produced no data; restarting")
                self._restart_process()
                time.sleep(0.2)
                continue
            try:
                self._handle_raw_chunk(data)
            except Exception as exc:  # pragma: no cover - defensive
                self._logger.error("failed to process audio chunk: %s", exc)
        self._cleanup_process()

    def _handle_raw_chunk(self, data: bytes) -> None:
        pcm = bytes(data)
        if not pcm:
            return
        msg = ByteMultiArray()
        msg.data = pcm
        self._audio_pub.publish(msg)

        mono = pcm
        if self._channels > 1:
            try:
                mono = audioop.tomono(pcm, 2, 0.5, 0.5)
            except Exception:
                mono = pcm
        try:
            resampled, self._ratecv_state = audioop.ratecv(
                mono,
                2,
                1,
                self._sample_rate,
                self._segment_sample_rate,
                self._ratecv_state,
            )
        except Exception as exc:
            self._logger.warning("Failed to resample audio chunk: %s", exc)
            return
        if not resampled:
            return
        self._resample_buffer.extend(resampled)
        self._drain_resample_buffer()

    def _drain_resample_buffer(self) -> None:
        frame_bytes = self._frame_bytes
        while len(self._resample_buffer) >= frame_bytes:
            frame = bytes(self._resample_buffer[:frame_bytes])
            del self._resample_buffer[:frame_bytes]
            self._process_frame(frame)

    def _process_frame(self, frame: bytes) -> None:
        samples = len(frame) // 2
        if samples <= 0:
            return
        try:
            is_speech = bool(self._vad.is_speech(frame, self._segment_sample_rate))
        except Exception as exc:
            self._logger.warning("VAD failure: %s", exc)
            is_speech = False
        if is_speech:
            self._append_speech_frame(frame)
        else:
            self._handle_silence_frame(frame)

    def _append_speech_frame(self, frame: bytes) -> None:
        if not frame:
            return
        if self._pending_silence and self._pending_silence_duration <= self._silence_padding_seconds:
            self._append_to_buffer(bytes(self._pending_silence))
        self._pending_silence.clear()
        self._pending_silence_duration = 0.0
        self._append_to_buffer(frame)

    def _handle_silence_frame(self, frame: bytes) -> None:
        frame_duration = len(frame) / (2 * self._segment_sample_rate)
        if not self._buffer:
            self._pending_silence.clear()
            self._pending_silence_duration = 0.0
            return
        self._pending_silence.extend(frame)
        self._pending_silence_duration += frame_duration
        if self._pending_silence_duration <= self._silence_padding_seconds:
            self._append_to_buffer(frame)
            return
        if self._pending_silence_duration >= self._silence_flush_seconds:
            self._request_force_finalize()
            self._pending_silence.clear()
            self._pending_silence_duration = 0.0

    def _append_to_buffer(self, pcm: bytes) -> None:
        if not pcm:
            return
        with self._buffer_lock:
            self._buffer.extend(pcm)
            self._buffer_has_new_data = True
        self._new_data_evt.set()

    def _request_force_finalize(self) -> None:
        with self._buffer_lock:
            if not self._buffer:
                self._force_finalize_pending = False
                return
            self._force_finalize_pending = True
        self._new_data_evt.set()

    def _restart_process(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        with suppress(Exception):
            if proc.poll() is None:
                proc.terminate()
        with suppress(Exception):
            if proc.stdout:
                proc.stdout.close()

    def _cleanup_process(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        with suppress(Exception):
            if proc.poll() is None:
                proc.terminate()
        with suppress(Exception):
            if proc.poll() is None:
                proc.kill()
        with suppress(Exception):
            if proc.stdout:
                proc.stdout.close()

    # ------------------------------------------------------------------
    # ASR loop
    def _asr_loop(self) -> None:
        while not self._stop_evt.is_set():
            triggered = self._new_data_evt.wait(timeout=0.2)
            if self._stop_evt.is_set():
                break
            if not triggered:
                continue
            self._new_data_evt.clear()
            while not self._stop_evt.is_set():
                snapshot, force = self._snapshot_buffer()
                if snapshot is None:
                    break
                if self._backend is None:
                    self._logger.warning("No ASR backend available; dropping audio")
                    self._drop_buffer()
                    break
                now = time.monotonic()
                delay = self._request_interval - (now - self._last_request_time)
                if delay > 0:
                    time.sleep(delay)
                try:
                    result = self._backend.transcribe(snapshot, self._segment_sample_rate)  # type: ignore[attr-defined]
                except Exception as exc:  # pragma: no cover - runtime error path
                    self._logger.warning("ASR request failed: %s", exc)
                    time.sleep(0.3)
                    continue
                self._last_request_time = time.monotonic()
                handled = self._handle_transcript(result, force=force)
                if not handled and force:
                    self._drop_buffer()
                if force:
                    break

    def _snapshot_buffer(self) -> Tuple[Optional[bytes], bool]:
        with self._buffer_lock:
            if not self._buffer:
                self._buffer_has_new_data = False
                self._force_finalize_pending = False
                return None, False
            if not self._buffer_has_new_data and not self._force_finalize_pending:
                return None, False
            data = bytes(self._buffer)
            force = self._force_finalize_pending
            self._buffer_has_new_data = False
        return data, force

    def _handle_transcript(self, result: Optional[TranscriptionResult], *, force: bool) -> bool:
        if not result_is_usable(result):
            if force:
                self._logger.warning("Forced flush but ASR returned no transcript; dropping buffer")
            return False
        assert result is not None
        words = self._ensure_words(result)
        text = result.text.strip() or _words_to_text(words)
        if text:
            msg = String()
            msg.data = text
            self._partial_text_pub.publish(msg)
            detail = _build_transcript_message(
                text=text,
                words=words,
                speaker=self._speaker_label,
                confidence=result.confidence,
                offset=self._timeline_offset,
            )
            self._partial_detail_pub.publish(detail)
        if force:
            consumed_duration = self._finalise_words(words, result.confidence)
            self._tracker.reset()
            if consumed_duration > 0.0:
                residual_words = [
                    WordData(
                        start=max(0.0, word.start - consumed_duration),
                        end=max(0.0, word.end - consumed_duration),
                        text=word.text,
                    )
                    for word in words
                    if word.end > consumed_duration
                ]
                if residual_words:
                    self._tracker.seed(residual_words)
            return True
        stable = self._tracker.update(words)
        if stable is None:
            return True
        consumed_duration = self._finalise_words(stable.words, result.confidence)
        trimmed_words = []
        if consumed_duration > 0.0:
            trimmed_words = [
                WordData(
                    start=max(0.0, word.start - consumed_duration),
                    end=max(0.0, word.end - consumed_duration),
                    text=word.text,
                )
                for word in words
                if word.end > consumed_duration
            ]
        self._tracker.reset()
        if trimmed_words:
            self._tracker.seed(trimmed_words)
        return True

    def _ensure_words(self, result: TranscriptionResult) -> List[WordData]:
        if result.words:
            return [WordData(start=w.start, end=w.end, text=w.text) for w in result.words if w.text]
        return coalesce_words(result.text, result.segments, result.words)

    def _finalise_words(
        self,
        words: Sequence[WordData],
        confidence: float,
    ) -> float:
        if not words:
            return 0.0
        text = _words_to_text(words)
        consumed_pcm, consumed_duration = self._consume_buffer(words[-1].end)
        if not consumed_pcm:
            return 0.0
        wav_bytes = _pcm_to_wav_bytes(consumed_pcm, self._segment_sample_rate)
        final_msg = String()
        final_msg.data = text
        self._final_text_pub.publish(final_msg)
        detail = _build_transcript_message(
            text=text,
            words=words,
            speaker=self._speaker_label,
            confidence=confidence,
            offset=self._timeline_offset,
        )
        self._final_detail_pub.publish(detail)
        audio_msg = ByteMultiArray()
        audio_msg.data = wav_bytes
        self._final_audio_pub.publish(audio_msg)
        self._timeline_offset += consumed_duration
        return consumed_duration

    def _consume_buffer(self, end_time: float) -> Tuple[bytes, float]:
        if end_time <= 0.0:
            return b"", 0.0
        with self._buffer_lock:
            if not self._buffer:
                return b"", 0.0
            total_samples = len(self._buffer) // 2
            target_samples = min(total_samples, max(0, int(round(end_time * self._segment_sample_rate))))
            if target_samples <= 0:
                return b"", 0.0
            consumed = bytes(self._buffer[: target_samples * 2])
            del self._buffer[: target_samples * 2]
            if not self._buffer:
                self._buffer_has_new_data = False
                self._force_finalize_pending = False
        duration = target_samples / self._segment_sample_rate
        return consumed, duration

    def _drop_buffer(self) -> None:
        with self._buffer_lock:
            self._buffer.clear()
            self._buffer_has_new_data = False
            self._force_finalize_pending = False
        self._tracker.reset()

    # ------------------------------------------------------------------
    # Shutdown
    def destroy_node(self) -> None:
        self._stop_evt.set()
        self._new_data_evt.set()
        if self._reader_thread.is_alive():
            with suppress(Exception):
                self._reader_thread.join(timeout=3.0)
        if self._asr_thread.is_alive():
            with suppress(Exception):
                self._asr_thread.join(timeout=3.0)
        self._cleanup_process()
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:  # pragma: no cover - requires ROS
    if rclpy is None:
        raise RuntimeError("rclpy is required to launch the hole node")
    rclpy.init(args=args)
    node = HoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["HoleNode", "HeadStabilityTracker", "StableHead", "main"]
