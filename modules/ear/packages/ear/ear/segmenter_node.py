"""VAD-driven speech segmentation node and supporting helpers."""
from __future__ import annotations

import audioop
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, List, Optional

from .audio_utils import coerce_pcm_bytes
from .qos import best_effort_qos, sensor_data_qos

try:  # pragma: no cover - exercised only when ROS is available
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import ByteMultiArray, UInt32
    from psyched_msgs.msg import VadFrame
except ImportError:  # pragma: no cover - unit tests rely on these lightweight stubs
    rclpy = None  # type: ignore[assignment]

    class _LoggerStub:
        def info(self, msg: str) -> None:  # noqa: D401 - mimic rclpy logger API
            """Record informational messages (ignored in tests)."""

        def warning(self, msg: str) -> None:
            pass

        warn = warning

        def error(self, msg: str) -> None:
            pass

    class _PublisherStub:
        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.published: list[Any] = []

        def publish(self, msg: Any) -> None:
            self.published.append(msg)

    class _SubscriptionStub:
        def __init__(self, topic: str, callback) -> None:
            self.topic = topic
            self.callback = callback

    class Node:  # type: ignore[override]
        def __init__(self, name: str) -> None:
            self._name = name
            self._logger = _LoggerStub()

        def declare_parameter(self, name: str, default_value: Any) -> Any:
            return type('Param', (), {'value': default_value})()

        def create_publisher(self, msg_type: Any, topic: str, qos: Any) -> _PublisherStub:
            return _PublisherStub(topic)

        def create_subscription(self, msg_type: Any, topic: str, callback, qos: Any) -> _SubscriptionStub:
            return _SubscriptionStub(topic, callback)

        def get_logger(self) -> _LoggerStub:
            return self._logger

        def get_clock(self) -> Any:
            class _Clock:
                @staticmethod
                def now() -> Any:
                    class _Now:
                        @staticmethod
                        def to_msg() -> Any:
                            return type('Time', (), {'sec': 0, 'nanosec': 0})()

                    return _Now()

            return _Clock()

    @dataclass
    class ByteMultiArray:  # type: ignore[override]
        data: bytes = b""

    @dataclass
    class UInt32:  # type: ignore[override]
        data: int = 0

    class VadFrame:  # type: ignore[override]
        def __init__(self, **kwargs: Any) -> None:
            self.stamp = kwargs.get('stamp')
            self.sample_rate = kwargs.get('sample_rate', 0)
            self.frame_samples = kwargs.get('frame_samples', 0)
            self.is_speech = kwargs.get('is_speech', False)
            self.audio = kwargs.get('audio', b"")


@dataclass(frozen=True)
class SegmentUpdate:
    """Result emitted after processing an individual VAD frame."""

    duration_ms: int
    segment: Optional[bytes] = None
    active: bool = False


@dataclass(frozen=True)
class FrameChunk:
    """Container for a single VAD frame worth of PCM audio."""

    pcm: bytes
    is_speech: bool
    samples: int


class SpeechSegmenter:
    """Stateful helper that converts VAD frames into PCM speech segments."""

    def __init__(
        self,
        *,
        silence_release_ms: float = 450.0,
        lead_silence_ms: float = 120.0,
        min_speech_ms: float = 300.0,
        max_segment_ms: Optional[float] = 12000.0,
        trim_window_ms: float = 30.0,
        trim_keep_ms: float = 60.0,
        trim_rms_ratio: float = 0.12,
        trim_rms_floor: float = 200.0,
    ) -> None:
        self._silence_release_ms = max(0.0, float(silence_release_ms))
        self._lead_silence_ms = max(0.0, float(lead_silence_ms))
        self._min_speech_ms = max(0.0, float(min_speech_ms))
        max_segment = float(max_segment_ms) if max_segment_ms not in (None, 0.0) else None
        self._max_segment_ms = max_segment if (max_segment is None or max_segment > 0.0) else None
        self._trim_window_ms = max(1.0, float(trim_window_ms))
        self._trim_keep_ms = max(0.0, float(trim_keep_ms))
        self._trim_rms_ratio = max(0.0, float(trim_rms_ratio))
        self._trim_rms_floor = max(0.0, float(trim_rms_floor))

        self._sample_rate: Optional[int] = None
        self._silence_release_samples = 0
        self._lead_silence_samples = 0
        self._min_speech_samples = 0
        self._max_segment_samples: Optional[int] = None

        self._frames: List[FrameChunk] = []
        self._lead_frames: Deque[FrameChunk] = deque()
        self._lead_samples = 0
        self._active = False
        self._speech_samples = 0
        self._total_samples = 0
        self._trailing_silence_samples = 0
        self._trim_window_samples = 0
        self._trim_keep_samples = 0

    def reset(self, *, preserve_rate: bool = True) -> None:
        """Clear any buffered state."""

        self._frames.clear()
        self._active = False
        self._speech_samples = 0
        self._total_samples = 0
        self._trailing_silence_samples = 0
        if not preserve_rate:
            self._sample_rate = None
            self._silence_release_samples = 0
            self._lead_silence_samples = 0
            self._min_speech_samples = 0
            self._max_segment_samples = None
        self._lead_frames.clear()
        self._lead_samples = 0

    def process_frame(self, frame: VadFrame) -> SegmentUpdate:
        """Consume *frame* and return the updated duration/segment state."""

        pcm = coerce_pcm_bytes(getattr(frame, 'audio', b""))
        sample_rate_raw = int(getattr(frame, 'sample_rate', 0) or 0)
        frame_samples = int(getattr(frame, 'frame_samples', 0) or 0)
        if frame_samples <= 0 and pcm:
            frame_samples = len(pcm) // 2
        if frame_samples < 0:
            frame_samples = 0

        completed_segment: Optional[bytes] = None

        if sample_rate_raw > 0 and self._sample_rate and sample_rate_raw != self._sample_rate:
            completed_segment = self._finalise_segment(force=True) or completed_segment
            self.reset(preserve_rate=False)

        if self._sample_rate is None:
            rate = sample_rate_raw or 16000
            self._configure_for_rate(rate)
        elif sample_rate_raw > 0 and sample_rate_raw != self._sample_rate:
            self._configure_for_rate(sample_rate_raw)

        is_speech = bool(getattr(frame, 'is_speech', False))

        if is_speech and frame_samples > 0 and pcm:
            self._handle_speech_frame(pcm, frame_samples)
        elif is_speech:
            # No audio to append; report current duration/state.
            pass
        elif self._active:
            self._handle_silence_frame(pcm, frame_samples)
            if self._max_segment_samples and self._total_samples >= self._max_segment_samples:
                completed_segment = completed_segment or self._finalise_segment(force=True)
            elif self._silence_release_samples and self._trailing_silence_samples >= self._silence_release_samples:
                completed_segment = completed_segment or self._finalise_segment(force=False)
        else:
            self._append_lead_silence(pcm, frame_samples)

        duration_ms = self._current_duration_ms()
        active = self._active
        return SegmentUpdate(duration_ms=duration_ms, segment=completed_segment, active=active)

    def _handle_speech_frame(self, pcm: bytes, samples: int) -> None:
        if self._sample_rate is None or samples <= 0:
            return
        if not self._active:
            self._activate_segment()
        chunk = FrameChunk(pcm=pcm, is_speech=True, samples=samples)
        self._frames.append(chunk)
        self._speech_samples += samples
        self._total_samples += samples
        self._trailing_silence_samples = 0

    def _handle_silence_frame(self, pcm: bytes, samples: int) -> None:
        if self._sample_rate is None or samples <= 0:
            self._trailing_silence_samples = 0 if not self._frames else self._trailing_silence_samples
            return
        chunk = FrameChunk(pcm=pcm, is_speech=False, samples=samples)
        self._frames.append(chunk)
        self._total_samples += samples
        self._trailing_silence_samples += samples

    def _append_lead_silence(self, pcm: bytes, samples: int) -> None:
        if self._sample_rate is None or samples <= 0 or not pcm:
            return
        chunk = FrameChunk(pcm=pcm, is_speech=False, samples=samples)
        self._lead_frames.append(chunk)
        self._lead_samples += samples
        while self._lead_samples > self._lead_silence_samples and self._lead_frames:
            removed = self._lead_frames.popleft()
            self._lead_samples -= removed.samples

    def _activate_segment(self) -> None:
        self._active = True
        while self._lead_frames:
            chunk = self._lead_frames.popleft()
            if chunk.samples <= 0 or not chunk.pcm:
                continue
            self._frames.append(chunk)
            self._total_samples += chunk.samples
        self._lead_samples = 0

    def _finalise_segment(self, *, force: bool) -> Optional[bytes]:
        if not self._active:
            return None
        speech_samples = self._speech_samples
        if speech_samples <= 0:
            self.reset(preserve_rate=True)
            return None
        if not force and speech_samples < self._min_speech_samples:
            trailing = self._pop_trailing_silence()
            self.reset(preserve_rate=True)
            self._seed_lead_frames(trailing)
            return None

        trailing = self._pop_trailing_silence()
        payload_bytes = b"".join(chunk.pcm for chunk in self._frames if chunk.pcm)
        if not payload_bytes:
            self.reset(preserve_rate=True)
            self._seed_lead_frames(trailing)
            return None

        trimmed_payload = self._trim_silence(payload_bytes)
        if not trimmed_payload:
            trimmed_payload = payload_bytes
        if self._min_speech_samples and len(trimmed_payload) // 2 < self._min_speech_samples:
            self.reset(preserve_rate=True)
            self._seed_lead_frames(trailing)
            return None

        self.reset(preserve_rate=True)
        self._seed_lead_frames(trailing)
        return trimmed_payload

    def _pop_trailing_silence(self) -> List[FrameChunk]:
        trailing: List[FrameChunk] = []
        while self._frames and not self._frames[-1].is_speech:
            trailing.append(self._frames.pop())
        trailing.reverse()
        return trailing

    def _seed_lead_frames(self, frames: List[FrameChunk]) -> None:
        self._lead_frames.clear()
        self._lead_samples = 0
        if not frames:
            return
        carry_samples = 0
        for chunk in reversed(frames):
            if chunk.samples <= 0 or not chunk.pcm:
                continue
            proposed = carry_samples + chunk.samples
            if self._lead_silence_samples and proposed > self._lead_silence_samples:
                continue
            carry_samples = proposed
            self._lead_frames.appendleft(chunk)
        self._lead_samples = carry_samples

    def _trim_silence(self, pcm: bytes) -> bytes:
        if not pcm or self._sample_rate is None or self._trim_window_samples <= 0:
            return pcm
        window_samples = self._trim_window_samples
        window_bytes = window_samples * 2
        if window_bytes <= 0:
            return pcm
        rms_values: List[int] = []
        chunks: List[bytes] = []
        for offset in range(0, len(pcm), window_bytes):
            chunk = pcm[offset : offset + window_bytes]
            if not chunk:
                break
            chunks.append(chunk)
            try:
                rms = audioop.rms(chunk, 2)
            except Exception:
                rms = 0
            rms_values.append(rms)
        if not rms_values:
            return pcm
        max_rms = max(rms_values)
        if max_rms <= 0:
            return pcm
        threshold = max(int(self._trim_rms_floor), int(max_rms * self._trim_rms_ratio))
        if threshold <= 0:
            return pcm
        start_index = None
        for idx, value in enumerate(rms_values):
            if value >= threshold:
                start_index = idx
                break
        if start_index is None:
            return pcm
        end_index = None
        for idx, value in enumerate(reversed(rms_values)):
            if value >= threshold:
                end_index = len(rms_values) - idx - 1
                break
        if end_index is None or end_index < start_index:
            return pcm
        keep_start_samples = max(0, start_index * window_samples - self._trim_keep_samples)
        keep_end_samples = min(len(pcm) // 2, (end_index + 1) * window_samples + self._trim_keep_samples)
        keep_start_bytes = keep_start_samples * 2
        keep_end_bytes = keep_end_samples * 2
        trimmed = pcm[keep_start_bytes:keep_end_bytes]
        return trimmed if trimmed else pcm

    def _configure_for_rate(self, sample_rate: int) -> None:
        sample_rate = max(1, sample_rate)
        self._sample_rate = sample_rate
        self._silence_release_samples = (
            int(round(sample_rate * (self._silence_release_ms / 1000.0)))
            if self._silence_release_ms > 0.0
            else 0
        )
        self._lead_silence_samples = (
            int(round(sample_rate * (self._lead_silence_ms / 1000.0)))
            if self._lead_silence_ms > 0.0
            else 0
        )
        self._min_speech_samples = (
            int(round(sample_rate * (self._min_speech_ms / 1000.0)))
            if self._min_speech_ms > 0.0
            else 0
        )
        self._max_segment_samples = (
            int(round(sample_rate * (self._max_segment_ms / 1000.0)))
            if self._max_segment_ms
            else None
        )
        self._trim_window_samples = max(1, int(round(sample_rate * (self._trim_window_ms / 1000.0))))
        self._trim_keep_samples = max(0, int(round(sample_rate * (self._trim_keep_ms / 1000.0))))

    def _current_duration_ms(self) -> int:
        if not self._active or not self._sample_rate:
            return 0
        effective_samples = max(0, self._total_samples - self._trailing_silence_samples)
        return int(round((effective_samples / self._sample_rate) * 1000))


class SegmenterNode(Node):  # type: ignore[misc]
    """ROS node that converts VAD frames into speech segments."""

    def __init__(self) -> None:  # pragma: no cover - requires ROS
        super().__init__('speech_segmenter')

        self._frame_topic = self.declare_parameter('frame_topic', '/audio/vad_frames').value
        self._segment_topic = self.declare_parameter('segment_topic', '/audio/speech_segment').value
        self._accum_topic = self.declare_parameter(
            'accumulating_topic', '/audio/speech_segment_accumulating'
        ).value
        self._duration_topic = self.declare_parameter('duration_topic', '/audio/speech_duration').value

        silence_release = float(self.declare_parameter('silence_release_ms', 450.0).value)
        lead_silence = float(self.declare_parameter('lead_silence_ms', 120.0).value)
        min_speech = float(self.declare_parameter('min_speech_ms', 300.0).value)
        max_segment = self.declare_parameter('max_segment_ms', 12000.0).value
        max_segment_ms = float(max_segment) if max_segment is not None else 0.0
        trim_window = float(self.declare_parameter('trim_window_ms', 30.0).value)
        trim_keep = float(self.declare_parameter('trim_keep_ms', 60.0).value)
        trim_ratio = float(self.declare_parameter('trim_rms_ratio', 0.12).value)
        trim_floor = float(self.declare_parameter('trim_rms_floor', 200.0).value)

        self._segmenter = SpeechSegmenter(
            silence_release_ms=silence_release,
            lead_silence_ms=lead_silence,
            min_speech_ms=min_speech,
            max_segment_ms=max_segment_ms,
            trim_window_ms=trim_window,
            trim_keep_ms=trim_keep,
            trim_rms_ratio=trim_ratio,
            trim_rms_floor=trim_floor,
        )
        self._current_accum = bytearray()
        self._segment_active = False

        self._segment_pub = self.create_publisher(ByteMultiArray, self._segment_topic, sensor_data_qos())
        self._accum_pub = self.create_publisher(ByteMultiArray, self._accum_topic, sensor_data_qos())
        self._duration_pub = self.create_publisher(UInt32, self._duration_topic, best_effort_qos())
        self._subscription = self.create_subscription(VadFrame, self._frame_topic, self._on_frame, sensor_data_qos())

        self.get_logger().info(
            (
                'Speech segmenter ready: frame=%s segment=%s accum=%s duration=%s '
                'silence_release=%.1fms lead_silence=%.1fms min_speech=%.1fms max_segment=%.1fms '
                'trim_window=%.1fms trim_keep=%.1fms trim_ratio=%.2f trim_floor=%.0f'
                % (
                    self._frame_topic,
                    self._segment_topic,
                    self._accum_topic,
                    self._duration_topic,
                    silence_release,
                    lead_silence,
                    min_speech,
                    max_segment_ms,
                    trim_window,
                    trim_keep,
                    trim_ratio,
                    trim_floor,
                )
            )
        )

    def _on_frame(self, msg: VadFrame) -> None:  # pragma: no cover - requires ROS
        prev_active = self._segment_active
        update = self._segmenter.process_frame(msg)

        duration_msg = UInt32()
        duration_msg.data = max(0, update.duration_ms)
        self._duration_pub.publish(duration_msg)

        is_speech = bool(getattr(msg, 'is_speech', False))
        if is_speech:
            pcm = coerce_pcm_bytes(getattr(msg, 'audio', b""))
            if pcm:
                self._current_accum.extend(pcm)
                accum_msg = ByteMultiArray()
                accum_msg.data = bytes(self._current_accum)
                self._accum_pub.publish(accum_msg)

        active = bool(update.active)
        segment_payload: Optional[bytes] = None
        if update.segment is not None:
            segment_payload = bytes(update.segment)
        elif prev_active and not active and self._current_accum:
            segment_payload = bytes(self._current_accum)

        if segment_payload is not None:
            segment_msg = ByteMultiArray()
            segment_msg.data = segment_payload
            self._segment_pub.publish(segment_msg)
            self._current_accum.clear()
        elif not is_speech and not active:
            self._current_accum.clear()

        self._segment_active = active


def main(args: Any = None) -> None:  # pragma: no cover - requires ROS
    if rclpy is None:
        raise RuntimeError('rclpy is required to run SegmenterNode')

    rclpy.init(args=args)
    node = SegmenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['SegmenterNode', 'SpeechSegmenter', 'SegmentUpdate']
