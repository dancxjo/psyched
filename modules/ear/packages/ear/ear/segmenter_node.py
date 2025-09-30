"""VAD-driven speech segmentation node and supporting helpers."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

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


class SpeechSegmenter:
    """Stateful helper that converts VAD frames into PCM speech segments."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._sample_rate: Optional[int] = None
        self._sample_count = 0

    def reset(self) -> None:
        """Clear any buffered state."""

        self._buffer.clear()
        self._sample_rate = None
        self._sample_count = 0

    def process_frame(self, frame: VadFrame) -> SegmentUpdate:
        """Consume *frame* and return the updated duration/segment state."""

        pcm = coerce_pcm_bytes(getattr(frame, 'audio', b""))
        sample_rate = int(getattr(frame, 'sample_rate', 0) or 0)
        frame_samples = int(getattr(frame, 'frame_samples', 0) or 0)
        if frame_samples <= 0 and pcm:
            frame_samples = len(pcm) // 2

        if getattr(frame, 'is_speech', False):
            if not pcm:
                return SegmentUpdate(duration_ms=self._current_duration_ms())

            if self._sample_rate is None and sample_rate > 0:
                self._sample_rate = sample_rate
            elif self._sample_rate and sample_rate and sample_rate != self._sample_rate:
                completed = bytes(self._buffer)
                self.reset()
                if completed:
                    if sample_rate > 0:
                        self._sample_rate = sample_rate
                    if pcm:
                        self._buffer.extend(pcm)
                        self._sample_count = max(0, frame_samples)
                    return SegmentUpdate(duration_ms=self._current_duration_ms(), segment=completed)

            self._buffer.extend(pcm)
            self._sample_count += max(0, frame_samples)
            if self._sample_rate is None:
                self._sample_rate = sample_rate or 16000

            return SegmentUpdate(duration_ms=self._current_duration_ms())

        if self._buffer:
            segment = bytes(self._buffer)
            self.reset()
            return SegmentUpdate(duration_ms=0, segment=segment)

        self.reset()
        return SegmentUpdate(duration_ms=0)

    def _current_duration_ms(self) -> int:
        if self._sample_rate and self._sample_rate > 0:
            return int(round((self._sample_count / self._sample_rate) * 1000))
        return 0


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

        self._segmenter = SpeechSegmenter()
        self._current_accum = bytearray()

        self._segment_pub = self.create_publisher(ByteMultiArray, self._segment_topic, sensor_data_qos())
        self._accum_pub = self.create_publisher(ByteMultiArray, self._accum_topic, sensor_data_qos())
        self._duration_pub = self.create_publisher(UInt32, self._duration_topic, best_effort_qos())
        self._subscription = self.create_subscription(VadFrame, self._frame_topic, self._on_frame, sensor_data_qos())

        self.get_logger().info(
            (
                'Speech segmenter ready: frame=%s segment=%s accum=%s duration=%s'
                % (self._frame_topic, self._segment_topic, self._accum_topic, self._duration_topic)
            )
        )

    def _on_frame(self, msg: VadFrame) -> None:  # pragma: no cover - requires ROS
        update = self._segmenter.process_frame(msg)

        duration_msg = UInt32()
        duration_msg.data = max(0, update.duration_ms)
        self._duration_pub.publish(duration_msg)

        if getattr(msg, 'is_speech', False):
            pcm = coerce_pcm_bytes(getattr(msg, 'audio', b""))
            if pcm:
                self._current_accum.extend(pcm)
                accum_msg = ByteMultiArray()
                accum_msg.data = bytes(self._current_accum)
                self._accum_pub.publish(accum_msg)
        elif update.segment is None:
            self._current_accum.clear()

        if update.segment is not None:
            segment_msg = ByteMultiArray()
            segment_msg.data = bytes(update.segment)
            self._segment_pub.publish(segment_msg)
            self._current_accum.clear()


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
