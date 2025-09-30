"""Aggregate successive speech segments into a longer context window."""
from __future__ import annotations

import time
from typing import Any, Callable, Optional

from .audio_utils import coerce_pcm_bytes
from .qos import sensor_data_qos

try:  # pragma: no cover - exercised only when ROS is available
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import ByteMultiArray
except ImportError:  # pragma: no cover - unit tests rely on these lightweight stubs
    rclpy = None  # type: ignore[assignment]

    class _LoggerStub:
        def info(self, msg: str) -> None:  # noqa: D401 - parity with rclpy logger
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

    class ByteMultiArray:  # type: ignore[override]
        def __init__(self, data: bytes = b"") -> None:
            self.data = data


class SegmentAccumulator:
    """Collect speech segments while enforcing gap and count limits."""

    def __init__(
        self,
        *,
        reset_timeout: float,
        max_segments: int,
        time_source: Optional[Callable[[], float]] = None,
    ) -> None:
        self._reset_timeout = max(0.0, float(reset_timeout))
        self._max_segments = max(1, int(max_segments))
        self._time = time_source or time.monotonic
        self._buffer = bytearray()
        self._segments = 0
        self._last_segment_at: Optional[float] = None

    def reset(self) -> None:
        """Clear accumulated state."""

        self._buffer.clear()
        self._segments = 0
        self._last_segment_at = None

    def add_segment(self, segment: bytes) -> bytes:
        """Append *segment* and return the updated aggregate buffer."""

        pcm = coerce_pcm_bytes(segment)
        if not pcm:
            return bytes(self._buffer)

        now = self._time()
        should_reset = False
        if self._last_segment_at is None:
            should_reset = True
        else:
            gap = now - self._last_segment_at
            if gap > max(0.0, self._reset_timeout):
                should_reset = True

        if self._segments >= self._max_segments:
            should_reset = True

        if should_reset:
            self.reset()

        self._buffer.extend(pcm)
        self._segments += 1
        self._last_segment_at = now
        return bytes(self._buffer)


class SegmentAccumulatorNode(Node):  # type: ignore[misc]
    """ROS node that publishes a rolling speech context window."""

    def __init__(self) -> None:  # pragma: no cover - requires ROS
        super().__init__('segment_accumulator')

        self._segment_topic = self.declare_parameter('segment_topic', '/audio/speech_segment').value
        self._accum_topic = self.declare_parameter('accum_topic', '/audio/speech_accumulating').value
        self._reset_timeout = float(self.declare_parameter('reset_timeout', 12.0).value)
        self._max_segments = int(self.declare_parameter('max_segments', 8).value)

        self._accumulator = SegmentAccumulator(
            reset_timeout=self._reset_timeout,
            max_segments=self._max_segments,
        )

        self._publisher = self.create_publisher(ByteMultiArray, self._accum_topic, sensor_data_qos())
        self._subscription = self.create_subscription(ByteMultiArray, self._segment_topic, self._on_segment, sensor_data_qos())

        self.get_logger().info(
            (
                'Segment accumulator ready: segment=%s accum=%s reset_timeout=%.1fs max_segments=%d'
                % (self._segment_topic, self._accum_topic, self._reset_timeout, self._max_segments)
            )
        )

    def _on_segment(self, msg: ByteMultiArray) -> None:  # pragma: no cover - requires ROS
        combined = self._accumulator.add_segment(getattr(msg, 'data', msg))
        if not combined:
            return
        outbound = ByteMultiArray()
        outbound.data = bytes(combined)
        self._publisher.publish(outbound)


def main(args: Any = None) -> None:  # pragma: no cover - requires ROS
    if rclpy is None:
        raise RuntimeError('rclpy is required to run SegmentAccumulatorNode')

    rclpy.init(args=args)
    node = SegmentAccumulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['SegmentAccumulatorNode', 'SegmentAccumulator']
