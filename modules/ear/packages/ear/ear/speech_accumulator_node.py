"""Aggregate successive speech segments into a longer context buffer."""
from __future__ import annotations

import time
from typing import Optional

try:  # Optional ROS imports for runtime usage.
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import ByteMultiArray
except ImportError:  # pragma: no cover - unit tests stub out ROS.
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    ByteMultiArray = object  # type: ignore

from .audio_utils import coerce_pcm_bytes


class SpeechAccumulatorNode(Node):  # type: ignore[misc]
    """ROS 2 node that accumulates silence-delimited segments into a longer window."""

    def __init__(self) -> None:  # pragma: no cover - requires ROS
        super().__init__('speech_accumulator')

        self._segment_topic = self.declare_parameter('segment_topic', '/audio/speech_segment').value
        self._accum_topic = self.declare_parameter('accum_topic', '/audio/speech_accumulating').value
        self._reset_timeout = float(self.declare_parameter('reset_timeout', 12.0).value)
        self._max_segments = int(self.declare_parameter('max_segments', 8).value)

        self._buffer = bytearray()
        self._segments = 0
        self._last_segment_at: Optional[float] = None

        self._publisher = self.create_publisher(ByteMultiArray, self._accum_topic, 10)
        self._subscription = self.create_subscription(ByteMultiArray, self._segment_topic, self._on_segment, 10)
        self.get_logger().info(
            f'Speech accumulator ready: segment={self._segment_topic} accum={self._accum_topic} reset_timeout={self._reset_timeout:.1f}s max_segments={self._max_segments}'
        )

    def _on_segment(self, msg: ByteMultiArray) -> None:  # pragma: no cover - requires ROS
        pcm = coerce_pcm_bytes(msg.data)
        if not pcm:
            return
        now = time.monotonic()
        should_reset = False
        if self._last_segment_at is None:
            should_reset = True
        else:
            gap = now - self._last_segment_at
            if gap > max(0.1, self._reset_timeout):
                should_reset = True
        if self._segments >= max(1, self._max_segments):
            should_reset = True

        if should_reset:
            self._buffer.clear()
            self._segments = 0

        self._buffer.extend(pcm)
        self._segments += 1
        self._last_segment_at = now

        message = ByteMultiArray()
        message.data = bytes(self._buffer)
        self._publisher.publish(message)


def main(args=None):  # pragma: no cover - requires ROS
    rclpy.init(args=args)
    node = SpeechAccumulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['SpeechAccumulatorNode']
