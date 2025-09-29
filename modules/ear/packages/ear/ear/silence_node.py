"""ROS 2 node that derives a silence gauge from raw PCM audio.

The node subscribes to ``/audio/raw`` (``std_msgs/msg/ByteMultiArray``),
converts each message to PCM16 samples, and feeds their RMS amplitude into a
:class:`~ear.silence_tracker.SilenceTracker`.  The resulting duration in
milliseconds is emitted on ``/audio/silence_ms`` using ``std_msgs/msg/UInt32``.

A small set of ROS stubs is bundled for unit tests so the logic can be exercised
without a full ROS installation.  At runtime the real ``rclpy`` imports are
used automatically.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any, Callable, Optional

from .audio_utils import coerce_pcm_bytes
from .silence_tracker import SilenceTracker

try:  # pragma: no cover - exercised only in ROS environments
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import ByteMultiArray, UInt32
except ImportError:  # pragma: no cover - unit tests rely on these lightweight stubs
    rclpy = None  # type: ignore[assignment]

    class _LoggerStub:
        def info(self, msg: str) -> None:  # noqa: D401 - parity with rclpy logger
            """Log informational messages (ignored during tests)."""

        def warning(self, msg: str) -> None:
            pass

        # ``rclpy`` exposes ``warn``; keep compatibility with existing code.
        warn = warning

        def error(self, msg: str) -> None:
            pass

    class _PublisherStub:
        """Minimal publisher that records outbound messages for assertions."""

        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.published: list[Any] = []

        def publish(self, msg: Any) -> None:
            self.published.append(msg)

    class _SubscriptionStub:
        def __init__(self, topic: str, callback: Callable[[Any], None]) -> None:
            self.topic = topic
            self.callback = callback

    class Node:  # type: ignore[override]
        """Lightweight stand-in for :class:`rclpy.node.Node` used in tests."""

        def __init__(self, name: str) -> None:
            self._name = name
            self._logger = _LoggerStub()

        def declare_parameter(self, name: str, default_value: Any) -> SimpleNamespace:
            return SimpleNamespace(value=default_value)

        def create_publisher(self, msg_type: Any, topic: str, qos: int) -> _PublisherStub:
            return _PublisherStub(topic)

        def create_subscription(self, msg_type: Any, topic: str, callback: Callable[[Any], None], qos: int) -> _SubscriptionStub:
            return _SubscriptionStub(topic, callback)

        def get_logger(self) -> _LoggerStub:
            return self._logger

    @dataclass
    class ByteMultiArray:  # type: ignore[override]
        data: Any = b""

    @dataclass
    class UInt32:  # type: ignore[override]
        data: int = 0


def _calculate_rms(pcm: bytes) -> float:
    """Return the RMS amplitude of *pcm* assuming little-endian PCM16 samples."""
    if not pcm:
        return 0.0

    sample_count = len(pcm) // 2
    if sample_count == 0:
        return 0.0

    # Truncate any dangling byte to preserve struct alignment; PyAudio provides
    # well-formed buffers but callers may pass trimmed payloads during tests.
    if len(pcm) % 2:
        pcm = pcm[:-1]
        sample_count -= 1
        if sample_count == 0:
            return 0.0

    samples = struct.unpack(f"<{sample_count}h", pcm)
    mean_square = sum(sample * sample for sample in samples) / sample_count
    return math.sqrt(mean_square)


class SilenceNode(Node):  # type: ignore[misc]
    """ROS-compatible node that publishes the ``/audio/silence_ms`` gauge."""

    def __init__(self, tracker_factory: Optional[Callable[[float], SilenceTracker]] = None) -> None:
        super().__init__('silence_monitor')

        self._silence_threshold = float(self.declare_parameter('silence_threshold', 500.0).value)
        factory = tracker_factory or (lambda threshold: SilenceTracker(threshold))
        self._tracker = factory(self._silence_threshold)

        self._publisher = self.create_publisher(UInt32, '/audio/silence_ms', 10)
        self._subscription = self.create_subscription(ByteMultiArray, '/audio/raw', self._on_audio, 10)

        self.get_logger().info(f'Silence monitor ready: threshold={self._silence_threshold:.1f}')

    def _on_audio(self, msg: ByteMultiArray) -> None:
        """Handle inbound raw audio payloads."""
        try:
            pcm = coerce_pcm_bytes(getattr(msg, 'data', msg))
        except TypeError as exc:
            self.get_logger().warning(f'Failed to decode audio payload: {exc}')
            return

        rms = _calculate_rms(pcm)
        silence_ms = self._tracker.update(rms=rms)

        message = UInt32()
        message.data = silence_ms
        self._publisher.publish(message)


def main(args: Any = None) -> None:  # pragma: no cover - requires ROS
    if rclpy is None:
        raise RuntimeError('rclpy is required to run SilenceNode')

    rclpy.init(args=args)
    node = SilenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['SilenceNode', 'SilenceTracker', 'ByteMultiArray', 'UInt32']
