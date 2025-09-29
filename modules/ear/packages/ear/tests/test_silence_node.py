"""Tests for the silence monitoring ROS node.

These cover the behaviour of :class:`ear.silence_node.SilenceNode` using the
built-in ROS stubs provided when ``rclpy`` is unavailable.  The stubs expose
publishers and subscriptions so the unit tests can inject audio payloads and
inspect the resulting gauge updates without spinning a ROS executor.
"""
from __future__ import annotations

import struct
from pathlib import Path
import sys
from typing import Callable

package_root = str(Path(__file__).resolve().parents[1])
if package_root not in sys.path:
    sys.path.insert(0, package_root)

from ear.silence_node import ByteMultiArray, SilenceNode, SilenceTracker, UInt32


def _tracker_factory(time_source: Callable[[], float]) -> Callable[[float], SilenceTracker]:
    """Return a tracker factory bound to *time_source* for deterministic tests."""

    def factory(threshold: float) -> SilenceTracker:
        return SilenceTracker(silence_threshold=threshold, time_source=time_source)

    return factory


def test_silence_node_emits_duration_from_quiet_frames() -> None:
    """The node should accumulate silence while frames remain under threshold."""
    fake_now = [0.0]

    node = SilenceNode(tracker_factory=_tracker_factory(lambda: fake_now[0]))

    # The stub publisher keeps every message in a ``published`` list for
    # inspection during tests.
    publisher = node._publisher  # type: ignore[attr-defined]
    assert publisher.topic == '/audio/silence_ms'

    quiet_frame = struct.pack('<4h', 0, 0, 0, 0)
    node._on_audio(ByteMultiArray(data=quiet_frame))
    assert isinstance(publisher.published[-1], UInt32)
    assert publisher.published[-1].data == 0

    fake_now[0] = 0.5
    node._on_audio(ByteMultiArray(data=quiet_frame))
    assert publisher.published[-1].data == 500

    fake_now[0] = 1.0
    node._on_audio(ByteMultiArray(data=quiet_frame))
    assert publisher.published[-1].data == 1000


def test_silence_node_resets_when_audio_exceeds_threshold() -> None:
    """High-RMS frames should reset the silence gauge to zero."""
    fake_now = [0.0]

    node = SilenceNode(tracker_factory=_tracker_factory(lambda: fake_now[0]))
    publisher = node._publisher  # type: ignore[attr-defined]

    quiet_frame = struct.pack('<4h', 0, 0, 0, 0)
    node._on_audio(ByteMultiArray(data=quiet_frame))
    fake_now[0] = 0.5
    node._on_audio(ByteMultiArray(data=quiet_frame))
    assert publisher.published[-1].data == 500

    loud_frame = struct.pack('<4h', 6000, 6000, 6000, 6000)
    fake_now[0] = 0.6
    node._on_audio(ByteMultiArray(data=loud_frame))
    assert publisher.published[-1].data == 0


