"""Unit tests for the segment accumulation helper."""
from __future__ import annotations

from pathlib import Path
import sys

package_root = str(Path(__file__).resolve().parents[1])
if package_root not in sys.path:
    sys.path.insert(0, package_root)

from ear.segment_accumulator_node import SegmentAccumulator  # noqa: E402


def test_accumulator_resets_after_timeout() -> None:
    """Segments that arrive after a long pause should start a new buffer."""
    now = [0.0]

    def time_source() -> float:
        return now[0]

    accumulator = SegmentAccumulator(reset_timeout=0.2, max_segments=5, time_source=time_source)

    first = accumulator.add_segment(b"abc")
    assert first == b"abc"

    now[0] = 0.1
    second = accumulator.add_segment(b"def")
    assert second == b"abcdef"

    # Advance beyond the timeout to force a reset.
    now[0] = 1.0
    third = accumulator.add_segment(b"ghi")
    assert third == b"ghi"


def test_accumulator_enforces_max_segments() -> None:
    """The helper should reset once the configured segment limit is reached."""
    now = [0.0]

    accumulator = SegmentAccumulator(reset_timeout=10.0, max_segments=2, time_source=lambda: now[0])

    first = accumulator.add_segment(b"one")
    assert first == b"one"

    now[0] = 0.05
    second = accumulator.add_segment(b"two")
    assert second == b"onetwo"

    now[0] = 0.1
    third = accumulator.add_segment(b"three")
    assert third == b"three"
