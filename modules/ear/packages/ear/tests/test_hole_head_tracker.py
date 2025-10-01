"""Unit tests for the head stability tracker used by the hole node."""
from __future__ import annotations

import sys
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from ear.hole_node import HeadStabilityTracker
from ear.transcription_types import WordData


def _words(tokens: list[str], *, start: float = 0.0, step: float = 0.25) -> list[WordData]:
    words: list[WordData] = []
    cursor = start
    for token in tokens:
        end = cursor + step
        words.append(WordData(start=cursor, end=end, text=token))
        cursor = end
    return words


def test_tracker_reports_common_prefix_only_after_window() -> None:
    tracker = HeadStabilityTracker(window=3, min_words=2, min_duration=0.0)

    assert tracker.update(_words(["hello", "world"])) is None
    assert tracker.update(_words(["hello", "world", "again"], start=0.1)) is None

    head = tracker.update(_words(["hello", "world", "there"], start=0.2))
    assert head is not None
    assert head.text == "hello world"
    assert head.duration > 0.0


def test_tracker_respects_minimum_duration() -> None:
    tracker = HeadStabilityTracker(window=2, min_words=4, min_duration=0.8)
    tracker.update(_words(["a", "b", "c", "d"], step=0.1))
    head = tracker.update(_words(["a", "b", "c", "d"], step=0.1))
    assert head is None  # duration too short

    tracker = HeadStabilityTracker(window=2, min_words=1, min_duration=0.0)
    tracker.update(_words(["ok"], step=0.5))
    head2 = tracker.update(_words(["ok"], step=0.5))
    assert head2 is not None
    assert head2.text == "ok"
    assert head2.duration == 0.5


def test_tracker_reset_clears_history() -> None:
    tracker = HeadStabilityTracker(window=2, min_words=1, min_duration=0.0)
    tracker.update(_words(["stay"], step=0.4))
    tracker.update(_words(["stay"], step=0.4))
    assert tracker.update(_words(["stay"], step=0.4)) is not None

    tracker.reset()
    assert tracker.update(_words(["fresh"], step=0.4)) is None
