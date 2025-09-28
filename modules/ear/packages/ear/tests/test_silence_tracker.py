"""Behavioral tests for :class:`~ear.silence_tracker.SilenceTracker`."""
from pathlib import Path
import sys

package_root = str(Path(__file__).resolve().parents[1])
if package_root not in sys.path:
    sys.path.insert(0, package_root)

from ear.silence_tracker import SilenceTracker


def test_tracker_resets_to_zero_when_sound_detected():
    tracker = SilenceTracker(silence_threshold=0.5)

    # Begin in silence and accumulate time.
    assert tracker.update(rms=0.1, timestamp=0.0) == 0
    assert tracker.update(rms=0.1, timestamp=0.5) == 500

    # A burst of sound should reset the counter immediately.
    assert tracker.update(rms=1.0, timestamp=0.6) == 0

    # Once silence returns the first tick should still be zero and then grow.
    assert tracker.update(rms=0.1, timestamp=1.0) == 0
    assert tracker.update(rms=0.1, timestamp=1.4) == 400


def test_autophony_forces_reset_even_if_below_threshold():
    tracker = SilenceTracker(silence_threshold=0.5)

    tracker.update(rms=0.1, timestamp=0.0)
    tracker.update(rms=0.1, timestamp=0.5)

    # Autophony > 0 indicates self-generated audio, so the gauge should reset.
    assert tracker.update(rms=0.1, autophony_ms=10, timestamp=0.6) == 0
    assert tracker.update(rms=0.1, timestamp=1.0) == 0
    assert tracker.update(rms=0.1, timestamp=1.2) == 200
