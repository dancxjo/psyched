from datetime import datetime, timezone

from pilot.cockpit.status import PilotStatusTracker


def make_clock(*values: datetime):
    sequence = iter(values)

    def _clock() -> datetime:
        try:
            return next(sequence)
        except StopIteration:  # pragma: no cover - defensive guard
            return values[-1]

    return _clock


def test_initial_snapshot_uses_clock_value() -> None:
    clock = make_clock(datetime(2025, 1, 1, tzinfo=timezone.utc))
    tracker = PilotStatusTracker(clock=clock)
    snapshot = tracker.snapshot()

    assert snapshot.note == "Cockpit bridge initialised"
    assert snapshot.timestamp == "2025-01-01T00:00:00+00:00"
    assert snapshot.active_modules == 0


def test_update_note_and_timestamp_from_clock() -> None:
    clock = make_clock(
        datetime(2025, 1, 1, 0, 0, 0, tzinfo=timezone.utc),
        datetime(2025, 1, 1, 0, 0, 5, tzinfo=timezone.utc),
    )
    tracker = PilotStatusTracker(clock=clock)

    changed = tracker.update(note="Operator ping")
    snapshot = tracker.snapshot()

    assert changed is True
    assert snapshot.note == "Operator ping"
    assert snapshot.timestamp == "2025-01-01T00:00:05+00:00"
    assert snapshot.active_modules == 0


def test_module_registration_tracks_active_count() -> None:
    clock = make_clock(
        datetime(2025, 1, 1, 0, 0, 0, tzinfo=timezone.utc),
        datetime(2025, 1, 1, 0, 0, 1, tzinfo=timezone.utc),
        datetime(2025, 1, 1, 0, 0, 2, tzinfo=timezone.utc),
        datetime(2025, 1, 1, 0, 0, 3, tzinfo=timezone.utc),
        datetime(2025, 1, 1, 0, 0, 4, tzinfo=timezone.utc),
    )
    tracker = PilotStatusTracker(clock=clock)

    assert tracker.snapshot().active_modules == 0

    assert tracker.register_module("foot") is True
    assert tracker.snapshot().active_modules == 1

    assert tracker.register_module("foot") is False
    assert tracker.snapshot().active_modules == 1

    assert tracker.register_module("eye") is True
    assert tracker.snapshot().active_modules == 2

    assert tracker.unregister_module("foot") is False
    assert tracker.snapshot().active_modules == 2

    assert tracker.unregister_module("foot") is True
    assert tracker.snapshot().active_modules == 1

    assert tracker.unregister_module("eye") is True
    assert tracker.snapshot().active_modules == 0
