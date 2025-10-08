"""Pilot status bookkeeping shared between cockpit components."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable, Dict, Optional


@dataclass(slots=True)
class PilotStatusSnapshot:
    """Immutable view of the cockpit bridge status."""

    note: str
    timestamp: str
    active_modules: int

    def to_payload(self) -> Dict[str, object]:
        """Serialise the snapshot for websocket consumers."""

        payload: Dict[str, object] = {
            "note": self.note,
            "timestamp": self.timestamp,
            "activeModules": self.active_modules,
        }
        return payload


class PilotStatusTracker:
    """Tracks operator-facing status for the cockpit websocket bridge."""

    def __init__(self, clock: Optional[Callable[[], datetime]] = None) -> None:
        self._clock = clock or (lambda: datetime.now(timezone.utc))
        self._note = "Cockpit bridge initialised"
        self._timestamp = self._now_iso()
        self._module_counts: Dict[str, int] = {}

    def snapshot(self) -> PilotStatusSnapshot:
        """Return the current status as a :class:`PilotStatusSnapshot`."""

        return PilotStatusSnapshot(
            note=self._note,
            timestamp=self._timestamp,
            active_modules=self.active_module_count,
        )

    @property
    def active_module_count(self) -> int:
        """Number of modules with at least one active subscriber."""

        return sum(1 for count in self._module_counts.values() if count > 0)

    def update(self, *, note: Optional[str] = None, timestamp: Optional[str] = None) -> bool:
        """Update the status note and timestamp.

        Returns ``True`` when the snapshot changed.
        """

        changed = False
        if note is not None:
            if not isinstance(note, str):  # pragma: no cover - guarded upstream
                raise TypeError("note must be a string")
            if note != self._note:
                self._note = note
                changed = True

        if timestamp is not None:
            if not isinstance(timestamp, str):  # pragma: no cover - guarded upstream
                raise TypeError("timestamp must be a string")
            new_timestamp = timestamp
        else:
            new_timestamp = self._now_iso()

        if new_timestamp != self._timestamp:
            self._timestamp = new_timestamp
            changed = True

        return changed

    def register_module(self, name: Optional[str]) -> bool:
        """Record that a module has at least one interested subscriber."""

        if not name:
            return False
        previous = self.active_module_count
        self._module_counts[name] = self._module_counts.get(name, 0) + 1
        if self.active_module_count != previous:
            self._timestamp = self._now_iso()
            return True
        return False

    def unregister_module(self, name: Optional[str]) -> bool:
        """Record that a module lost a subscriber."""

        if not name or name not in self._module_counts:
            return False

        previous = self.active_module_count
        count = self._module_counts[name] - 1
        if count <= 0:
            del self._module_counts[name]
        else:
            self._module_counts[name] = count

        if self.active_module_count != previous:
            self._timestamp = self._now_iso()
            return True
        return False

    def _now_iso(self) -> str:
        return self._clock().replace(microsecond=0).isoformat()

