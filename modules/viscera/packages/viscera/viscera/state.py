"""System-level state definitions consumed by viscera feelers."""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


def _clamp(value: Optional[float]) -> Optional[float]:
    """Return *value* constrained to ``[0.0, 1.0]`` when provided."""
    if value is None:
        return None
    return max(0.0, min(1.0, value))


@dataclass(frozen=True)
class BatteryState:
    """Snapshot of the robot's energy reserves.

    The charge fraction is always coerced into the ``[0, 1]`` range so feelers can
    reason about the value without defensive checks.

    Examples
    --------
    >>> state = BatteryState(charge_fraction=1.3, is_charging=True)
    >>> state.charge_fraction
    1.0
    >>> state.is_charging
    True
    """

    charge_fraction: Optional[float] = None
    is_charging: Optional[bool] = None
    temperature_c: Optional[float] = None
    health_fraction: Optional[float] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "charge_fraction", _clamp(self.charge_fraction))
        object.__setattr__(self, "health_fraction", _clamp(self.health_fraction))


@dataclass(frozen=True)
class FootState:
    """Summarises proprioceptive cues coming from Pete's foot sensors."""

    hazard_contacts: int = 0
    is_slipping: bool = False
    contact_confidence: Optional[float] = None
    temperature_c: Optional[float] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "contact_confidence", _clamp(self.contact_confidence))


@dataclass(frozen=True)
class SystemState:
    """Aggregated sensor view that feelers consume."""

    timestamp: datetime
    battery: BatteryState = field(default_factory=BatteryState)
    cpu_load: Optional[float] = None
    memory_load: Optional[float] = None
    disk_fill_level: Optional[float] = None
    foot: Optional[FootState] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "cpu_load", _clamp(self.cpu_load))
        object.__setattr__(self, "memory_load", _clamp(self.memory_load))
        object.__setattr__(self, "disk_fill_level", _clamp(self.disk_fill_level))


__all__ = ["BatteryState", "FootState", "SystemState"]
