"""Utilities for reading and simulating DHT-series thermal sensors."""

from __future__ import annotations

import itertools
import math
import random
import time
from dataclasses import dataclass, field
from typing import Iterable, Optional

__all__ = [
    "ThermalSample",
    "celsius_to_fahrenheit",
    "simulated_samples",
]


@dataclass(slots=True)
class ThermalSample:
    """Represent a single temperature and humidity reading.

    Parameters
    ----------
    temperature_celsius:
        Ambient temperature in degrees Celsius.
    humidity_percent:
        Relative humidity percentage in the [0, 100] interval.
    sequence:
        Monotonic sequence number used to order samples.
    source:
        Optional human-readable identifier for the sampling backend.
    captured_at:
        Epoch timestamp in seconds when the reading was taken.
    """

    temperature_celsius: float
    humidity_percent: float
    sequence: int
    source: Optional[str] = None
    captured_at: float = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.captured_at = time.time()

    @property
    def fahrenheit(self) -> float:
        """Return the temperature expressed in degrees Fahrenheit."""

        return celsius_to_fahrenheit(self.temperature_celsius)


def celsius_to_fahrenheit(celsius: float) -> float:
    """Convert a Celsius temperature into Fahrenheit.

    Examples
    --------
    >>> celsius_to_fahrenheit(25.0)
    77.0
    """

    return (celsius * 9.0 / 5.0) + 32.0


def simulated_samples(*, seed: Optional[int] = None) -> Iterable[ThermalSample]:
    """Yield a smooth stream of plausible thermal samples.

    Parameters
    ----------
    seed:
        Optional seed value to make the pseudo-random waveform deterministic.
    Yields
    ------
    ThermalSample
        Fresh thermal readings synthesised from a bounded waveform.
    """

    rng = random.Random(seed)
    baseline_temp = rng.uniform(21.0, 24.0)
    baseline_humidity = rng.uniform(40.0, 55.0)

    for sequence in itertools.count():
        temp_variation = math.sin(sequence / 4.5) * 3.5 + rng.uniform(-0.2, 0.2)
        humidity_variation = math.cos(sequence / 5.0) * 8.0 + rng.uniform(-1.5, 1.5)
        yield ThermalSample(
            temperature_celsius=baseline_temp + temp_variation,
            humidity_percent=max(0.0, min(100.0, baseline_humidity + humidity_variation)),
            sequence=sequence,
            source="simulated",
        )
