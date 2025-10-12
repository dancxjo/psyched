"""Behaviour-driven tests for the hypothalamus sensing helpers."""

from __future__ import annotations

from typing import Iterable

import math

import pytest

from hypothalamus.sensing import ThermalSample, celsius_to_fahrenheit, simulated_samples


@pytest.mark.parametrize(
    "celsius, expected",
    [
        pytest.param(0.0, 32.0, id="freezing point"),
        pytest.param(25.0, 77.0, id="room temperature"),
        pytest.param(-10.0, 14.0, id="subzero"),
    ],
)
def test_celsius_to_fahrenheit_converts_linearly(celsius: float, expected: float) -> None:
    """Given a Celsius reading, when we convert it, then the Fahrenheit value matches physics."""

    assert math.isclose(celsius_to_fahrenheit(celsius), expected, rel_tol=1e-6)


def test_simulated_samples_cycle_within_expected_limits() -> None:
    """Given the simulator, when we ask for several readings, then the envelope stays realistic."""

    readings: Iterable[ThermalSample] = simulated_samples(seed=42)

    for index, sample in zip(range(20), readings):
        assert 15.0 <= sample.temperature_celsius <= 35.0
        assert 20.0 <= sample.humidity_percent <= 90.0
        assert sample.fahrenheit == pytest.approx(celsius_to_fahrenheit(sample.temperature_celsius))
        assert sample.sequence == index
