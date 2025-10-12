"""Hypothalamus ROS 2 integration for thermal sensing."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from .sensing import ThermalSample, celsius_to_fahrenheit, simulated_samples

__all__ = [
    "ThermoregulationNode",
    "ThermalSample",
    "celsius_to_fahrenheit",
    "simulated_samples",
    "main",
]


if TYPE_CHECKING:  # pragma: no cover - type checkers only
    from .node import ThermoregulationNode, main


def __getattr__(name: str) -> Any:
    if name in {"ThermoregulationNode", "main"}:
        from .node import ThermoregulationNode as _ThermoregulationNode, main as _main

        if name == "ThermoregulationNode":
            return _ThermoregulationNode
        return _main
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
