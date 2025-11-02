"""Prompt translators for the Hypothalamus thermal telemetry."""

from __future__ import annotations

from typing import Any, Mapping


def _coerce_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number


def summarise_temperature(payload: Any) -> str:
    """Summarize the current Celsius temperature."""

    if not isinstance(payload, Mapping):
        return "Temperature reading pending."
    temp = _coerce_float(payload.get("temperature"))
    if temp is None:
        return "Temperature reading pending."
    frame = None
    header = payload.get("header")
    if isinstance(header, Mapping):
        frame = header.get("frame_id")
    suffix = f" ({frame})" if isinstance(frame, str) and frame else ""
    return f"Temperature {temp:.1f}°C{suffix}."


def summarise_humidity_percent(payload: Any) -> str:
    """Summarize the relative humidity percent."""

    value = None
    if isinstance(payload, Mapping):
        value = payload.get("data") if "data" in payload else payload.get("relative_humidity")
        if isinstance(value, (int, float)) and 0.0 <= float(value) <= 1.0:
            value = float(value) * 100.0
    else:
        value = payload
    humidity = _coerce_float(value)
    if humidity is None:
        return "Humidity reading pending."
    humidity = max(0.0, min(100.0, humidity))
    return f"Humidity {humidity:.0f}%."


def summarise_status(payload: Any) -> str:
    """Summarize the latest thermal backend status string."""

    if isinstance(payload, Mapping):
        raw = payload.get("data") or payload.get("status")
    else:
        raw = payload
    if isinstance(raw, str):
        text = raw.strip()
        if text:
            return f"Thermal status: {text}."
    return "Thermal backend nominal."


TOPIC_TRANSLATORS = {
    "/environment/temperature": summarise_temperature,
    "/environment/humidity_percent": summarise_humidity_percent,
    "/environment/thermostat_status": summarise_status,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Hypothalamus publishes ambient conditions on /environment/temperature "
        "(°C), /environment/humidity_percent, and /environment/thermostat_status. "
        "Summaries keep the pilot aware of comfort levels and sensor health."
    )
]


__all__ = [
    "TOPIC_TRANSLATORS",
    "STATIC_PROMPT_SECTIONS",
    "summarise_temperature",
    "summarise_humidity_percent",
    "summarise_status",
]
