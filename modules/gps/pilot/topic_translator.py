"""Natural language summaries for GPS fixes."""

from __future__ import annotations

from typing import Any, Mapping


def _coerce_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number


def _format_latitude(value: float) -> str:
    hemisphere = "N" if value >= 0 else "S"
    return f"{abs(value):.4f}°{hemisphere}"


def _format_longitude(value: float) -> str:
    hemisphere = "E" if value >= 0 else "W"
    return f"{abs(value):.4f}°{hemisphere}"


def _describe_status(payload: Mapping[str, Any]) -> str:
    status = payload.get("status")
    code = None
    if isinstance(status, Mapping):
        code = status.get("status")
    else:
        code = status
    if isinstance(code, str):
        try:
            code = int(code)
        except ValueError:
            code = None
    if code == 0:
        return "fix"
    if code == 1:
        return "sb fix"
    return "no fix"


def summarise_nav_fix(payload: Any) -> str:
    """Summarise the ``sensor_msgs/msg/NavSatFix`` telemetry."""

    if not isinstance(payload, Mapping):
        return "GPS fix: awaiting satellite lock."

    lat = _coerce_float(payload.get("latitude"))
    lon = _coerce_float(payload.get("longitude"))
    alt = _coerce_float(payload.get("altitude"))
    if lat is None or lon is None:
        return "GPS fix: awaiting satellite lock."

    parts = [f"GPS fix: {_format_latitude(lat)} {_format_longitude(lon)}"]
    if alt is not None:
        parts.append(f"alt={alt:.1f}m")
    parts.append(f"({_describe_status(payload)}).")
    return " ".join(parts)


TOPIC_TRANSLATORS = {
    "/ublox_gps_node/fix": summarise_nav_fix,
    "/gps/fix": summarise_nav_fix,
    "fix": summarise_nav_fix,
}

STATIC_PROMPT_SECTIONS = [
    (
        "GPS publishes NavSatFix telemetry over /ublox_gps_node/fix. Summaries "
        "report latitude, longitude, altitude, and lock status so the pilot can "
        "orient Pete outdoors."
    )
]


__all__ = ["TOPIC_TRANSLATORS", "STATIC_PROMPT_SECTIONS", "summarise_nav_fix"]
