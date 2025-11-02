"""Prompt translators for Foot drive-base telemetry."""

from __future__ import annotations

import math
from typing import Any, Mapping, MutableMapping


def _coerce_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def summarise_charge_ratio(payload: Any) -> str:
    """Summarize the drive battery charge ratio as a percentage."""

    value = None
    if isinstance(payload, Mapping):
        value = payload.get("data") if "data" in payload else payload.get("value")
    else:
        value = payload
    ratio = _coerce_float(value)
    if ratio is None:
        return "Drive battery status pending."
    percent = max(0.0, min(100.0, ratio * 100.0))
    return f"Drive battery at {percent:.0f}%."


def _extract_pose(payload: Mapping[str, Any]) -> tuple[float | None, float | None, Mapping[str, Any] | None]:
    pose = payload.get("pose")
    if isinstance(pose, Mapping):
        pose = pose.get("pose")
    if not isinstance(pose, Mapping):
        return None, None, None
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, Mapping):
        return None, None, orientation if isinstance(orientation, Mapping) else None
    x = _coerce_float(position.get("x"))
    y = _coerce_float(position.get("y"))
    return x, y, orientation if isinstance(orientation, Mapping) else None


def _extract_twist(payload: Mapping[str, Any]) -> tuple[float | None, float | None]:
    twist = payload.get("twist")
    if isinstance(twist, Mapping):
        twist = twist.get("twist")
    if not isinstance(twist, Mapping):
        return None, None
    linear = twist.get("linear")
    angular = twist.get("angular")
    linear_x = _coerce_float(linear.get("x")) if isinstance(linear, Mapping) else None
    angular_z = _coerce_float(angular.get("z")) if isinstance(angular, Mapping) else None
    return linear_x, angular_z


def _yaw_from_quaternion(orientation: Mapping[str, Any] | None) -> float | None:
    if not isinstance(orientation, Mapping):
        return None
    x = _coerce_float(orientation.get("x")) or 0.0
    y = _coerce_float(orientation.get("y")) or 0.0
    z = _coerce_float(orientation.get("z")) or 0.0
    w = _coerce_float(orientation.get("w")) or 1.0
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def summarise_odometry(payload: Any) -> str:
    """Summarize planar pose and twist from ``nav_msgs/msg/Odometry``."""

    if not isinstance(payload, Mapping):
        return "Drive odom awaiting first reading."

    x, y, orientation = _extract_pose(payload)
    linear_x, angular_z = _extract_twist(payload)
    heading = _yaw_from_quaternion(orientation)

    if x is None and y is None and linear_x is None and angular_z is None:
        return "Drive odom awaiting first reading."

    parts: MutableMapping[str, str] = {}
    if x is not None:
        parts["x"] = f"x={x:.2f}m"
    if y is not None:
        parts["y"] = f"y={y:.2f}m"
    if heading is not None:
        parts["heading"] = f"heading={heading:.2f}rad"
    if linear_x is not None:
        parts["linear"] = f"linear={linear_x:.2f}m/s"
    if angular_z is not None:
        parts["angular"] = f"angular={angular_z:.2f}rad/s"

    if not parts:
        return "Drive odom awaiting first reading."

    summary = " ".join(parts[key] for key in ("x", "y", "heading", "linear", "angular") if key in parts)
    return f"Drive odom: {summary}."


TOPIC_TRANSLATORS = {
    "/create_driver/battery/charge_ratio": summarise_charge_ratio,
    "battery/charge_ratio": summarise_charge_ratio,
    "/create_driver/odom": summarise_odometry,
    "odom": summarise_odometry,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Foot surfaces drive-base vitals via /create_driver/battery/charge_ratio "
        "and /create_driver/odom. These summaries highlight remaining charge, "
        "pose, and motion so the pilot can plan navigation decisions."
    )
]


__all__ = [
    "TOPIC_TRANSLATORS",
    "STATIC_PROMPT_SECTIONS",
    "summarise_charge_ratio",
    "summarise_odometry",
]
