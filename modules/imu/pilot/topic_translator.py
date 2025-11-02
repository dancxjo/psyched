"""Natural language summaries for IMU motion telemetry."""

from __future__ import annotations

import math
from typing import Any, Mapping


def _coerce_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def _euler_from_quaternion(payload: Mapping[str, Any] | None) -> tuple[float | None, float | None, float | None]:
    if not isinstance(payload, Mapping):
        return None, None, None
    x = _coerce_float(payload.get("x")) or 0.0
    y = _coerce_float(payload.get("y")) or 0.0
    z = _coerce_float(payload.get("z")) or 0.0
    w = _coerce_float(payload.get("w")) or 1.0

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


def _format_vector(payload: Mapping[str, Any] | None, *, precision: int = 2) -> str | None:
    if not isinstance(payload, Mapping):
        return None
    x = _coerce_float(payload.get("x"))
    y = _coerce_float(payload.get("y"))
    z = _coerce_float(payload.get("z"))
    if x is None or y is None or z is None:
        return None
    return f"{x:.{precision}f},{y:.{precision}f},{z:.{precision}f}"


def summarise_imu_motion(payload: Any) -> str:
    """Summarize ``sensor_msgs/msg/Imu`` orientation and motion."""

    if not isinstance(payload, Mapping):
        return "IMU telemetry awaiting first reading."

    roll, pitch, yaw = _euler_from_quaternion(payload.get("orientation"))
    ang_vel = _format_vector(payload.get("angular_velocity"), precision=2)
    lin_acc = _format_vector(payload.get("linear_acceleration"), precision=2)

    if roll is None and pitch is None and yaw is None and ang_vel is None and lin_acc is None:
        return "IMU telemetry awaiting first reading."

    parts = ["IMU:"]
    if roll is not None and pitch is not None and yaw is not None:
        parts.append(
            f"roll={math.degrees(roll):.1f}° pitch={math.degrees(pitch):.1f}° yaw={math.degrees(yaw):.1f}°"
        )
    if ang_vel is not None:
        parts.append(f"ang_vel={ang_vel} rad/s")
    if lin_acc is not None:
        parts.append(f"lin_acc={lin_acc} m/s²")
    return " ".join(parts) + "."


TOPIC_TRANSLATORS = {
    "/imu/data": summarise_imu_motion,
    "/imu/data_raw": summarise_imu_motion,
}

STATIC_PROMPT_SECTIONS = [
    (
        "IMU streams orientation and motion on /imu/data. Summaries convert the "
        "quaternion into roll/pitch/yaw degrees and surface angular velocity and "
        "linear acceleration so the pilot senses Pete's posture."
    )
]


__all__ = ["TOPIC_TRANSLATORS", "STATIC_PROMPT_SECTIONS", "summarise_imu_motion"]
