"""ROS 2 QoS helpers with graceful degradation for unit tests."""
from __future__ import annotations

from typing import Any

try:  # pragma: no cover - requires ROS 2 runtime
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, SensorDataQoS
except ImportError:  # pragma: no cover - executed in non-ROS test environments
    QoSDurabilityPolicy = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    QoSReliabilityPolicy = None  # type: ignore[assignment]

    def SensorDataQoS() -> int:  # type: ignore[misc]
        return 10


def sensor_data_qos() -> Any:
    """Return a QoS profile appropriate for high-rate sensor streams."""

    return SensorDataQoS()


def best_effort_qos(*, depth: int = 10) -> Any:
    """Return a best-effort QoS profile with volatile durability."""

    if (
        QoSProfile is not None
        and QoSHistoryPolicy is not None
        and QoSReliabilityPolicy is not None
        and QoSDurabilityPolicy is not None
    ):  # pragma: no cover - requires ROS 2 runtime
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
    return depth


__all__ = ["sensor_data_qos", "best_effort_qos"]

