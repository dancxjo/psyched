"""Unit tests for cockpit ROS QoS helpers."""

from __future__ import annotations

import pytest

pytest.importorskip("rclpy")

from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy

from cockpit.ros import RosClient


def _ros_client() -> RosClient:
    # Avoid running the full RosClient initialiser (which would spin up rclpy)
    # by constructing an instance without invoking __init__.
    return object.__new__(RosClient)  # type: ignore[call-arg]


def test_default_qos_prefers_best_effort() -> None:
    profile = RosClient._build_qos_profile(_ros_client(), queue_length=5, overrides=None)
    assert profile.depth == 5
    assert profile.reliability == QoSReliabilityPolicy.BEST_EFFORT
    assert profile.durability == QoSDurabilityPolicy.VOLATILE


def test_qos_override_restores_reliable() -> None:
    overrides = {"reliability": "reliable", "durability": "transient_local"}
    profile = RosClient._build_qos_profile(_ros_client(), queue_length=1, overrides=overrides)
    assert profile.reliability == QoSReliabilityPolicy.RELIABLE
    assert profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
