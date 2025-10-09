from types import SimpleNamespace

import pytest

try:
    from pilot.cockpit import bridge
    from rclpy.qos import qos_profile_sensor_data
except ModuleNotFoundError as exc:  # pragma: no cover - optional dependency in CI containers
    pytestmark = pytest.mark.skip(reason=f"rclpy is not available: {exc}")
    bridge = None  # type: ignore[assignment]
    qos_profile_sensor_data = None  # type: ignore[assignment]


def make_imu_message(**overrides):
    header = overrides.get(
        "header",
        SimpleNamespace(
            frame_id="imu_link",
            stamp=SimpleNamespace(sec=12, nanosec=340_000_000),
        ),
    )
    orientation = overrides.get(
        "orientation",
        SimpleNamespace(x=0.1, y=0.2, z=0.3, w=0.4),
    )
    angular_velocity = overrides.get(
        "angular_velocity",
        SimpleNamespace(x=0.01, y=-0.02, z=0.03),
    )
    linear_acceleration = overrides.get(
        "linear_acceleration",
        SimpleNamespace(x=0.5, y=0.6, z=0.7),
    )
    message = SimpleNamespace(
        header=header,
        orientation=orientation,
        angular_velocity=angular_velocity,
        linear_acceleration=linear_acceleration,
    )
    for key, value in overrides.items():
        setattr(message, key, value)
    return message


def test_imu_subscription_declares_best_effort_qos():
    relays = getattr(bridge, "_SIMPLE_TOPIC_RELAYS", None)
    assert relays is not None, "IMU relay registry should exist"
    imu_spec = relays.get("/imu/data")
    assert imu_spec is not None, "IMU topic should be declared in the relay registry"
    assert imu_spec.qos_profile is qos_profile_sensor_data


def test_imu_payload_serializer_matches_ros_structure():
    serializer = getattr(bridge, "_make_imu_payload", None)
    assert serializer is not None, "IMU payload serializer must be exposed for tests"
    message = make_imu_message()

    payload = serializer(message)

    assert payload["header"]["frame_id"] == "imu_link"
    assert payload["header"]["stamp"] == {"sec": 12, "nanosec": 340_000_000}
    assert payload["orientation"] == {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.4}
    assert payload["angular_velocity"] == {"x": 0.01, "y": -0.02, "z": 0.03}
    assert payload["linear_acceleration"] == {"x": 0.5, "y": 0.6, "z": 0.7}

    with pytest.raises(AttributeError):
        serializer(SimpleNamespace())
