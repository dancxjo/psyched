"""Tests for IMU module topic translators."""

from modules.imu.pilot.topic_translator import summarise_imu_motion


def test_summarise_imu_motion_reports_orientation_and_motion() -> None:
    """IMU summaries should include Euler angles and motion vectors."""

    payload = {
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.3826834, "w": 0.9238795},
        "angular_velocity": {"x": 0.01, "y": 0.02, "z": 0.03},
        "linear_acceleration": {"x": 0.1, "y": 0.0, "z": 9.8},
    }
    assert (
        summarise_imu_motion(payload)
        == "IMU: roll=0.0° pitch=0.0° yaw=45.0° ang_vel=0.01,0.02,0.03 rad/s lin_acc=0.10,0.00,9.80 m/s²."
    )


def test_summarise_imu_motion_handles_missing_payload() -> None:
    """Gracefully handle payloads that have not arrived yet."""

    assert summarise_imu_motion({}) == "IMU telemetry awaiting first reading."
