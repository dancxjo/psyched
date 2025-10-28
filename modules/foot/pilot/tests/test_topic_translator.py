"""Tests for Foot module topic translators."""

from modules.foot.pilot.topic_translator import summarise_charge_ratio, summarise_odometry


def test_summarise_charge_ratio_formats_percentage() -> None:
    """Battery charge ratio should render as a human-friendly percent."""

    payload = {"data": 0.58}
    assert summarise_charge_ratio(payload) == "Drive battery at 58%."


def test_summarise_charge_ratio_handles_invalid_data() -> None:
    """Invalid payloads should fall back to a neutral message."""

    assert summarise_charge_ratio({"data": ""}) == "Drive battery status pending."


def test_summarise_odometry_reports_pose_and_velocity() -> None:
    """Odometry summaries should include planar pose and forward speed."""

    payload = {
        "pose": {
            "pose": {
                "position": {"x": 1.25, "y": -0.4, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0998, "w": 0.9950},
            }
        },
        "twist": {
            "twist": {
                "linear": {"x": 0.12, "y": 0.0, "z": 0.0},
                "angular": {"z": 0.05},
            }
        },
    }
    assert (
        summarise_odometry(payload)
        == "Drive odom: x=1.25m y=-0.40m heading=0.20rad linear=0.12m/s angular=0.05rad/s."
    )


def test_summarise_odometry_handles_missing_data() -> None:
    """When odometry has not published yet a calm message is returned."""

    assert summarise_odometry({}) == "Drive odom awaiting first reading."
