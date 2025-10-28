"""Tests for GPS module topic translators."""

from modules.gps.pilot.topic_translator import summarise_nav_fix


def test_summarise_nav_fix_formats_coordinates() -> None:
    """Latitude, longitude, altitude, and fix state should be included."""

    payload = {
        "latitude": 49.2827,
        "longitude": -123.1207,
        "altitude": 70.0,
        "status": {"status": 0},
    }
    assert (
        summarise_nav_fix(payload)
        == "GPS fix: 49.2827°N 123.1207°W alt=70.0m (fix)."
    )


def test_summarise_nav_fix_handles_missing_fix() -> None:
    """When data is incomplete the translator should fall back."""

    assert summarise_nav_fix({}) == "GPS fix: awaiting satellite lock."
