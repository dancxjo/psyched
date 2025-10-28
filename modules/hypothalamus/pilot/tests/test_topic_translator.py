"""Tests for Hypothalamus module topic translators."""

from modules.hypothalamus.pilot.topic_translator import (
    summarise_humidity_percent,
    summarise_status,
    summarise_temperature,
)


def test_summarise_temperature_includes_frame() -> None:
    """Temperature summaries should include Celsius and frame hints."""

    payload = {"temperature": 21.6, "header": {"frame_id": "environment_link"}}
    assert (
        summarise_temperature(payload)
        == "Temperature 21.6°C (environment_link)."
    )


def test_summarise_humidity_formats_percentage() -> None:
    """Humidity percentages should be clamped and rendered clearly."""

    payload = {"data": 58.3}
    assert summarise_humidity_percent(payload) == "Humidity 58%."


def test_summarise_status_defaults_when_empty() -> None:
    """Status strings should fall back gracefully when missing."""

    assert summarise_status({"data": ""}) == "Thermal backend nominal."
