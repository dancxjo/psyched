"""Tests for Nav module topic translators."""

from modules.nav.pilot.topic_translator import summarise_laserscan, summarise_vision_annotation


def test_summarise_vision_annotation_returns_text() -> None:
    """Vision annotations should surface their plain text description."""

    payload = {"data": "Obstacle 2m ahead"}
    assert summarise_vision_annotation(payload) == 'Vision annotation: "Obstacle 2m ahead".'


def test_summarise_laserscan_computes_range_statistics() -> None:
    """Laser scans should report min/median/max finite ranges."""

    payload = {
        "ranges": [0.4, 1.5, float("inf"), 2.0],
        "range_min": 0.2,
        "range_max": 3.5,
    }
    assert (
        summarise_laserscan(payload)
        == "Depth scan: 3 beams min=0.40m median=1.50m max=2.00m."
    )


def test_summarise_laserscan_handles_no_ranges() -> None:
    """Empty scans should return a calm placeholder."""

    assert summarise_laserscan({}) == "Depth scan awaiting data."
