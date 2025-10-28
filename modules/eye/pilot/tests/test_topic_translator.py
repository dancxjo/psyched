"""Tests for the Eye module topic translators."""

from modules.eye.pilot.topic_translator import summarise_eye_image


def test_eye_image_summary_includes_dimensions_and_size() -> None:
    """RGB frames should mention resolution, format, and payload size."""

    payload = {
        "width": 640,
        "height": 480,
        "format": "jpeg",
        "byte_length": 9216,
    }
    assert (
        summarise_eye_image(payload)
        == "Eye camera frame 640x480 jpeg (9.0 KiB)."
    )


def test_eye_image_summary_handles_missing_data() -> None:
    """Gracefully handle frames that have not reported metadata yet."""

    payload = {"width": 0, "height": 0}
    assert summarise_eye_image(payload) == "Eye camera feed is standing by for the next frame."


def test_eye_image_summary_with_raw_string() -> None:
    """Arbitrary payloads should fall back to a neutral message."""

    assert (
        summarise_eye_image("not a mapping")
        == "Eye camera feed is standing by for the next frame."
    )
