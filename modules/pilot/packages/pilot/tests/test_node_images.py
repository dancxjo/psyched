"""Tests covering pilot node vision image helpers."""

from __future__ import annotations

from pilot.node import _latest_image_base64, _sort_images_by_recency
from pilot.prompt_builder import PromptImage


def _image(topic: str, *, captured_at: float | None, data: str) -> PromptImage:
    """Create a :class:`PromptImage` with consistent defaults for tests."""

    return PromptImage(
        topic=topic,
        description=f"frame from {topic}",
        base64_data=data,
        captured_at=captured_at,
    )


def test_sort_images_by_recency_descending() -> None:
    """Images should be ordered from newest to oldest based on timestamps."""

    images = [
        _image("/old", captured_at=5.0, data="old"),
        _image("/new", captured_at=12.5, data="new"),
        _image("/unknown", captured_at=None, data="mystery"),
    ]

    ordered = _sort_images_by_recency(images)

    assert [img.topic for img in ordered] == ["/new", "/old", "/unknown"]


def test_latest_image_base64_returns_newest_payload() -> None:
    """Only the freshest non-empty image payload should be forwarded to the LLM."""

    images = [
        _image("/stale", captured_at=1.0, data="stale"),
        _image("/fresh", captured_at=3.0, data="fresh"),
    ]

    assert _latest_image_base64(images) == "fresh"


def test_latest_image_base64_ignores_missing_payloads() -> None:
    """Empty payloads should be skipped when selecting the latest image."""

    images = [
        _image("/fresh", captured_at=7.0, data=""),
        _image("/older", captured_at=5.0, data="older"),
        _image("/no-timestamp", captured_at=None, data=""),
    ]

    assert _latest_image_base64(images) == "older"
