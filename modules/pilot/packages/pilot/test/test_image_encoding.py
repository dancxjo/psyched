"""Unit tests for the cockpit image encoding helpers."""
from __future__ import annotations

import base64
import logging
from types import SimpleNamespace

import pytest

from pilot.cockpit.image import (
    ImageEncoder,
    _looks_like_depth,
    _make_depth_preview,
    _stamp_to_iso,
)


class _DummyStamp(SimpleNamespace):
    pass


class _DummyHeader(SimpleNamespace):
    pass


class _DummyImage(SimpleNamespace):
    pass


@pytest.fixture
def encoder() -> ImageEncoder:
    """Provide an :class:`ImageEncoder` instance with logging muted."""

    logger = logging.getLogger("pilot.test.image_encoder")
    logger.addHandler(logging.NullHandler())
    instance = ImageEncoder(logger=logger)
    # Force the slow-path so tests do not depend on optional libraries.
    instance._bridge = None  # type: ignore[attr-defined]
    return instance


def test_encode_falls_back_to_raw_bytes(encoder: ImageEncoder) -> None:
    """When no OpenCV bindings are available the encoder base64s the raw bytes."""

    payload = b"\x00\x01\x02"
    message = _DummyImage(
        height=480,
        width=640,
        step=1920,
        encoding="rgb8",
        data=payload,
        header=_DummyHeader(frame_id="camera", stamp=_DummyStamp(sec=1, nanosec=500_000_000)),
    )

    result = encoder.encode("/image_raw", message)
    assert result is not None
    assert result.payload["format"] == "raw"
    assert result.payload["frame_id"] == "camera"
    assert result.payload["size_bytes"] == len(payload)
    assert result.payload["timestamp"].startswith("1970-01-01T00:00:01")
    assert result.payload["data"] == base64.b64encode(payload).decode("ascii")


def test_stamp_to_iso_handles_missing_values() -> None:
    """Gracefully ignore malformed stamp objects."""

    assert _stamp_to_iso(None) is None
    assert _stamp_to_iso(object()) is None
    assert _stamp_to_iso(_DummyStamp(sec="oops")) is None


def test_looks_like_depth_detects_common_encodings() -> None:
    """Depth encodings should be detected case-insensitively."""

    assert _looks_like_depth("16UC1")
    assert _looks_like_depth("mono16")
    assert _looks_like_depth("something_depth")
    assert not _looks_like_depth("rgb8")


def test_make_depth_preview_normalises_with_numpy() -> None:
    """Depth preview helper should normalise arrays when numpy is installed."""

    np = pytest.importorskip("numpy")
    frame = np.array([[1.0, 2.0], [3.0, np.nan]], dtype=np.float32)
    preview, stats = _make_depth_preview(frame)
    assert preview.dtype == np.uint8
    assert preview.min() == 0
    assert preview.max() == 255
    assert stats["min"] == pytest.approx(1.0)
    assert stats["max"] == pytest.approx(3.0)
    assert stats["mean"] == pytest.approx(2.0)
