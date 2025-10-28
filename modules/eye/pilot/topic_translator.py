"""Natural language summaries for Eye camera feeds."""

from __future__ import annotations

from typing import Any, Mapping, Tuple


def _extract_dimensions(payload: Mapping[str, Any]) -> Tuple[int | None, int | None]:
    width = payload.get("width") or payload.get("image_width")
    height = payload.get("height") or payload.get("image_height")
    try:
        return int(width), int(height)
    except (TypeError, ValueError):
        return None, None


def _format_bytes(length: Any) -> str | None:
    try:
        value = int(length)
    except (TypeError, ValueError):
        return None
    if value <= 0:
        return None
    if value < 1024:
        return f"{value} bytes"
    kib = value / 1024.0
    if kib < 1024:
        return f"{kib:.1f} KiB"
    mib = kib / 1024.0
    return f"{mib:.2f} MiB"


def summarise_eye_image(payload: Any) -> str:
    """Describe the latest Eye frame published on ``/camera/color/image_raw``.

    Examples
    --------
    >>> summarise_eye_image({"width": 640, "height": 480, "format": "jpeg", "byte_length": 9216})
    'Eye camera frame 640x480 jpeg (9.0 KiB).'
    >>> summarise_eye_image({"width": 0})
    'Eye camera feed is standing by for the next frame.'
    """

    if not isinstance(payload, Mapping):
        return "Eye camera feed is standing by for the next frame."

    width, height = _extract_dimensions(payload)
    format_hint = str(payload.get("format") or payload.get("encoding") or "").strip()
    byte_hint = _format_bytes(payload.get("byte_length") or payload.get("data_length"))

    parts: list[str] = []
    if width and height and width > 0 and height > 0:
        parts.append(f"{width}x{height}")
    if format_hint:
        parts.append(format_hint)
    if byte_hint:
        parts.append(f"({byte_hint})")

    if not parts:
        return "Eye camera feed is standing by for the next frame."

    summary_body = " ".join(parts)
    return f"Eye camera frame {summary_body}."


TOPIC_TRANSLATORS = {
    "/camera/color/image_raw*": summarise_eye_image,
    "/kinect_ros2/image_raw*": summarise_eye_image,
    "/image_raw*": summarise_eye_image,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Eye streams Kinect RGB frames over /camera/color/image_raw*. Summaries "
        "highlight resolution, encoding, and payload size so the pilot knows how "
        "fresh imagery flows through the instant window."
    )
]


__all__ = ["TOPIC_TRANSLATORS", "STATIC_PROMPT_SECTIONS", "summarise_eye_image"]
