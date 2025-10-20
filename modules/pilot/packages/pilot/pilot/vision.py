from __future__ import annotations

import base64
from collections.abc import Mapping, Sequence
from typing import Any, Dict, Tuple

from .prompt_builder import PromptImage


def _coerce_bytes(value: Any) -> Tuple[bytes | None, int]:
    """Return ``(bytes_value, length)`` for arbitrary ROS ``data`` payloads."""

    if isinstance(value, (bytes, bytearray)):
        raw = bytes(value)
        return raw, len(raw)
    if isinstance(value, Sequence):
        try:
            raw = bytes(value)
        except (TypeError, ValueError):
            return None, 0
        return raw, len(raw)
    return None, 0


def summarise_image_message(
    topic: str,
    msg: Any,
    metadata: Mapping[str, Any] | None,
) -> tuple[Any, PromptImage | None]:
    """Sanitise *metadata* and optionally produce a :class:`PromptImage`.

    The helper removes bulky binary blobs from ``metadata`` so prompts stay
    concise while still advertising the byte size. When a compressed image is
    detected the raw bytes are base64-encoded for the Ollama images API.

    Example
    -------
    >>> meta = {"format": "jpeg", "data": [0, 1, 2]}
    >>> class _Msg:
    ...     format = "jpeg"
    ...     data = [0, 1, 2]
    >>> sanitised, image = summarise_image_message("/camera", _Msg(), meta)
    >>> sanitised["data"]
    '<omitted binary: 3 bytes>'
    >>> image.prompt_hint()
    '/camera (jpeg, 3 bytes)'
    """

    if not isinstance(metadata, Mapping):
        return metadata, None

    sanitised: Dict[str, Any] = dict(metadata)

    is_image_like = bool(
        hasattr(msg, "format")
        or hasattr(msg, "encoding")
        or "format" in sanitised
        or "encoding" in sanitised
    )
    if not is_image_like:
        return sanitised, None

    raw_bytes, byte_length = _coerce_bytes(getattr(msg, "data", sanitised.get("data")))
    if byte_length == 0:
        sanitised["data"] = "<omitted binary: 0 bytes>"
        return sanitised, None

    sanitised["data"] = f"<omitted binary: {byte_length} bytes>"
    sanitised["byte_length"] = byte_length

    format_hint = str(getattr(msg, "format", sanitised.get("format", "")) or "").strip()
    description_parts = []
    width = sanitised.get("width") or sanitised.get("image_width")
    height = sanitised.get("height") or sanitised.get("image_height")
    if isinstance(width, int) and isinstance(height, int) and width > 0 and height > 0:
        description_parts.append(f"{width}x{height}")
    if format_hint:
        description_parts.append(format_hint)
    description_parts.append(f"{byte_length} bytes")
    description = ", ".join(description_parts)

    if raw_bytes is None:
        return sanitised, None

    encoded = base64.b64encode(raw_bytes).decode("ascii")
    prompt_image = PromptImage(
        topic=topic,
        description=description,
        base64_data=encoded,
    )
    return sanitised, prompt_image
