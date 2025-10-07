"""Utilities for normalising :mod:`sensor_msgs` image messages for the cockpit."""
from __future__ import annotations

import base64
import datetime as dt
import logging
from dataclasses import dataclass
from typing import Any, Dict, Optional, TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - imported for type checking only
    from sensor_msgs.msg import Image  # noqa: F401

try:  # pragma: no cover - optional dependency
    import numpy as np
except ImportError:  # pragma: no cover - fallback when numpy missing
    np = None  # type: ignore[assignment]

try:  # pragma: no cover - optional dependency
    import cv2
except ImportError:  # pragma: no cover - fallback when OpenCV missing
    cv2 = None  # type: ignore[assignment]

try:  # pragma: no cover - optional dependency
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - fallback when cv_bridge missing
    CvBridge = None  # type: ignore[assignment]


@dataclass(slots=True)
class ImageEncodingResult:
    """Container returned by :class:`ImageEncoder` with metadata and payload."""

    payload: Dict[str, Any]
    """JSON-serialisable payload destined for cockpit websocket clients."""
    topic: str
    """ROS topic name that produced the payload."""


class ImageEncodingError(RuntimeError):
    """Raised when a ROS image message cannot be converted into JSON payload."""


class ImageEncoder:
    """Convert ROS 2 image messages into cockpit friendly payloads.

    The encoder prefers OpenCV + :mod:`cv_bridge` for efficient compression. When
    these optional dependencies are unavailable the encoder falls back to
    shipping raw bytes so that callers can still provide telemetry, albeit with
    reduced visual fidelity.
    """

    def __init__(self, logger: logging.Logger) -> None:
        self._logger = logger
        self._bridge = CvBridge() if CvBridge is not None else None

    def encode(self, topic: str, message: "Image") -> Optional[ImageEncodingResult]:
        """Return a cockpit payload for ``message``.

        Parameters
        ----------
        topic:
            ROS topic the message was received on. Included in the result for
            convenience when callers fan-out to multiple consumers.
        message:
            The ROS 2 :class:`sensor_msgs.msg.Image` instance to transform.

        Returns
        -------
        ImageEncodingResult | None
            ``None`` is returned when the message could not be converted. A
            warning is logged in that case so operators can investigate missing
            dependencies or malformed frames.
        """

        payload: Dict[str, Any] = {
            "height": int(getattr(message, "height", 0)),
            "width": int(getattr(message, "width", 0)),
            "encoding": getattr(message, "encoding", ""),
            "step": int(getattr(message, "step", 0)),
            "frame_id": getattr(getattr(message, "header", None), "frame_id", None),
        }

        stamp = getattr(getattr(message, "header", None), "stamp", None)
        iso_stamp = _stamp_to_iso(stamp)
        if iso_stamp is not None:
            payload["timestamp"] = iso_stamp
        payload["received_at"] = dt.datetime.now(dt.timezone.utc).isoformat()

        raw_data = getattr(message, "data", b"") or b""
        payload["size_bytes"] = len(raw_data)

        encoding = (payload["encoding"] or "").lower()
        is_depth = _looks_like_depth(encoding)
        payload["is_depth"] = is_depth

        if self._bridge is None or cv2 is None or np is None:
            self._logger.warning(
                "cv_bridge/OpenCV not available; streaming raw bytes for topic %s",
                topic,
            )
            payload.update(
                {
                    "format": "raw",
                    "data": base64.b64encode(raw_data).decode("ascii"),
                },
            )
            return ImageEncodingResult(payload=payload, topic=topic)

        try:
            if is_depth:
                cv_image = self._bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            else:
                cv_image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover - passthrough to logging
            self._logger.warning("Failed to convert image message on %s: %s", topic, exc)
            payload.update(
                {
                    "format": "raw",
                    "data": base64.b64encode(raw_data).decode("ascii"),
                },
            )
            return ImageEncodingResult(payload=payload, topic=topic)

        if cv_image is None:
            self._logger.warning("cv_bridge returned None for topic %s", topic)
            payload.update(
                {
                    "format": "raw",
                    "data": base64.b64encode(raw_data).decode("ascii"),
                },
            )
            return ImageEncodingResult(payload=payload, topic=topic)

        if is_depth:
            preview, stats = _make_depth_preview(cv_image)
            payload["statistics"] = stats
            encoded = _encode_image_bytes(preview, ext=".png")
            payload["format"] = "png"
        else:
            encoded = _encode_image_bytes(cv_image, ext=".jpg")
            payload["format"] = "jpeg"

        if encoded is None:
            self._logger.warning("Failed to encode preview for topic %s", topic)
            return None

        payload["data"] = base64.b64encode(encoded).decode("ascii")
        return ImageEncodingResult(payload=payload, topic=topic)


def _encode_image_bytes(image: Any, ext: str) -> Optional[bytes]:
    """Encode ``image`` using OpenCV and return the raw bytes."""

    if cv2 is None:  # pragma: no cover - guard keeps mypy happy
        return None

    ok, buffer = cv2.imencode(ext, image)
    if not ok:
        return None
    return buffer.tobytes()


def _make_depth_preview(image: Any) -> tuple[Any, Dict[str, float]]:
    """Normalise a depth frame into an 8-bit preview with statistics."""

    if np is None:  # pragma: no cover - guard keeps mypy happy
        return image, {}

    array = image.astype("float32", copy=False)
    finite_mask = np.isfinite(array)
    if not finite_mask.any():
        return np.zeros_like(array, dtype="uint8"), {}

    finite_values = array[finite_mask]
    min_val = float(np.min(finite_values))
    max_val = float(np.max(finite_values))
    mean_val = float(np.mean(finite_values))
    stats = {"min": min_val, "max": max_val, "mean": mean_val}

    scale = max_val - min_val
    if scale <= 1e-6:
        normalised = np.zeros_like(array, dtype="uint8")
    else:
        normalised = np.clip((array - min_val) / scale, 0.0, 1.0)
        normalised = (normalised * 255).astype("uint8")

    return normalised, stats


def _stamp_to_iso(stamp: Any) -> Optional[str]:
    """Convert a ROS time stamp object into an ISO 8601 string."""

    if stamp is None:
        return None

    seconds = getattr(stamp, "sec", None)
    if seconds is None:
        seconds = getattr(stamp, "secs", None)
    if seconds is None:
        return None

    nanoseconds = (
        getattr(stamp, "nanosec", None)
        or getattr(stamp, "nanosecs", None)
        or getattr(stamp, "nsec", None)
        or getattr(stamp, "nsecs", None)
        or 0
    )

    try:
        total_seconds = float(seconds) + float(nanoseconds) / 1_000_000_000
    except (TypeError, ValueError):
        return None

    dt_obj = dt.datetime.fromtimestamp(total_seconds, tz=dt.timezone.utc)
    return dt_obj.isoformat()


def _looks_like_depth(encoding: str) -> bool:
    """Return ``True`` when ``encoding`` represents a depth image."""

    encoding_lower = encoding.lower()
    if any(tag in encoding_lower for tag in ("16uc1", "32fc1", "mono16", "16sc1")):
        return True
    return "depth" in encoding_lower


__all__ = [
    "ImageEncoder",
    "ImageEncodingError",
    "ImageEncodingResult",
]
