"""Face detection utilities for the Psyched faces module."""
from __future__ import annotations

from importlib import import_module
from typing import Any, TYPE_CHECKING

from .message_builder import build_face_detections_msg  # noqa: F401

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .processing import (  # type: ignore[attr-defined]
        BasicEmbeddingExtractor,
        BoundingBox,
        Detection,
        FaceProcessor,
        HaarCascadeDetector,
        ProcessedFace,
    )

__all__ = [
    "BasicEmbeddingExtractor",
    "BoundingBox",
    "Detection",
    "FaceProcessor",
    "HaarCascadeDetector",
    "ProcessedFace",
    "build_face_detections_msg",
]

_PROCESSING_EXPORTS = {
    "BasicEmbeddingExtractor",
    "BoundingBox",
    "Detection",
    "FaceProcessor",
    "HaarCascadeDetector",
    "ProcessedFace",
}

_PROCESSING_MODULE: Any | None = None
_PROCESSING_IMPORT_ERROR: ModuleNotFoundError | None = None


def _missing_dependency_message(missing: str) -> str:
    human_readable = "OpenCV (cv2)" if missing == "cv2" else missing
    return (
        "faces module is missing the '{}' dependency. "
        "Run `psh mod setup faces` or install the required packages.".format(human_readable)
    )


def _load_processing_module() -> Any:
    """Lazily import ``faces.processing`` and surface helpful errors."""

    global _PROCESSING_MODULE, _PROCESSING_IMPORT_ERROR

    if _PROCESSING_MODULE is not None:
        return _PROCESSING_MODULE
    if _PROCESSING_IMPORT_ERROR is not None:
        missing = getattr(_PROCESSING_IMPORT_ERROR, "name", "dependency")
        raise ModuleNotFoundError(_missing_dependency_message(missing)) from _PROCESSING_IMPORT_ERROR
    try:
        module = import_module(".processing", __name__)
    except ModuleNotFoundError as exc:
        _PROCESSING_IMPORT_ERROR = exc
        missing = getattr(exc, "name", "dependency")
        raise ModuleNotFoundError(_missing_dependency_message(missing)) from exc
    _PROCESSING_MODULE = module
    return module


def __getattr__(name: str) -> Any:
    if name in _PROCESSING_EXPORTS:
        module = _load_processing_module()
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
