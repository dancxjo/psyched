"""Core face detection and embedding utilities."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Protocol

import cv2
import os
import numpy as np


def _default_haarcascade_path() -> str:
    """Return a reasonable default path to OpenCV's haarcascade xml.

    Tries multiple strategies to locate the file so code works across
    different OpenCV packaging variations (some builds don't expose
    ``cv2.data``).
    """
    filename = "haarcascade_frontalface_default.xml"

    # 1) Preferred: cv2.data.haarcascades (most wheels expose this)
    try:
        haar_dir = getattr(cv2, "data", None)
        if haar_dir is not None:
            # cv2.data.haarcascades is typically a string path
            haar_path = getattr(cv2.data, "haarcascades", None)
            if isinstance(haar_path, str):
                candidate = os.path.join(haar_path, filename) if not haar_path.endswith(filename) else haar_path
            else:
                # cv2.data may itself be a path-like string in some builds
                if isinstance(haar_dir, str):
                    candidate = os.path.join(haar_dir, filename)
                else:
                    candidate = ""
            if candidate and os.path.exists(candidate):
                return candidate
    except Exception:
        # Be defensive: any unexpected shape of cv2.data should not crash
        pass

    # 2) Check next to the cv2 module (many installations place a data/ dir
    #    alongside the package file)
    try:
        cv2_dir = os.path.dirname(cv2.__file__)
        alt = os.path.join(cv2_dir, "data", filename)
        if os.path.exists(alt):
            return alt
    except Exception:
        pass

    # 3) Common system paths
    for base in ("/usr/share/opencv4/haarcascades", "/usr/share/opencv/haarcascades", "/usr/local/share/opencv4/haarcascades", "/usr/local/share/opencv/haarcascades"):
        candidate = os.path.join(base, filename)
        if os.path.exists(candidate):
            return candidate

    raise ValueError(
        "Could not locate 'haarcascade_frontalface_default.xml'. "
        "Install OpenCV data files (e.g. 'apt install opencv-data' or the equivalent), "
        "or pass an explicit 'cascade_path' to HaarCascadeDetector()."
    )


@dataclass(frozen=True)
class BoundingBox:
    """Axis-aligned bounding box using pixel coordinates."""

    x: int
    y: int
    width: int
    height: int


@dataclass(frozen=True)
class Detection:
    """Intermediate detection result returned by a detector backend."""

    bbox: BoundingBox
    confidence: float


@dataclass(frozen=True)
class ProcessedFace:
    """Container for cropped faces and their embeddings."""

    bbox: BoundingBox
    confidence: float
    crop: np.ndarray
    embedding: np.ndarray


class DetectorBackend(Protocol):
    """Protocol for detector backends used by :class:`FaceProcessor`."""

    def detect(self, image: np.ndarray) -> Iterable[Detection]:
        """Return detections for the provided image."""


class HaarCascadeDetector:
    """Detect faces using OpenCV's Haar cascades."""

    def __init__(
        self,
        cascade_path: str | None = None,
        *,
        scale_factor: float = 1.1,
        min_neighbors: int = 5,
    ) -> None:
        if cascade_path is None:
            cascade_path = _default_haarcascade_path()

        self._classifier = cv2.CascadeClassifier(cascade_path)
        if self._classifier.empty():  # pragma: no cover - defensive guard
            raise ValueError(f"Failed to load Haar cascade at {cascade_path}")

        self._scale_factor = float(scale_factor)
        self._min_neighbors = int(min_neighbors)

    def detect(self, image: np.ndarray) -> List[Detection]:
        """Return detected faces with approximate confidences."""

        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        faces: List[Detection] = []

        if hasattr(self._classifier, "detectMultiScale3"):
            rects, reject_levels, level_weights = self._classifier.detectMultiScale3(  # type: ignore[attr-defined]
                gray,
                scaleFactor=self._scale_factor,
                minNeighbors=self._min_neighbors,
                outputRejectLevels=True,
            )
            if rects is None or len(rects) == 0:
                return []
            for (x, y, w, h), weight in zip(rects, level_weights):
                bbox = BoundingBox(int(x), int(y), int(w), int(h))
                faces.append(Detection(bbox=bbox, confidence=float(weight)))
            return faces

        rects = self._classifier.detectMultiScale(
            gray,
            scaleFactor=self._scale_factor,
            minNeighbors=self._min_neighbors,
        )
        for (x, y, w, h) in rects:
            bbox = BoundingBox(int(x), int(y), int(w), int(h))
            faces.append(Detection(bbox=bbox, confidence=1.0))
        return faces


class BasicEmbeddingExtractor:
    """Generate deterministic embeddings by downsampling to a compact vector."""

    EMBEDDING_SIZE = 256

    def __init__(self, embedding_size: int | None = None) -> None:
        size = int(embedding_size or self.EMBEDDING_SIZE)
        side = int(math.sqrt(size))
        if side * side != size:
            raise ValueError("embedding_size must be a perfect square")
        self._size = size
        self._side = side

    def embed(self, crop: np.ndarray) -> np.ndarray:
        """Return a flattened, normalised embedding for the crop."""

        if crop.ndim == 3:
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        else:
            gray = crop

        resized = cv2.resize(gray, (self._side, self._side), interpolation=cv2.INTER_AREA)
        embedding = resized.astype(np.float32)
        if embedding.size == 0:  # pragma: no cover - defensive guard
            return np.zeros(self._size, dtype=np.float32)

        if embedding.max() == embedding.min():
            normalised = np.zeros_like(embedding, dtype=np.float32)
        else:
            normalised = (embedding - embedding.min()) / (embedding.max() - embedding.min())
        return normalised.flatten()

    @property
    def size(self) -> int:
        """Return the dimensionality of the embedding vector."""

        return self._size


class FaceProcessor:
    """High-level pipeline to crop faces and derive embeddings."""

    def __init__(self, *, detector: DetectorBackend, embedder: BasicEmbeddingExtractor) -> None:
        self._detector = detector
        self._embedder = embedder

    def process(self, image: np.ndarray) -> List[ProcessedFace]:
        """Return processed faces for the given image."""

        height, width = image.shape[:2]
        processed: List[ProcessedFace] = []
        for detection in self._detector.detect(image):
            bbox = detection.bbox
            x1 = max(0, min(width, bbox.x))
            y1 = max(0, min(height, bbox.y))
            x2 = max(0, min(width, bbox.x + bbox.width))
            y2 = max(0, min(height, bbox.y + bbox.height))
            if x2 <= x1 or y2 <= y1:
                continue

            crop = image[y1:y2, x1:x2].copy()
            embedding = self._embedder.embed(crop)
            processed.append(
                ProcessedFace(
                    bbox=BoundingBox(x=x1, y=y1, width=x2 - x1, height=y2 - y1),
                    confidence=float(detection.confidence),
                    crop=crop,
                    embedding=embedding,
                )
            )
        return processed
