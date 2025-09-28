"""Core face detection and embedding utilities."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Protocol

import cv2
import numpy as np


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
            cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"

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
