"""Tests for the face detection processing pipeline."""
from __future__ import annotations

import sys
import types
from typing import List

import numpy as np
import pytest


def _ensure_ros_stubs() -> None:
    """Provide lightweight stand-ins for ROS packages during unit tests."""

    if "sensor_msgs.msg" not in sys.modules:
        class _StubImage:  # pylint: disable=too-few-public-methods
            def __init__(self) -> None:
                self.height = 0
                self.width = 0
                self.encoding = ""
                self.data = b""
                self.step = 0
                self.header = types.SimpleNamespace(frame_id="", stamp=types.SimpleNamespace(sec=0, nanosec=0))

        sensor_msgs = types.SimpleNamespace(msg=types.SimpleNamespace(Image=_StubImage))
        sys.modules.setdefault("sensor_msgs", sensor_msgs)
        sys.modules.setdefault("sensor_msgs.msg", sensor_msgs.msg)

    if "std_msgs.msg" not in sys.modules:
        class _StubHeader:  # pylint: disable=too-few-public-methods
            def __init__(self) -> None:
                self.frame_id = ""
                self.stamp = types.SimpleNamespace(sec=0, nanosec=0)

        std_msgs = types.SimpleNamespace(msg=types.SimpleNamespace(Header=_StubHeader))
        sys.modules.setdefault("std_msgs", std_msgs)
        sys.modules.setdefault("std_msgs.msg", std_msgs.msg)

    if "cv_bridge" not in sys.modules:
        class _StubBridge:  # pylint: disable=too-few-public-methods
            """Minimal CvBridge compatible with the conversion helpers."""

            @staticmethod
            def cv2_to_imgmsg(array: np.ndarray, encoding: str = "bgr8") -> types.SimpleNamespace:
                image = sys.modules["sensor_msgs.msg"].Image()
                image.height, image.width = array.shape[:2]
                image.encoding = encoding
                image.step = array.shape[1] * (array.shape[2] if array.ndim == 3 else 1)
                image.data = array.tobytes()
                return image

        cv_bridge = types.SimpleNamespace(CvBridge=_StubBridge)
        sys.modules.setdefault("cv_bridge", cv_bridge)


def _ensure_faces_msgs_stub() -> None:
    """Create stub message classes for faces_msgs."""

    if "faces_msgs.msg" in sys.modules:
        return

    class _StubFaceDetection:  # pylint: disable=too-few-public-methods
        def __init__(self) -> None:
            self.header = None
            self.x = 0
            self.y = 0
            self.width = 0
            self.height = 0
            self.confidence = 0.0
            self.crop = None
            self.embedding: List[float] = []

    class _StubFaceDetections:  # pylint: disable=too-few-public-methods
        def __init__(self) -> None:
            self.header = None
            self.faces: List[_StubFaceDetection] = []

    msg_module = types.SimpleNamespace(
        FaceDetection=_StubFaceDetection,
        FaceDetections=_StubFaceDetections,
    )
    pkg_module = types.SimpleNamespace(msg=msg_module)
    sys.modules.setdefault("faces_msgs", pkg_module)
    sys.modules.setdefault("faces_msgs.msg", msg_module)


_ensure_ros_stubs()
_ensure_faces_msgs_stub()

from faces.processing import (  # noqa: E402  pylint: disable=wrong-import-position
    BasicEmbeddingExtractor,
    BoundingBox,
    Detection,
    FaceProcessor,
    ProcessedFace,
)
from faces.message_builder import (  # noqa: E402  pylint: disable=wrong-import-position
    build_face_detections_msg,
)


class StubDetector:
    """Simple detector stub returning a fixed bounding box for testing."""

    def detect(self, image: np.ndarray) -> List[Detection]:  # pylint: disable=unused-argument
        bbox = BoundingBox(x=5, y=10, width=12, height=8)
        return [Detection(bbox=bbox, confidence=0.85)]


@pytest.fixture()
def sample_image() -> np.ndarray:
    """Create a colorful test image with deterministic pixel values."""

    rng = np.random.default_rng(seed=42)
    # Height 32, width 32, 3 channels
    return (rng.random((32, 32, 3)) * 255).astype(np.uint8)


def test_processor_returns_processed_faces_with_embeddings(sample_image: np.ndarray) -> None:
    """Processor should crop detected faces and attach deterministic embeddings."""

    processor = FaceProcessor(detector=StubDetector(), embedder=BasicEmbeddingExtractor())

    faces = processor.process(sample_image)

    assert len(faces) == 1

    face = faces[0]
    assert isinstance(face, ProcessedFace)
    assert face.bbox.width == 12
    assert face.bbox.height == 8
    assert face.crop.shape == (8, 12, 3)
    assert face.embedding.shape == (BasicEmbeddingExtractor.EMBEDDING_SIZE,)
    assert np.isfinite(face.embedding).all()
    assert 0.0 <= face.embedding.min() <= face.embedding.max() <= 1.0


def test_face_detections_message_contains_crops_and_vectors(sample_image: np.ndarray) -> None:
    """The ROS message should include crops and embeddings for each face."""

    processor = FaceProcessor(detector=StubDetector(), embedder=BasicEmbeddingExtractor())
    faces = processor.process(sample_image)

    header = sys.modules["std_msgs.msg"].Header()
    header.frame_id = "camera"

    detections_msg = build_face_detections_msg(header, faces)

    assert detections_msg.header.frame_id == "camera"
    assert len(detections_msg.faces) == 1

    face_msg = detections_msg.faces[0]
    assert face_msg.width == faces[0].bbox.width
    assert face_msg.height == faces[0].bbox.height
    assert face_msg.crop.height == faces[0].bbox.height
    assert face_msg.crop.width == faces[0].bbox.width
    assert len(face_msg.embedding) == BasicEmbeddingExtractor.EMBEDDING_SIZE
    assert pytest.approx(face_msg.embedding[0]) == faces[0].embedding[0]
