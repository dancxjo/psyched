"""Helpers for constructing ROS messages from processed faces."""
from __future__ import annotations

from typing import Iterable, Optional, TYPE_CHECKING

import numpy as np
from cv_bridge import CvBridge
from faces_msgs.msg import FaceDetection, FaceDetections
from std_msgs.msg import Header

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .processing import ProcessedFace


def build_face_detections_msg(
    header: Header,
    faces: Iterable["ProcessedFace"],
    *,
    bridge: Optional[CvBridge] = None,
) -> FaceDetections:
    """Convert processed faces into a ROS message payload."""

    bridge = bridge or CvBridge()

    detections_msg = FaceDetections()
    detections_msg.header = header
    detections_msg.faces = []

    for face in faces:
        detection_msg = FaceDetection()
        detection_msg.header = header
        detection_msg.x = int(face.bbox.x)
        detection_msg.y = int(face.bbox.y)
        detection_msg.width = int(face.bbox.width)
        detection_msg.height = int(face.bbox.height)
        detection_msg.confidence = float(face.confidence)
        detection_msg.embedding = np.asarray(face.embedding, dtype=np.float32).tolist()
        detection_msg.crop = bridge.cv2_to_imgmsg(face.crop, encoding="bgr8")
        detection_msg.crop.header = header
        detections_msg.faces.append(detection_msg)

    return detections_msg
