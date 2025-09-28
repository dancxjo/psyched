"""ROS node that detects faces, publishes crops, and emits embeddings."""
from __future__ import annotations

import hashlib
import json
import time
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from faces_msgs.msg import FaceDetections

from .message_builder import build_face_detections_msg
from .processing import BasicEmbeddingExtractor, FaceProcessor, HaarCascadeDetector


class FaceDetectorNode(Node):
    """Detect faces from camera frames and publish crops and embeddings."""

    def __init__(self) -> None:
        super().__init__("psyched_faces")
        self._bridge = CvBridge()
        self._processor = FaceProcessor(
            detector=HaarCascadeDetector(),
            embedder=BasicEmbeddingExtractor(),
        )

        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("faces_topic", "/vision/faces")
        self.declare_parameter("face_detected_topic", "/vision/face_detected")
        self.declare_parameter("trigger_cooldown_sec", 2.0)

        self._camera_topic = self._get_param("camera_topic", "/camera/color/image_raw")
        self._faces_topic = self._get_param("faces_topic", "/vision/faces")
        self._face_detected_topic = self._get_param("face_detected_topic", "/vision/face_detected")
        self._trigger_cooldown = float(self._get_param("trigger_cooldown_sec", 2.0))
        self._last_signature: Optional[str] = None
        self._last_trigger_time: float = 0.0

        self._detections_pub = self.create_publisher(FaceDetections, self._faces_topic, 10)
        self._trigger_pub = self.create_publisher(String, self._face_detected_topic, 10)

        self.create_subscription(Image, self._camera_topic, self._handle_image, 10)

        self.get_logger().info(
            "Face detector initialised with camera_topic=%s faces_topic=%s",  # pragma: no cover - logging only
            self._camera_topic,
            self._faces_topic,
        )

    def _get_param(self, name: str, default: object) -> object:
        value = self.get_parameter(name).value
        if value is None:
            return default
        if isinstance(value, str) and not value:
            return default
        return value

    def _handle_image(self, msg: Image) -> None:
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        faces = self._processor.process(image)
        if not faces:
            return

        detections_msg = build_face_detections_msg(msg.header, faces, bridge=self._bridge)
        self._detections_pub.publish(detections_msg)

        signature = self._derive_signature(faces[0].embedding)
        now = time.monotonic()
        if signature != self._last_signature or (now - self._last_trigger_time) > self._trigger_cooldown:
            payload = {"name": "Stranger", "signature": signature}
            trigger_msg = String()
            trigger_msg.data = json.dumps(payload)
            self._trigger_pub.publish(trigger_msg)
            self._last_signature = signature
            self._last_trigger_time = now

    @staticmethod
    def _derive_signature(embedding: np.ndarray) -> str:
        """Return a stable short signature derived from the embedding."""

        digest = hashlib.sha1(np.asarray(embedding, dtype=np.float32).tobytes()).hexdigest()
        return digest[:12]


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FaceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - manual shutdown path
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - script entry point
    main()
