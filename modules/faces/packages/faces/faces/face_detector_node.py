"""ROS node that detects faces, publishes crops, and emits embeddings."""
from __future__ import annotations

import hashlib
import json
import time
from typing import Optional
from rcl_interfaces.msg import SetParametersResult

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from faces_msgs.msg import FaceDetections

from .message_builder import build_face_detections_msg
from .processing import BasicEmbeddingExtractor, FaceProcessor, HaarCascadeDetector

try:  # pragma: no cover - requires ROS 2 runtime
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
        SensorDataQoS,
    )
except ImportError:  # pragma: no cover - exercised in unit tests without rclpy
    QoSDurabilityPolicy = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    QoSReliabilityPolicy = None  # type: ignore[assignment]

    def SensorDataQoS():  # type: ignore[misc]
        return 10


def _best_effort_qos(*, depth: int = 10):
    if (
        QoSProfile is None
        or QoSHistoryPolicy is None
        or QoSReliabilityPolicy is None
        or QoSDurabilityPolicy is None
    ):
        return depth
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class FaceDetectorNode(Node):
    """Detect faces from camera frames and publish crops and embeddings."""

    def __init__(self) -> None:
        super().__init__("psyched_faces")
        self._bridge = CvBridge()
        self._processor = FaceProcessor(
            detector=HaarCascadeDetector(),
            embedder=BasicEmbeddingExtractor(),
        )

        # prefer the generic /image_raw topic (and its children) by default
        self.declare_parameter("camera_topic", "/image_raw")
        self.declare_parameter("faces_topic", "/vision/faces")
        self.declare_parameter("face_detected_topic", "/vision/face_detected")
        self.declare_parameter("trigger_cooldown_sec", 2.0)

        self._camera_topic = self._get_param("camera_topic", "/image_raw")
        self._faces_topic = self._get_param("faces_topic", "/vision/faces")
        self._face_detected_topic = self._get_param("face_detected_topic", "/vision/face_detected")
        self._trigger_cooldown = float(self._get_param("trigger_cooldown_sec", 2.0))
        self._last_signature: Optional[str] = None
        self._last_trigger_time: float = 0.0

        self._detections_pub = self.create_publisher(FaceDetections, self._faces_topic, SensorDataQoS())
        self._trigger_pub = self.create_publisher(String, self._face_detected_topic, _best_effort_qos(depth=5))

        # create the subscription and keep a handle so we can change it at runtime
        self._camera_sub = self.create_subscription(
            Image,
            self._camera_topic,
            self._handle_image,
            SensorDataQoS(),
        )

        # watch for parameter updates so we can re-subscribe if camera_topic changes
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # pragma: no cover - logging only
        self.get_logger().info(
            f"Face detector initialised with camera_topic={self._camera_topic} faces_topic={self._faces_topic}"
        )
        # Extra visibility: node is watching for frames
        self.get_logger().info("Keeping an eye out for faces...")

    def _get_param(self, name: str, default: object) -> object:
        value = self.get_parameter(name).value
        if value is None:
            return default
        if isinstance(value, str) and not value:
            return default
        return value

    def _handle_image(self, msg: Image) -> None:
        # Log that we've received a frame and include some basic header info
        try:
            stamp = msg.header.stamp
            stamp_str = f"{stamp.sec}.{stamp.nanosec:09d}"
        except Exception:
            stamp_str = "unknown"
        self.get_logger().info(f"_handle_image called: header.stamp={stamp_str} width={msg.width} height={msg.height}")

        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        faces = self._processor.process(image)
        if not faces:
            self.get_logger().info("No faces detected in current frame")
            return

        self.get_logger().info(f"Detected {len(faces)} face(s) in current frame")

        detections_msg = build_face_detections_msg(msg.header, faces, bridge=self._bridge)
        self._detections_pub.publish(detections_msg)
        self.get_logger().info(f"Published FaceDetections to {self._faces_topic} (faces={len(faces)})")

        signature = self._derive_signature(faces[0].embedding)
        now = time.monotonic()
        decision = None
        if signature != self._last_signature:
            decision = "new_signature"
        elif (now - self._last_trigger_time) > self._trigger_cooldown:
            decision = "cooldown_elapsed"
        else:
            decision = "duplicate_within_cooldown"

        if decision in ("new_signature", "cooldown_elapsed"):
            payload = {"name": "Stranger", "signature": signature}
            trigger_msg = String()
            trigger_msg.data = json.dumps(payload)
            self._trigger_pub.publish(trigger_msg)
            self.get_logger().info(f"Published trigger ({decision}) -> {self._face_detected_topic}: {payload}")
            self._last_signature = signature
            self._last_trigger_time = now
        else:
            self.get_logger().info(f"Skipping trigger publish ({decision}) for signature {signature}")

    def _on_set_parameters(self, params: list) -> SetParametersResult:
        """Handle parameter updates; when camera_topic changes, recreate subscription."""
        new_topic: Optional[str] = None
        for p in params:
            if p.name == "camera_topic":
                # ignore empty strings
                if isinstance(p.value, str) and p.value:
                    new_topic = p.value
                break

        if new_topic is None or new_topic == self._camera_topic:
            return SetParametersResult(successful=True)

        try:
            # remove old subscription
            try:
                self.destroy_subscription(self._camera_sub)
            except Exception:
                pass
            # create new subscription
            self._camera_topic = new_topic
            self._camera_sub = self.create_subscription(
                Image,
                self._camera_topic,
                self._handle_image,
                SensorDataQoS(),
            )
            self.get_logger().info(f"Re-subscribed to camera topic: {self._camera_topic}")
            return SetParametersResult(successful=True)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f"Failed to update camera subscription: {exc}")
            return SetParametersResult(successful=False)

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
