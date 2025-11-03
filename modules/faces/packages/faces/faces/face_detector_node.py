"""ROS node that detects faces, publishes crops, and emits embeddings."""
from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass
from types import ModuleType
from typing import Any, Mapping, Optional, Sequence, TYPE_CHECKING

from rcl_interfaces.msg import SetParametersResult

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import Image
from psyched_msgs.msg import SensationStamped
from std_msgs.msg import Header, String

from faces_msgs.msg import FaceDetections

from .message_builder import build_face_detections_msg

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .processing import (  # type: ignore[attr-defined]
        BasicEmbeddingExtractor,
        EmbeddingBackendUnavailableError,
        EmbeddingExtractor,
        FaceProcessor,
        ProcessedFace,
        HaarCascadeDetector,
        FaceRecognitionEmbeddingExtractor,
    )

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
_PROCESSING_MODULE: ModuleType | None = None
_PROCESSING_IMPORT_ERROR: ModuleNotFoundError | None = None


class MissingDependencyError(RuntimeError):
    """Raised when the faces module is missing runtime dependencies."""


DEFAULT_CAMERA_TOPIC = "/camera/color/image_raw"


def _missing_dependency_message(missing: str) -> str:
    human_readable = "OpenCV (cv2)" if missing == "cv2" else missing
    return (
        "faces module requires the '{}' dependency. "
        "Run `psh mod setup faces` or install the appropriate system packages.".format(human_readable)
    )


def _get_processing_module() -> ModuleType:
    """Return the cached ``faces.processing`` module or raise with guidance."""

    global _PROCESSING_MODULE, _PROCESSING_IMPORT_ERROR

    if _PROCESSING_MODULE is not None:
        return _PROCESSING_MODULE
    if _PROCESSING_IMPORT_ERROR is not None:
        missing = getattr(_PROCESSING_IMPORT_ERROR, "name", "cv2")
        raise MissingDependencyError(_missing_dependency_message(missing)) from _PROCESSING_IMPORT_ERROR
    try:
        from . import processing as _processing  # type: ignore[import-not-found]
    except ModuleNotFoundError as exc:  # pragma: no cover - dependency resolution
        _PROCESSING_IMPORT_ERROR = exc
        missing = getattr(exc, "name", "cv2")
        raise MissingDependencyError(_missing_dependency_message(missing)) from exc
    _PROCESSING_MODULE = _processing
    return _processing


@dataclass(frozen=True)
class MemoryWriteResult:
    """Outcome returned after attempting to persist an embedding."""

    memory_id: Optional[str]
    vector_id: Optional[str]


class MemoryClient:
    """Thin wrapper around the memory module's ROS services."""

    def __init__(self, node: Node, *, kind: str = "faces", timeout: float = 0.75) -> None:
        self._node = node
        self._kind = kind
        self._timeout = float(timeout)
        self._enabled = False
        self._warned: dict[str, bool] = {"memorize": False}
        try:
            from memory_interfaces.msg import MemoryEvent as _MemoryEvent  # type: ignore[import]
            from memory_interfaces.srv import Memorize as _Memorize  # type: ignore[import]
        except ImportError:
            node.get_logger().info(
                "memory_interfaces not available; faces memory bridge disabled"
            )
            self._MemoryEvent = None
            self._Memorize = None
            self._memorize_client = None
            return

        self._MemoryEvent = _MemoryEvent
        self._Memorize = _Memorize
        self._memorize_client = node.create_client(self._Memorize, "/memory/memorize")
        self._enabled = True

    @property
    def enabled(self) -> bool:
        return self._enabled

    def memorize(
        self,
        *,
        header: Optional[Header],
        embedding: Sequence[float],
        metadata: Mapping[str, Any],
        flush: bool,
    ) -> MemoryWriteResult:
        if not self._enabled or self._MemoryEvent is None or self._memorize_client is None:
            return MemoryWriteResult(memory_id=None, vector_id=None)
        if not embedding:
            return MemoryWriteResult(memory_id=None, vector_id=None)
        if not self._memorize_client.wait_for_service(timeout_sec=0.0):
            if not self._warned["memorize"]:
                self._node.get_logger().warning(
                    "/memory/memorize unavailable; face embeddings will stay local until it appears"
                )
                self._warned["memorize"] = True
            return MemoryWriteResult(memory_id=None, vector_id=None)

        event_msg = self._MemoryEvent()
        try:
            if header is not None:
                event_msg.header.stamp = header.stamp
                event_msg.header.frame_id = getattr(header, "frame_id", "") or "faces"
            else:
                event_msg.header.frame_id = "faces"
        except Exception:
            event_msg.header.frame_id = "faces"
        event_msg.kind = self._kind
        event_msg.json_data = json.dumps(dict(metadata), separators=(",", ":"))
        event_msg.embedding = [float(value) for value in embedding]

        request = self._Memorize.Request()
        request.event = event_msg
        request.flush = bool(flush)
        future = self._memorize_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout)
        if not future.done():
            future.cancel()
            self._node.get_logger().warning("/memory/memorize call timed out")
            return MemoryWriteResult(memory_id=None, vector_id=None)
        exc = future.exception()
        if exc is not None:
            self._node.get_logger().warning(f"/memory/memorize call failed: {exc}")
            return MemoryWriteResult(memory_id=None, vector_id=None)
        response = future.result()
        if response is None:
            return MemoryWriteResult(memory_id=None, vector_id=None)
        memory_id = response.memory_id or None
        vector_id = response.vector_id or None
        return MemoryWriteResult(memory_id=memory_id, vector_id=vector_id)


class FaceDetectorNode(Node):
    """Detect faces from camera frames and publish crops and embeddings."""

    def __init__(self) -> None:
        processing = _get_processing_module()

        super().__init__("psyched_faces")
        self._bridge = CvBridge()
        embedder = self._build_embedder(processing)
        self._processor = processing.FaceProcessor(
            detector=processing.HaarCascadeDetector(),
            embedder=embedder,
        )
        self._embedding_dim = embedder.size

        # default to the Kinect RGB stream published by the eye module
        self.declare_parameter("camera_topic", DEFAULT_CAMERA_TOPIC)
        self.declare_parameter("faces_topic", "/vision/faces")
        self.declare_parameter("face_detected_topic", "/vision/face_detected")
        self.declare_parameter("trigger_cooldown_sec", 2.0)
        self.declare_parameter("sensation_topic", "/sensations")
        self.declare_parameter("memory.enabled", True)
        self.declare_parameter("memory.flush", True)
        self.declare_parameter("memory.timeout_sec", 0.75)

        self._camera_topic = self._get_param("camera_topic", DEFAULT_CAMERA_TOPIC)
        self._faces_topic = self._get_param("faces_topic", "/vision/faces")
        self._face_detected_topic = self._get_param("face_detected_topic", "/vision/face_detected")
        self._sensation_topic = self._get_param("sensation_topic", "/sensations")
        self._trigger_cooldown = float(self._get_param("trigger_cooldown_sec", 2.0))
        self._memory_flush = bool(self._get_param("memory.flush", True))
        memory_enabled = bool(self._get_param("memory.enabled", True))
        memory_timeout = float(self._get_param("memory.timeout_sec", 0.75))
        self._memory_client = MemoryClient(self, kind="faces", timeout=memory_timeout) if memory_enabled else None
        self._last_signature: Optional[str] = None
        self._last_trigger_time: float = 0.0

        self._detections_pub = self.create_publisher(FaceDetections, self._faces_topic, SensorDataQoS())
        self._trigger_pub = self.create_publisher(String, self._face_detected_topic, _best_effort_qos(depth=5))
        self._sensation_pub = self.create_publisher(SensationStamped, self._sensation_topic, SensorDataQoS())

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
        self.get_logger().info(f"Embedding backend dimension={self._embedding_dim}")
        if self._memory_client is None or not self._memory_client.enabled:
            self.get_logger().info("Face memory bridge disabled; embeddings will not be persisted")
        else:
            self.get_logger().info(
                f"Face memory bridge enabled (flush={'on' if self._memory_flush else 'off'})"
            )

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
        sensation_payloads = self._publish_sensations(msg.header, faces)

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
            first_payload = sensation_payloads[0] if sensation_payloads else {}
            memory_hint = str(
                first_payload.get("memory_id") or first_payload.get("memory_hint") or ""
            )
            vector_hint = str(
                first_payload.get("vector_id") or first_payload.get("vector_hint") or ""
            )
            payload = {
                "name": "Stranger",
                "memory_id": memory_hint,
                "vector_id": vector_hint,
                "collection": first_payload.get("collection", "faces"),
            }
            signature_hint = str(first_payload.get("signature") or signature or "")
            if signature_hint:
                payload["signature"] = signature_hint
            memory_fallback = str(first_payload.get("memory_hint") or "")
            if memory_fallback:
                payload["memory_hint"] = memory_fallback
            vector_fallback = str(first_payload.get("vector_hint") or "")
            if vector_fallback:
                payload["vector_hint"] = vector_fallback
            trigger_msg = String()
            trigger_msg.data = json.dumps(payload)
            self._trigger_pub.publish(trigger_msg)
            self.get_logger().info(f"Published trigger ({decision}) -> {self._face_detected_topic}: {payload}")
            self._last_signature = signature
            self._last_trigger_time = now
        else:
            self.get_logger().info(f"Skipping trigger publish ({decision}) for signature {signature}")

    def _publish_sensations(self, header: Header, faces: Sequence["ProcessedFace"]) -> list[dict[str, Any]]:
        payloads: list[dict[str, Any]] = []
        if not faces:
            return payloads
        frame_id = getattr(header, "frame_id", "") if header is not None else ""
        memory_client = self._memory_client if self._memory_client and self._memory_client.enabled else None

        for face in faces:
            bbox = {
                "x": int(face.bbox.x),
                "y": int(face.bbox.y),
                "width": int(face.bbox.width),
                "height": int(face.bbox.height),
            }
            embedding_values = np.asarray(face.embedding, dtype=np.float32).tolist()
            signature = self._derive_signature(face.embedding)
            metadata = self._build_memory_metadata(
                frame_id=frame_id,
                bbox=bbox,
                face=face,
                embedding_dim=len(embedding_values),
                signature=signature,
            )
            memory_result = MemoryWriteResult(memory_id=None, vector_id=None)
            if memory_client is not None:
                memory_result = memory_client.memorize(
                    header=header,
                    embedding=embedding_values,
                    metadata=metadata,
                    flush=self._memory_flush,
                )
            stored_memory_id = memory_result.memory_id or ""
            stored_vector_id = memory_result.vector_id or ""
            fallback_memory_id = stored_memory_id or f"mem-{signature}"
            fallback_vector_id = stored_vector_id or signature

            payload = {
                "memory_id": stored_memory_id,
                "memory_hint": fallback_memory_id,
                "vector_id": stored_vector_id,
                "vector_hint": fallback_vector_id,
                "signature": signature,
                "vector_preview": embedding_values[:8],
                "collection": "faces",
                "confidence": float(face.confidence),
                "frame_id": frame_id,
                "bbox": bbox,
                "topic": self._faces_topic,
                "embedding_dim": len(embedding_values),
            }

            sensation = SensationStamped()
            sensation.stamp = header.stamp if header is not None else None
            sensation.kind = "face"
            sensation.collection_hint = "faces"
            sensation.json_payload = json.dumps(payload, separators=(",", ":"))
            sensation.vector = embedding_values
            self._sensation_pub.publish(sensation)
            payloads.append(payload)

        self.get_logger().info(
            f"Published {len(faces)} face embedding sensations to {self._sensation_topic}"
        )
        return payloads

    def _build_embedder(self, processing: ModuleType) -> EmbeddingExtractor:
        try:
            embedder = processing.FaceRecognitionEmbeddingExtractor()
            self.get_logger().info("Using face_recognition embeddings (128D)")
            return embedder
        except (AttributeError, NameError):  # pragma: no cover - attribute missing
            pass
        except EmbeddingBackendUnavailableError as exc:
            self.get_logger().warning(f"face_recognition embeddings unavailable: {exc}")
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warning(f"Failed to initialise face_recognition embeddings: {exc}")
        self.get_logger().info("Falling back to basic downsampled embeddings (256D)")
        return processing.BasicEmbeddingExtractor()

    def _build_memory_metadata(
        self,
        *,
        frame_id: str,
        bbox: Mapping[str, int],
        face: "ProcessedFace",
        embedding_dim: int,
        signature: str,
    ) -> dict[str, Any]:
        return {
            "source": "faces/detector",
            "topic": self._faces_topic,
            "collection_hint": "faces",
            "frame_id": frame_id or "faces",
            "embedding_dim": int(embedding_dim),
            "signature": signature,
            "payload": {
                "bbox": dict(bbox),
                "confidence": float(face.confidence),
                "topic": self._faces_topic,
                "signature": signature,
            },
        }

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
    try:
        node = FaceDetectorNode()
    except MissingDependencyError as exc:
        get_logger("psyched_faces").error(str(exc))
        rclpy.shutdown()
        raise SystemExit(1) from exc
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - manual shutdown path
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - script entry point
    main()
