"""ROS node that detects faces, publishes crops, and emits embeddings."""
from __future__ import annotations

import hashlib
import json
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from types import ModuleType
from threading import Lock
from typing import Any, Dict, List, Mapping, MutableMapping, Optional, Sequence, TYPE_CHECKING

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
IDENTITY_RECALL_LIMIT = 6
IDENTITY_MIN_SCORE = 0.82


def _normalise_text(value: Any) -> str:
    if isinstance(value, str):
        text = value.strip()
        if text:
            return text
        return ""
    if isinstance(value, (int, float)):
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return ""
        if math.isnan(numeric) or math.isinf(numeric):
            return ""
        text = str(value).strip()
        return text
    return ""


def _normalise_list(value: Any) -> List[str]:
    items: List[str] = []
    if value is None:
        return items
    if isinstance(value, str):
        text = _normalise_text(value)
        if text:
            items.append(text)
        return items
    if isinstance(value, Sequence) and not isinstance(value, (bytes, bytearray, str)):
        for element in value:
            text = _normalise_text(element)
            if text and text not in items:
                items.append(text)
    return items


def _dedupe_preserving_order(items: Sequence[str]) -> List[str]:
    seen: set[str] = set()
    ordered: List[str] = []
    for item in items:
        if item not in seen:
            seen.add(item)
            ordered.append(item)
    return ordered


def _coerce_float(value: Any) -> Optional[float]:
    try:
        numeric = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if math.isnan(numeric) or math.isinf(numeric):
        return None
    return numeric


def _sanitize_identity_summary(raw: Mapping[str, Any]) -> Optional[Dict[str, Any]]:
    identity_id = (
        _normalise_text(raw.get("id"))
        or _normalise_text(raw.get("identity_id"))
        or _normalise_text(raw.get("uuid"))
    )
    name = _normalise_text(raw.get("name"))
    if not identity_id:
        identity_id = name
    if not identity_id:
        return None
    summary: Dict[str, Any] = {"id": identity_id}
    if name:
        summary["name"] = name
    aliases = _normalise_list(raw.get("aliases"))
    labels = _normalise_list(raw.get("labels"))
    if labels:
        aliases = _dedupe_preserving_order(aliases + labels)
    if aliases:
        summary["aliases"] = aliases
    signatures = _normalise_list(raw.get("signatures"))
    signature_history = _normalise_list(raw.get("signature_history"))
    if signature_history:
        signatures = _dedupe_preserving_order(signatures + signature_history)
    signature_hint = _normalise_text(raw.get("signature"))
    if signature_hint:
        signatures = _dedupe_preserving_order(signatures + [signature_hint])
    if signatures:
        summary["signatures"] = signatures
    memory_ids = _normalise_list(raw.get("memory_ids"))
    memory_hint = _normalise_text(raw.get("memory_id"))
    if memory_hint:
        memory_ids = _dedupe_preserving_order(memory_ids + [memory_hint])
    if memory_ids:
        summary["memory_ids"] = memory_ids
    confidence = _coerce_float(raw.get("confidence"))
    if confidence is None:
        confidence = _coerce_float(raw.get("score"))
    if confidence is not None:
        summary["confidence"] = confidence
    return summary


def _extract_identity_summary(metadata: Mapping[str, Any]) -> Optional[Dict[str, Any]]:
    if not isinstance(metadata, Mapping):
        return None
    identity_field = metadata.get("identity")
    if isinstance(identity_field, Mapping):
        summary = _sanitize_identity_summary(identity_field)
        if summary:
            return summary
    identities_field = metadata.get("identities")
    if isinstance(identities_field, Sequence):
        for entry in identities_field:
            if isinstance(entry, Mapping):
                summary = _sanitize_identity_summary(entry)
                if summary:
                    return summary
    fallback_name = _normalise_text(metadata.get("name"))
    if fallback_name:
        memory_hint = _normalise_text(metadata.get("memory_id"))
        return {"id": memory_hint or fallback_name, "name": fallback_name}
    return None


def resolve_identity_from_recall(
    signature: str,
    recall_results: Sequence[Mapping[str, Any]],
    *,
    min_score: float = IDENTITY_MIN_SCORE,
) -> tuple[Optional[Dict[str, Any]], List[Dict[str, Any]]]:
    best_identity: Optional[Dict[str, Any]] = None
    best_score = float("-inf")
    best_memory_id = ""
    cleaned_signature = _normalise_text(signature)
    matches: List[Dict[str, Any]] = []
    for result in recall_results:
        if not isinstance(result, Mapping):
            continue
        metadata_raw = result.get("metadata")
        metadata = metadata_raw if isinstance(metadata_raw, Mapping) else {}
        summary = _extract_identity_summary(metadata)
        memory_id_hint = (
            _normalise_text(result.get("memory_id"))
            or _normalise_text(metadata.get("memory_id"))
        )
        try:
            score = float(result.get("score", 0.0))
        except (TypeError, ValueError):
            score = 0.0
        signature_hint = _normalise_text(metadata.get("signature"))
        match_entry: Dict[str, Any] = {
            "memory_id": memory_id_hint,
            "score": score,
        }
        if signature_hint:
            match_entry["signature"] = signature_hint
        if summary is None and metadata:
            name = _normalise_text(metadata.get("name"))
            if name:
                summary = {"id": memory_id_hint or name, "name": name}
        if summary is not None:
            match_entry["identity"] = dict(summary)
        if match_entry.get("identity") or match_entry["memory_id"]:
            matches.append(match_entry)
        if summary is None:
            continue
        candidate = dict(summary)
        signatures = _normalise_list(candidate.get("signatures"))
        if signature_hint:
            signatures = _dedupe_preserving_order(signatures + [signature_hint])
        if cleaned_signature:
            signatures = _dedupe_preserving_order(signatures + [cleaned_signature])
        if signatures:
            candidate["signatures"] = signatures
        if memory_id_hint:
            memory_ids = _normalise_list(candidate.get("memory_ids"))
            memory_ids = _dedupe_preserving_order(memory_ids + [memory_id_hint])
            candidate["memory_ids"] = memory_ids
        if score >= min_score and (best_identity is None or score > best_score):
            best_identity = candidate
            best_score = score
            best_memory_id = memory_id_hint
    if best_identity is not None:
        best_identity["confidence"] = best_score
        if best_memory_id:
            memory_ids = _normalise_list(best_identity.get("memory_ids"))
            if best_memory_id not in memory_ids:
                memory_ids.append(best_memory_id)
            if memory_ids:
                best_identity["memory_ids"] = _dedupe_preserving_order(memory_ids)
    return best_identity, matches


def _enrich_identity_metadata(
    metadata: MutableMapping[str, Any],
    identity: Mapping[str, Any],
    signature: str,
) -> Dict[str, Any]:
    summary = _sanitize_identity_summary(identity) or {
        "id": _normalise_text(identity.get("id")) or _normalise_text(identity.get("name")) or signature
    }
    identity_copy: Dict[str, Any] = dict(summary)
    signatures = _normalise_list(identity_copy.get("signatures"))
    cleaned_signature = _normalise_text(signature)
    if cleaned_signature:
        signatures = _dedupe_preserving_order(signatures + [cleaned_signature])
    if signatures:
        identity_copy["signatures"] = signatures
    metadata["identity"] = identity_copy
    existing_list = metadata.get("identities")
    consolidated: List[Dict[str, Any]] = []
    seen: set[str] = set()
    for entry in [identity_copy] + (list(existing_list) if isinstance(existing_list, list) else []):
        if not isinstance(entry, Mapping):
            continue
        summary_entry = _sanitize_identity_summary(entry)
        if summary_entry is None:
            continue
        identity_id = summary_entry["id"]
        if identity_id in seen:
            continue
        seen.add(identity_id)
        consolidated.append(summary_entry)
    metadata["identities"] = consolidated
    name = _normalise_text(identity_copy.get("name"))
    if name:
        metadata["name"] = name
        tags = metadata.setdefault("tags", [])
        if isinstance(tags, list) and name not in tags:
            tags.append(name)
    return identity_copy



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
        self._warned: dict[str, bool] = {"memorize": False, "recall": False}
        self._MemoryEvent = None
        self._Memorize = None
        self._Recall = None
        self._memorize_client = None
        self._recall_client = None
        try:
            from memory_interfaces.msg import MemoryEvent as _MemoryEvent  # type: ignore[import]
            from memory_interfaces.srv import Memorize as _Memorize  # type: ignore[import]
        except ImportError:
            node.get_logger().info(
                "memory_interfaces not available; faces memory bridge disabled"
            )
            return
        try:
            from memory_interfaces.srv import Recall as _Recall  # type: ignore[import]
        except ImportError:
            _Recall = None
        self._MemoryEvent = _MemoryEvent
        self._Memorize = _Memorize
        self._memorize_client = node.create_client(self._Memorize, "/memory/memorize")
        if _Recall is not None:
            self._Recall = _Recall
            self._recall_client = node.create_client(self._Recall, "/memory/recall")
        else:  # pragma: no cover - optional dependency path
            self._node.get_logger().info(
                "memory recall service unavailable; identity resolution disabled"
            )
            self._Recall = None
            self._recall_client = None
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

    def recall(
        self,
        embedding: Sequence[float],
        *,
        limit: int = 5,
    ) -> List[dict[str, Any]]:
        if (
            not self._enabled
            or self._Recall is None
            or self._recall_client is None
        ):
            return []
        if not embedding:
            return []
        max_results = max(1, min(int(limit or 1), 20))
        if not self._recall_client.wait_for_service(timeout_sec=0.0):
            if not self._warned["recall"]:
                self._node.get_logger().warning(
                    "/memory/recall unavailable; face identification will rely on fallbacks"
                )
                self._warned["recall"] = True
            return []
        request = self._Recall.Request()
        request.kind = self._kind
        request.embedding = [float(value) for value in embedding]
        request.text = ""
        request.limit = max_results
        future = self._recall_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout)
        if not future.done():
            future.cancel()
            self._node.get_logger().warning("/memory/recall call timed out")
            return []
        exc = future.exception()
        if exc is not None:
            self._node.get_logger().warning(f"/memory/recall call failed: {exc}")
            return []
        response = future.result()
        if response is None:
            return []
        results: List[dict[str, Any]] = []
        for record in getattr(response, "results", ()):
            try:
                metadata = json.loads(record.json_metadata) if record.json_metadata else {}
            except json.JSONDecodeError:
                metadata = {"raw": record.json_metadata}
            try:
                score = float(record.score)
            except (TypeError, ValueError):
                score = 0.0
            results.append(
                {
                    "memory_id": getattr(record, "memory_id", "") or "",
                    "score": score,
                    "metadata": metadata,
                }
            )
        return results


class FaceDetectorNode(Node):
    """Detect faces from camera frames and publish crops and embeddings."""

    def __init__(self) -> None:
        processing = _get_processing_module()

        super().__init__("psyched_faces")
        self._bridge = CvBridge()
        service_uri_default = os.environ.get("FACES_SERVICE_URI", "").strip()
        service_timeout_default = 3.0
        timeout_env = os.environ.get("FACES_SERVICE_TIMEOUT_SEC")
        if timeout_env:
            try:
                service_timeout_default = float(timeout_env)
            except (TypeError, ValueError):
                service_timeout_default = 3.0
        service_model_default = os.environ.get("FACES_SERVICE_MODEL", "").strip()
        service_num_jitters_default = 0
        num_jitters_env = os.environ.get("FACES_SERVICE_NUM_JITTERS")
        if num_jitters_env:
            try:
                service_num_jitters_default = int(num_jitters_env)
            except (TypeError, ValueError):
                service_num_jitters_default = 0

        # default to the Kinect RGB stream published by the eye module
        self.declare_parameter("camera_topic", DEFAULT_CAMERA_TOPIC)
        self.declare_parameter("faces_topic", "/vision/faces")
        self.declare_parameter("face_detected_topic", "/vision/face_detected")
        self.declare_parameter("trigger_cooldown_sec", 2.0)
        self.declare_parameter("sensation_topic", "/sensations")
        self.declare_parameter("memory.enabled", True)
        self.declare_parameter("memory.flush", True)
        self.declare_parameter("memory.timeout_sec", 0.75)
        self.declare_parameter("memory.identity_min_score", IDENTITY_MIN_SCORE)
        self.declare_parameter("memory.identity_max_results", IDENTITY_RECALL_LIMIT)
        self.declare_parameter("embedding.service_uri", service_uri_default)
        self.declare_parameter("embedding.service_timeout_sec", service_timeout_default)
        self.declare_parameter("embedding.service_model", service_model_default)
        self.declare_parameter("embedding.service_num_jitters", service_num_jitters_default)

        service_uri_param = self._get_param("embedding.service_uri", service_uri_default)
        self._embedding_service_uri = str(service_uri_param).strip() if isinstance(service_uri_param, str) else service_uri_default

        service_timeout_param = self._get_param("embedding.service_timeout_sec", service_timeout_default)
        try:
            self._embedding_service_timeout = float(service_timeout_param)
        except (TypeError, ValueError):
            self._embedding_service_timeout = service_timeout_default
        if self._embedding_service_timeout <= 0.0:
            self._embedding_service_timeout = service_timeout_default

        service_model_param = self._get_param("embedding.service_model", service_model_default)
        self._embedding_service_model = (
            str(service_model_param).strip()
            if isinstance(service_model_param, str)
            else service_model_default
        )

        num_jitters_param = self._get_param("embedding.service_num_jitters", service_num_jitters_default)
        try:
            self._embedding_service_num_jitters = int(num_jitters_param)
        except (TypeError, ValueError):
            self._embedding_service_num_jitters = service_num_jitters_default
        if self._embedding_service_num_jitters < 0:
            self._embedding_service_num_jitters = 0

        embedder, backend_label = self._build_embedder(processing)
        self._embedding_backend_label = backend_label
        self._processor = processing.FaceProcessor(
            detector=processing.HaarCascadeDetector(),
            embedder=embedder,
        )
        self._embedding_dim = embedder.size

        self._camera_topic = self._get_param("camera_topic", DEFAULT_CAMERA_TOPIC)
        self._faces_topic = self._get_param("faces_topic", "/vision/faces")
        self._face_detected_topic = self._get_param("face_detected_topic", "/vision/face_detected")
        self._sensation_topic = self._get_param("sensation_topic", "/sensations")
        self._trigger_cooldown = float(self._get_param("trigger_cooldown_sec", 2.0))
        self._memory_flush = bool(self._get_param("memory.flush", True))
        identity_threshold = float(
            self._get_param("memory.identity_min_score", IDENTITY_MIN_SCORE)
        )
        identity_limit = int(
            self._get_param("memory.identity_max_results", IDENTITY_RECALL_LIMIT)
        )
        self._identity_match_threshold = max(0.0, min(identity_threshold, 1.0))
        self._identity_recall_limit = max(1, identity_limit)
        memory_enabled = bool(self._get_param("memory.enabled", True))
        memory_timeout = float(self._get_param("memory.timeout_sec", 0.75))
        self._memory_client = MemoryClient(self, kind="faces", timeout=memory_timeout) if memory_enabled else None
        self._last_signature: Optional[str] = None
        self._last_trigger_time: float = 0.0

        self._frame_lock = Lock()
        self._pending_frame: Optional[Image] = None
        self._processing_active = False
        self._dropped_frames = 0
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="faces-detector")
        self._camera_qos = self._build_camera_qos()

        self._detections_pub = self.create_publisher(FaceDetections, self._faces_topic, SensorDataQoS())
        self._trigger_pub = self.create_publisher(String, self._face_detected_topic, _best_effort_qos(depth=5))
        self._sensation_pub = self.create_publisher(SensationStamped, self._sensation_topic, SensorDataQoS())

        # create the subscription and keep a handle so we can change it at runtime
        self._camera_sub = self.create_subscription(
            Image,
            self._camera_topic,
            self._handle_image,
            self._camera_qos,
        )

        # watch for parameter updates so we can re-subscribe if camera_topic changes
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # pragma: no cover - logging only
        self.get_logger().info(
            f"Face detector initialised with camera_topic={self._camera_topic} faces_topic={self._faces_topic}"
        )
        # Extra visibility: node is watching for frames
        self.get_logger().info("Keeping an eye out for faces...")
        self.get_logger().info(
            f"Embedding backend ({self._embedding_backend_label}) dimension={self._embedding_dim}"
        )
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

    def _build_camera_qos(self):
        qos = SensorDataQoS()
        if hasattr(qos, "depth"):
            qos.depth = 1
            return qos
        if isinstance(qos, int):
            return 1
        return qos

    def _handle_image(self, msg: Image) -> None:
        with self._frame_lock:
            if self._processing_active:
                dropped = self._pending_frame is not None
                self._pending_frame = msg
                if dropped:
                    self._dropped_frames += 1
                    if self._dropped_frames == 1 or (self._dropped_frames % 25) == 0:
                        self.get_logger().warning(
                            f"Face detector busy; replacing {self._dropped_frames} queued frame(s)"
                        )
                else:
                    self._dropped_frames = 0
                return
            self._processing_active = True

        try:
            self._executor.submit(self._process_frames, msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to schedule face processing task: {exc}")
            with self._frame_lock:
                self._processing_active = False

    def _process_frames(self, first_msg: Image) -> None:
        next_msg: Optional[Image] = first_msg
        while next_msg is not None:
            try:
                self._process_frame(next_msg)
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().error(f"Face processing failed: {exc}")

            drop_count = 0
            with self._frame_lock:
                next_msg = self._pending_frame
                self._pending_frame = None
                if next_msg is None:
                    drop_count = self._dropped_frames
                    self._dropped_frames = 0
                    self._processing_active = False
            if next_msg is None and drop_count:
                self.get_logger().warning(
                    f"Face detector skipped {drop_count} intermediate frame(s) to catch up"
                )

    def _process_frame(self, msg: Image) -> None:
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
            name_hint = _normalise_text(first_payload.get("name")) or "Stranger"
            payload = {
                "name": name_hint,
                "memory_id": memory_hint,
                "vector_id": vector_hint,
                "collection": first_payload.get("collection", "faces"),
            }
            identity_hint = first_payload.get("identity")
            if isinstance(identity_hint, Mapping):
                payload["identity"] = identity_hint
            matches_hint = first_payload.get("matches")
            if isinstance(matches_hint, list) and matches_hint:
                payload["matches"] = matches_hint
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
            identity_matches: List[Dict[str, Any]] = []
            resolved_identity: Optional[Dict[str, Any]] = None
            if memory_client is not None:
                try:
                    recall_candidates = memory_client.recall(
                        embedding=embedding_values,
                        limit=self._identity_recall_limit,
                    )
                except Exception as exc:  # pragma: no cover - defensive guard
                    self.get_logger().warning(f"Face identity recall failed: {exc}")
                    recall_candidates = []
                resolved_identity, identity_matches = resolve_identity_from_recall(
                    signature,
                    recall_candidates,
                    min_score=self._identity_match_threshold,
                )
            metadata = self._build_memory_metadata(
                frame_id=frame_id,
                bbox=bbox,
                face=face,
                embedding_dim=len(embedding_values),
                signature=signature,
            )
            if resolved_identity is not None:
                resolved_identity = _enrich_identity_metadata(metadata, resolved_identity, signature)
            else:
                metadata.setdefault("name", "Unknown face")
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
            if resolved_identity is not None and stored_memory_id:
                memory_ids = resolved_identity.setdefault("memory_ids", [])
                if stored_memory_id not in memory_ids:
                    memory_ids.append(stored_memory_id)
            fallback_memory_id = stored_memory_id or f"mem-{signature}"
            fallback_vector_id = stored_vector_id or signature

            payload: Dict[str, Any] = {
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
            if resolved_identity is not None:
                payload["name"] = resolved_identity.get("name", "Unknown face")
                payload["identity"] = resolved_identity
                if identity_matches:
                    payload["matches"] = identity_matches
                confidence = resolved_identity.get("confidence")
                if isinstance(confidence, (int, float)):
                    payload["identity_confidence"] = float(confidence)
            else:
                payload["name"] = metadata.get("name", "Unknown face")
                if identity_matches:
                    payload["matches"] = identity_matches

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

    def _build_embedder(self, processing: ModuleType) -> tuple["EmbeddingExtractor", str]:
        service_uri = (self._embedding_service_uri or "").strip()
        if service_uri:
            service_result = self._build_service_embedder(processing, service_uri)
            if service_result is not None:
                return service_result
        return self._build_local_embedder(processing, log=True)

    def _build_service_embedder(
        self,
        processing: ModuleType,
        uri: str,
    ) -> Optional[tuple["EmbeddingExtractor", str]]:
        try:
            from .service_client import ServiceEmbeddingExtractor
        except ModuleNotFoundError as exc:
            self.get_logger().warning(
                f"Embedding service disabled; client dependency missing ({exc})"
            )
            return None
        except Exception as exc:  # pragma: no cover - defensive import guard
            self.get_logger().warning(
                f"Embedding service disabled; client initialisation failed ({exc})"
            )
            return None

        fallback_embedder, fallback_label = self._build_local_embedder(processing, log=False)
        try:
            embedder = ServiceEmbeddingExtractor(
                uri,
                timeout=self._embedding_service_timeout,
                model=self._embedding_service_model or None,
                num_jitters=(
                    self._embedding_service_num_jitters
                    if self._embedding_service_num_jitters > 0
                    else None
                ),
                fallback=fallback_embedder,
                logger=self.get_logger(),
                expected_dim=128,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"Embedding service initialisation failed ({exc}); using local embeddings instead"
            )
            return None

        self.get_logger().info(
            "Using remote face embedding service at %s (timeout=%.2fs)",
            uri,
            self._embedding_service_timeout,
        )
        if fallback_label == "face_recognition":
            self.get_logger().info(
                "Remote embedding fallback: local face_recognition backend"
            )
        elif fallback_label == "basic":
            self.get_logger().info(
                "Remote embedding fallback: basic downsampled embeddings"
            )
        label = "service+" + fallback_label if fallback_label else "service"
        return embedder, label

    def _build_local_embedder(
        self,
        processing: ModuleType,
        *,
        log: bool,
    ) -> tuple["EmbeddingExtractor", str]:
        unavailable_exc = getattr(processing, "EmbeddingBackendUnavailableError", RuntimeError)
        try:
            embedder = processing.FaceRecognitionEmbeddingExtractor(
                model=self._embedding_service_model or "small",
                num_jitters=max(1, int(self._embedding_service_num_jitters or 1)),
            )
        except unavailable_exc as exc:
            if log:
                self.get_logger().warning(f"face_recognition embeddings unavailable: {exc}")
        except Exception as exc:  # pragma: no cover - defensive
            if log:
                self.get_logger().warning(
                    f"Failed to initialise face_recognition embeddings: {exc}"
                )
        else:
            if log:
                self.get_logger().info("Using face_recognition embeddings (128D)")
            return embedder, "face_recognition"

        try:
            embedder = processing.BasicEmbeddingExtractor()
        except Exception as exc:  # pragma: no cover - defensive
            if log:
                self.get_logger().error(
                    f"Failed to initialise basic embedding extractor: {exc}"
                )
            raise
        if log:
            self.get_logger().info("Falling back to basic downsampled embeddings (256D)")
        return embedder, "basic"

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
                self._camera_qos,
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

    def destroy_node(self) -> bool:
        try:
            self._executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
        return super().destroy_node()


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
