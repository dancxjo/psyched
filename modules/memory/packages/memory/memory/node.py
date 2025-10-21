"""ROS 2 node exposing the memory services described in the spec."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Mapping, Optional, Sequence

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String as StdString

from memory_interfaces.msg import MemoryEvent as MemoryEventMsg
from memory_interfaces.msg import MemoryRecall as MemoryRecallMsg
from memory_interfaces.srv import Associate as AssociateSrv
from memory_interfaces.srv import Memorize as MemorizeSrv
from memory_interfaces.srv import Recall as RecallSrv

from .models import MemoryEventPayload, MemoryHeader, MemoryRecord
from .service import MemoryService
from .stores import Neo4jGraphStore, QdrantVectorStore


class MemoryNode(Node):
    """Expose memorize, associate, and recall services over ROS 2."""

    def __init__(self, service: Optional[MemoryService] = None) -> None:
        super().__init__("memory")
        self._service = service or self._build_service()
        self._memorize_srv = self.create_service(MemorizeSrv, "/memory/memorize", self._handle_memorize)
        self._associate_srv = self.create_service(AssociateSrv, "/memory/associate", self._handle_associate)
        self._recall_srv = self.create_service(RecallSrv, "/memory/recall", self._handle_recall)
        self._pilot_feed_publisher = self._create_pilot_feed_publisher()
        self._closeables = self._collect_closeables()

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------
    def _handle_memorize(self, request: MemorizeSrv.Request, response: MemorizeSrv.Response) -> MemorizeSrv.Response:
        try:
            event = self._convert_event(request.event)
            record = self._service.memorize(event, flush=request.flush)
            response.memory_id = record.memory_id
            response.vector_id = record.vector_id or ""
            self._publish_pilot_feed(record)
        except Exception:  # pragma: no cover - defensive logging
            self.get_logger().exception("Failed to memorize event")
            response.memory_id = ""
            response.vector_id = ""
        return response

    def _handle_associate(self, request: AssociateSrv.Request, response: AssociateSrv.Response) -> AssociateSrv.Response:
        try:
            properties = self._decode_json(request.json_properties)
            self._service.associate(
                request.source_id,
                request.target_id,
                relation_type=request.relation_type or "ASSOCIATED_WITH",
                properties=properties,
            )
            response.success = True
        except Exception:  # pragma: no cover - defensive logging
            self.get_logger().exception("Failed to associate memories")
            response.success = False
        return response

    def _handle_recall(self, request: RecallSrv.Request, response: RecallSrv.Response) -> RecallSrv.Response:
        try:
            embedding: Optional[Sequence[float]] = list(request.embedding) if request.embedding else None
            text_query = request.text or None
            results = self._service.recall(
                kind=request.kind,
                embedding=embedding,
                text=text_query,
                limit=request.limit or 5,
            )
            response.results = [
                MemoryRecallMsg(
                    memory_id=result.memory_id,
                    score=float(result.score),
                    json_metadata=self._encode_json(result.metadata),
                )
                for result in results
            ]
        except Exception:  # pragma: no cover - defensive logging
            self.get_logger().exception("Failed to recall memories")
            response.results = []
        return response

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _build_service(self) -> MemoryService:
        qdrant_url = str(self.declare_parameter("qdrant.url", "http://localhost:6333").value)
        qdrant_api_key = str(self.declare_parameter("qdrant.api_key", "").value)
        neo4j_uri = str(self.declare_parameter("neo4j.uri", "bolt://localhost:7687").value)
        neo4j_user = str(self.declare_parameter("neo4j.user", "neo4j").value)
        neo4j_password = str(self.declare_parameter("neo4j.password", "test").value)
        batch_size = int(self.declare_parameter("memory.batch_size", 10).value)

        vector_store = self._create_vector_store(qdrant_url, qdrant_api_key)
        graph_store = self._create_graph_store(neo4j_uri, neo4j_user, neo4j_password)
        return MemoryService(vector_store=vector_store, graph_store=graph_store, batch_size=batch_size)

    def _create_vector_store(self, url: str, api_key: str) -> QdrantVectorStore:
        try:
            from qdrant_client import QdrantClient
        except ImportError as exc:  # pragma: no cover - import guard
            raise RuntimeError("qdrant-client must be installed to launch the memory node") from exc

        client = QdrantClient(url=url, api_key=api_key or None)
        return QdrantVectorStore(client)

    def _create_graph_store(self, uri: str, user: str, password: str) -> Neo4jGraphStore:
        return Neo4jGraphStore(uri, user, password)

    def _collect_closeables(self) -> list[object]:
        closeables: list[object] = []
        for candidate in (self._service.vector_store, self._service.graph_store):
            if hasattr(candidate, "close"):
                closeables.append(candidate)
        return closeables

    def destroy_node(self) -> bool:
        publisher = getattr(self, "_pilot_feed_publisher", None)
        if publisher is not None:
            try:
                self.destroy_publisher(publisher)
            except Exception:
                self.get_logger().debug("Failed to destroy pilot feed publisher", exc_info=True)
            finally:
                self._pilot_feed_publisher = None
        try:
            return super().destroy_node()
        finally:
            for closeable in self._closeables:
                close = getattr(closeable, "close", None)
                if callable(close):
                    close()
            self._closeables.clear()

    def _create_pilot_feed_publisher(self):
        try:
            qos = QoSProfile(depth=30)
            qos.history = QoSHistoryPolicy.KEEP_LAST
            qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
            return self.create_publisher(StdString, "/memory/pilot_feed", qos)
        except Exception:
            self.get_logger().warning(
                "Failed to initialise cockpit memory feed publisher; pilot dashboard logs disabled"
            )
            return None

    def _publish_pilot_feed(self, record: MemoryRecord) -> None:
        publisher = getattr(self, "_pilot_feed_publisher", None)
        if publisher is None:
            return
        payload = {
            "memory_id": record.memory_id,
            "vector_id": record.vector_id,
            "kind": record.kind,
            "frame_id": record.frame_id,
            "source": record.source,
            "timestamp": record.timestamp.isoformat(),
            "metadata": record.metadata,
            "summary": self._summarise_metadata(record.metadata),
        }
        labels = self._extract_labels(record.metadata)
        if labels:
            payload["labels"] = labels
        try:
            message = StdString()
            message.data = self._encode_json(payload)
            publisher.publish(message)
        except Exception:
            self.get_logger().debug("Failed to publish memory pilot feed entry", exc_info=True)

    @staticmethod
    def _convert_event(message: MemoryEventMsg) -> MemoryEventPayload:
        stamp = datetime.fromtimestamp(
            float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000,
            tz=timezone.utc,
        )
        metadata = MemoryNode._decode_json(message.json_data)
        source = metadata.get("source") if isinstance(metadata, dict) else None
        header = MemoryHeader(
            stamp=stamp,
            frame_id=message.header.frame_id,
            source=source or message.header.frame_id or "unknown",
        )
        embedding = list(message.embedding) if message.embedding else None
        return MemoryEventPayload(header=header, kind=message.kind, json_data=metadata, embedding=embedding)

    @staticmethod
    def _decode_json(raw: str) -> dict:
        if not raw:
            return {}
        try:
            value = json.loads(raw)
            if not isinstance(value, dict):
                raise ValueError("JSON payload must decode to a mapping")
            return value
        except json.JSONDecodeError as exc:
            raise ValueError("Invalid JSON payload") from exc

    @staticmethod
    def _encode_json(data: object) -> str:
        return json.dumps(data, separators=(",", ":"))

    @staticmethod
    def _summarise_metadata(metadata: Mapping[str, object]) -> str:
        if not isinstance(metadata, Mapping):
            return ""
        for key in ("title", "thought_sentence", "spoken_sentence", "situation_overview"):
            value = metadata.get(key)
            if isinstance(value, str):
                text = value.strip()
                if text:
                    return text[:280]
        payload = metadata.get("payload")
        if isinstance(payload, Mapping):
            try:
                preview = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            except (TypeError, ValueError):
                preview = str(payload)
            return preview[:280]
        if payload is not None:
            text = str(payload).strip()
            if text:
                return text[:280]
        return ""

    @staticmethod
    def _extract_labels(metadata: Mapping[str, object]) -> list[str]:
        labels: list[str] = []
        if not isinstance(metadata, Mapping):
            return labels

        def _append(value: object) -> None:
            if isinstance(value, str):
                text = value.strip()
                if text and text not in labels:
                    labels.append(text)

        for field in ("memory_tag", "memory_collection_text", "memory_collection_raw", "memory_collection_emoji"):
            _append(metadata.get(field))

        tags = metadata.get("tags")
        if isinstance(tags, (list, tuple, set)):
            for item in tags:
                _append(item)

        source_topics = metadata.get("source_topics")
        if isinstance(source_topics, (list, tuple, set)):
            for topic in source_topics:
                _append(topic)

        return labels


def main() -> None:
    rclpy.init()
    node = MemoryNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["MemoryNode", "main"]
