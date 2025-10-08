"""ROS 2 node exposing the memory services described in the spec."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Optional, Sequence

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from memory_interfaces.msg import MemoryEvent as MemoryEventMsg
from memory_interfaces.msg import MemoryRecall as MemoryRecallMsg
from memory_interfaces.srv import Associate as AssociateSrv
from memory_interfaces.srv import Memorize as MemorizeSrv
from memory_interfaces.srv import Recall as RecallSrv

from .models import MemoryEventPayload, MemoryHeader
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
        try:
            return super().destroy_node()
        finally:
            for closeable in self._closeables:
                close = getattr(closeable, "close", None)
                if callable(close):
                    close()
            self._closeables.clear()
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
