"""Protocols and helpers describing persistence back-ends.

The concrete integrations with Qdrant and Neo4j live outside the unit
tests, but these lightweight protocols provide a strongly typed contract
that callers can mock with ease.  Keeping the definitions separate from
:mod:`memory.service` avoids circular dependencies.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Mapping, Optional, Protocol, Sequence

from .models import MemoryRecallResult

__all__ = [
    "EmbeddingProviderProtocol",
    "GraphStoreProtocol",
    "Neo4jGraphStore",
    "QdrantVectorStore",
    "VectorRecord",
    "VectorStoreProtocol",
]


@dataclass(frozen=True)
class VectorRecord:
    """Container describing a vector that should be written to Qdrant."""

    vector_id: str
    values: Sequence[float]
    payload: Mapping[str, Any]


class VectorStoreProtocol(Protocol):
    """Minimal surface area required from a vector database client."""

    def upsert_batch(self, collection: str, records: Sequence[VectorRecord]) -> None:
        """Persist a batch of vector records into a collection."""

    def search(
        self, collection: str, vector: Sequence[float], limit: int
    ) -> Sequence[MemoryRecallResult]:
        """Return similarity matches for the provided vector."""


class EmbeddingProviderProtocol(Protocol):
    """Strategy for turning free-form text queries into embeddings."""

    def embed_text(self, text: str, *, kind: Optional[str] = None) -> Sequence[float]:
        """Return an embedding suitable for the configured vector store."""


class GraphStoreProtocol(Protocol):
    """Interface mirroring the subset of Neo4j behaviour the node needs."""

    def upsert_memory(self, memory: Mapping[str, Any]) -> str:
        """Create or update a memory node and return its identifier."""

    def link_temporal(
        self, previous_id: Optional[str], memory_id: str, timestamp: datetime
    ) -> None:
        """Connect sequential memories based on their timestamps."""

    def link_source(self, memory_id: str, frame_id: str) -> None:
        """Relate a memory to its originating frame."""

    def link_vector(self, memory_id: str, vector_id: Optional[str]) -> None:
        """Attach the graph node to its Qdrant embedding, when available."""

    def create_association(
        self,
        source_id: str,
        target_id: str,
        relation_type: str,
        properties: Optional[Mapping[str, Any]] = None,
    ) -> None:
        """Create a semantic edge between two memories."""

    def fetch_memory(self, memory_id: str) -> Mapping[str, Any]:
        """Return metadata describing a stored memory node."""


class QdrantVectorStore(VectorStoreProtocol):
    """Implementation of :class:`VectorStoreProtocol` using Qdrant."""

    def __init__(self, client: "QdrantClient") -> None:  # pragma: no cover - simple assignment
        self._client = client
        self._known_collections: set[str] = set()

    def upsert_batch(self, collection: str, records: Sequence[VectorRecord]) -> None:
        if not records:
            return
        dimension = len(records[0].values)
        self._ensure_collection(collection, dimension)
        point_structs = [
            self._point_struct(record.vector_id, record.values, record.payload) for record in records
        ]
        self._client.upsert(collection_name=collection, points=point_structs, wait=True)

    def search(
        self, collection: str, vector: Sequence[float], limit: int
    ) -> Sequence[MemoryRecallResult]:
        results = self._client.search(collection_name=collection, query_vector=vector, limit=limit)
        output: list[MemoryRecallResult] = []
        for result in results:
            payload = result.payload or {}
            memory_id = str(payload.get("memory_id"))
            if not memory_id:
                continue
            output.append(
                MemoryRecallResult(
                    memory_id=memory_id,
                    score=float(result.score or 0.0),
                    metadata=payload,
                )
            )
        return output

    # Internal helpers -------------------------------------------------
    def _ensure_collection(self, collection: str, dimension: int) -> None:
        if collection in self._known_collections:
            return
        try:
            self._client.get_collection(collection_name=collection)
        except Exception:  # pragma: no cover - network failure paths
            vector_params = self._vector_params(dimension)
            self._client.create_collection(collection_name=collection, vectors_config=vector_params)
        self._known_collections.add(collection)

    @staticmethod
    def _point_struct(vector_id: str, values: Sequence[float], payload: Mapping[str, Any]) -> "PointStruct":
        PointStruct = _lazy_import_point_struct()
        return PointStruct(id=vector_id, vector=list(values), payload=dict(payload))

    @staticmethod
    def _vector_params(dimension: int) -> "VectorParams":
        models = _lazy_import_qdrant_models()
        return models.VectorParams(size=dimension, distance=models.Distance.COSINE)


class Neo4jGraphStore(GraphStoreProtocol):
    """Implementation of :class:`GraphStoreProtocol` backed by Neo4j."""

    def __init__(self, uri: str, user: str, password: str) -> None:  # pragma: no cover - trivial wiring
        self._driver = _lazy_import_neo4j_driver()(uri, auth=(user, password))

    def close(self) -> None:  # pragma: no cover - used in production
        self._driver.close()

    def upsert_memory(self, memory: Mapping[str, Any]) -> str:
        label = self._kind_to_label(str(memory.get("kind", "Memory")))
        with self._driver.session() as session:
            session.execute_write(self._write_memory, label, dict(memory))
        return str(memory["memory_id"])

    def link_temporal(
        self, previous_id: Optional[str], memory_id: str, timestamp: datetime
    ) -> None:
        with self._driver.session() as session:
            session.execute_write(self._write_temporal_links, previous_id, memory_id, timestamp.isoformat())

    def link_source(self, memory_id: str, frame_id: str) -> None:
        if not frame_id:
            return
        with self._driver.session() as session:
            session.execute_write(self._write_source_link, memory_id, frame_id)

    def link_vector(self, memory_id: str, vector_id: Optional[str]) -> None:
        if not vector_id:
            return
        with self._driver.session() as session:
            session.execute_write(self._write_vector_link, memory_id, vector_id)

    def create_association(
        self,
        source_id: str,
        target_id: str,
        relation_type: str,
        properties: Optional[Mapping[str, Any]] = None,
    ) -> None:
        rel_type = self._relationship_type(relation_type)
        with self._driver.session() as session:
            session.execute_write(self._write_association, source_id, target_id, rel_type, dict(properties or {}))

    def fetch_memory(self, memory_id: str) -> Mapping[str, Any]:
        with self._driver.session() as session:
            record = session.execute_read(self._read_memory, memory_id)
            return record or {}

    # Neo4j transactions -----------------------------------------------
    @staticmethod
    def _write_memory(tx, label: str, properties: Mapping[str, Any]) -> None:  # pragma: no cover - exercised via driver
        query = f"""
        MERGE (m:Memory:{label} {{memory_id: $memory_id}})
        SET m += $properties
        """
        tx.run(query, memory_id=properties["memory_id"], properties=dict(properties))

    @staticmethod
    def _write_temporal_links(tx, previous_id: Optional[str], memory_id: str, timestamp: str) -> None:
        tx.run(
            """
            MATCH (current:Memory {memory_id: $memory_id})
            SET current.timestamp = $timestamp
            WITH current
            OPTIONAL MATCH (previous:Memory {memory_id: $previous_id})
            WITH previous, current
            WHERE previous IS NOT NULL
            MERGE (previous)-[:NEXT]->(current)
            MERGE (current)-[:PREVIOUS]->(previous)
            """,
            memory_id=memory_id,
            previous_id=previous_id,
            timestamp=timestamp,
        )

    @staticmethod
    def _write_source_link(tx, memory_id: str, frame_id: str) -> None:
        tx.run(
            """
            MATCH (memory:Memory {memory_id: $memory_id})
            MERGE (frame:Frame {frame_id: $frame_id})
            MERGE (memory)-[:FROM_FRAME]->(frame)
            """,
            memory_id=memory_id,
            frame_id=frame_id,
        )

    @staticmethod
    def _write_vector_link(tx, memory_id: str, vector_id: str) -> None:
        tx.run(
            """
            MATCH (memory:Memory {memory_id: $memory_id})
            MERGE (embedding:Embedding {vector_id: $vector_id})
            MERGE (memory)-[:EMBEDDED_AS]->(embedding)
            """,
            memory_id=memory_id,
            vector_id=vector_id,
        )

    @staticmethod
    def _write_association(
        tx,
        source_id: str,
        target_id: str,
        relation_type: str,
        properties: Mapping[str, Any],
    ) -> None:
        query = f"""
        MATCH (source:Memory {{memory_id: $source_id}})
        MATCH (target:Memory {{memory_id: $target_id}})
        MERGE (source)-[rel:{relation_type}]->(target)
        SET rel += $properties
        """
        tx.run(query, source_id=source_id, target_id=target_id, properties=dict(properties))

    @staticmethod
    def _read_memory(tx, memory_id: str) -> Optional[Mapping[str, Any]]:
        result = tx.run("MATCH (memory:Memory {memory_id: $memory_id}) RETURN memory", memory_id=memory_id)
        record = result.single()
        if record is None:
            return None
        node = record["memory"]
        return dict(node)

    @staticmethod
    def _kind_to_label(kind: str) -> str:
        cleaned = ''.join(ch if ch.isalnum() else '_' for ch in kind.strip()) or "Memory"
        if cleaned[0].isdigit():
            cleaned = f"_{cleaned}"
        return cleaned

    @staticmethod
    def _relationship_type(relation_type: str) -> str:
        cleaned = ''.join(ch if ch.isalnum() else '_' for ch in relation_type.upper() or "ASSOCIATED_WITH")
        if cleaned[0].isdigit():
            cleaned = f"R_{cleaned}"
        return cleaned


def _lazy_import_point_struct():
    try:  # pragma: no cover - import guard
        from qdrant_client.http import models as qmodels
    except ImportError as exc:  # pragma: no cover - executed when dependency missing
        raise RuntimeError("qdrant-client is required for QdrantVectorStore") from exc
    return qmodels.PointStruct


def _lazy_import_qdrant_models():
    try:  # pragma: no cover - import guard
        from qdrant_client.http import models as qmodels
    except ImportError as exc:  # pragma: no cover - executed when dependency missing
        raise RuntimeError("qdrant-client is required for QdrantVectorStore") from exc
    return qmodels


def _lazy_import_neo4j_driver():
    try:  # pragma: no cover - import guard
        from neo4j import GraphDatabase
    except ImportError as exc:  # pragma: no cover - executed when dependency missing
        raise RuntimeError("neo4j is required for Neo4jGraphStore") from exc
    return GraphDatabase.driver
