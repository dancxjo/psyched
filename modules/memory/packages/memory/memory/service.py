"""High level orchestration for the memory module.

The :class:`MemoryService` coordinates vector storage in Qdrant with the
semantic graph maintained in Neo4j.  It exposes a friendly API that ROS
services, HTTP handlers, or CLI tools can depend on without needing to
know about database details.
"""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime
from typing import Dict, List, Mapping, MutableMapping, Optional, Sequence
from uuid import uuid4

from .models import MemoryEventPayload, MemoryRecallResult, MemoryRecord
from .stores import (
    EmbeddingProviderProtocol,
    GraphStoreProtocol,
    VectorRecord,
    VectorStoreProtocol,
)

__all__ = ["MemoryService"]


class MemoryService:
    """Coordinate persistence and recall across Qdrant and Neo4j.

    Parameters
    ----------
    vector_store:
        Object responsible for writing and searching embeddings.
    graph_store:
        Abstraction over the Neo4j driver in charge of storing metadata
        and relationships.
    embedding_provider:
        Optional helper capable of turning free-form text into an
        embedding.  It is only required when :meth:`recall` is called
        with a ``text`` query.
    batch_size:
        Number of embeddings that should be accumulated before they are
        flushed to the vector store.  A value of ``1`` keeps the behaviour
        synchronous while larger values enable simple batching.

    Examples
    --------
    >>> from datetime import datetime, timezone
    >>> from memory.models import MemoryEventPayload, MemoryHeader
    >>> from memory.stores import VectorRecord
    >>> class DummyVectorStore:
    ...     def __init__(self):
    ...         self.upserts = []
    ...     def upsert_batch(self, collection, records):
    ...         self.upserts.append((collection, list(records)))
    ...     def search(self, collection, vector, limit):
    ...         return []
    >>> class DummyGraphStore:
    ...     def __init__(self):
    ...         self.nodes = {}
    ...     def upsert_memory(self, memory):
    ...         self.nodes[memory["memory_id"]] = memory
    ...         return memory["memory_id"]
    ...     def link_temporal(self, previous_id, memory_id, timestamp):
    ...         pass
    ...     def link_source(self, memory_id, frame_id):
    ...         pass
    ...     def link_vector(self, memory_id, vector_id):
    ...         pass
    ...     def create_association(self, source_id, target_id, relation_type, properties=None):
    ...         pass
    ...     def fetch_memory(self, memory_id):
    ...         return self.nodes[memory_id]
    >>> service = MemoryService(DummyVectorStore(), DummyGraphStore())
    >>> payload = MemoryEventPayload(
    ...     header=MemoryHeader(datetime.now(tz=timezone.utc), "frame", "sensor/face"),
    ...     kind="face",
    ...     json_data={"emotion": "curious"},
    ...     embedding=[0.1, 0.2],
    ... )
    >>> record = service.memorize(payload)
    >>> record.kind
    'face'
    """

    def __init__(
        self,
        vector_store: VectorStoreProtocol,
        graph_store: GraphStoreProtocol,
        embedding_provider: Optional[EmbeddingProviderProtocol] = None,
        *,
        batch_size: int = 1,
    ) -> None:
        if batch_size < 1:
            raise ValueError("batch_size must be at least 1")
        self.vector_store = vector_store
        self.graph_store = graph_store
        self._embedding_provider = embedding_provider
        self._batch_size = batch_size
        self._vector_queue: MutableMapping[str, List[VectorRecord]] = defaultdict(list)
        self._previous_by_frame: Dict[str, str] = {}

    # ------------------------------------------------------------------
    # Memorisation
    # ------------------------------------------------------------------
    def memorize(self, event: MemoryEventPayload, *, flush: bool = True) -> MemoryRecord:
        """Persist a new memory event.

        The embedding is queued for batch insertion.  The Neo4j node is
        written immediately so that callers can obtain a stable memory
        identifier regardless of batching behaviour.
        """

        memory_id = self._generate_id()
        timestamp = event.header.stamp
        metadata = dict(event.metadata)
        vector_id: Optional[str] = None

        if event.embedding is not None:
            vector_id = self._queue_vector(event.kind, memory_id, event.embedding, metadata, timestamp, event)
            if flush or self._pending_count(event.kind) >= self._batch_size:
                self.flush_vectors()

        graph_payload = self._build_graph_payload(
            memory_id=memory_id,
            vector_id=vector_id,
            event=event,
            timestamp=timestamp,
            metadata=metadata,
        )
        self.graph_store.upsert_memory(graph_payload)
        previous_id = self._previous_by_frame.get(event.header.frame_id)
        self.graph_store.link_temporal(previous_id, memory_id, timestamp)
        self.graph_store.link_source(memory_id, event.header.frame_id)
        self.graph_store.link_vector(memory_id, vector_id)
        self._previous_by_frame[event.header.frame_id] = memory_id

        return MemoryRecord(
            memory_id=memory_id,
            vector_id=vector_id,
            kind=event.kind,
            timestamp=timestamp,
            frame_id=event.header.frame_id,
            source=event.header.source,
            metadata=metadata,
        )

    def flush_vectors(self) -> None:
        """Persist all queued vectors to the vector store."""

        for collection, records in list(self._vector_queue.items()):
            if not records:
                continue
            self.vector_store.upsert_batch(collection, records)
            records.clear()
        self._vector_queue.clear()

    # ------------------------------------------------------------------
    # Associations
    # ------------------------------------------------------------------
    def associate(
        self,
        source_id: str,
        target_id: str,
        *,
        relation_type: str = "ASSOCIATED_WITH",
        properties: Optional[Mapping[str, object]] = None,
    ) -> None:
        """Create an explicit association between two memories."""

        self.graph_store.create_association(source_id, target_id, relation_type, properties)

    # ------------------------------------------------------------------
    # Recall
    # ------------------------------------------------------------------
    def recall(
        self,
        *,
        kind: str,
        embedding: Optional[Sequence[float]] = None,
        text: Optional[str] = None,
        limit: int = 5,
    ) -> List[MemoryRecallResult]:
        """Return memories similar to the provided query."""

        query_vector = self._resolve_query_vector(kind=kind, embedding=embedding, text=text)
        results = self.vector_store.search(kind, query_vector, limit)
        enriched: List[MemoryRecallResult] = []
        for result in results:
            graph_metadata = self.graph_store.fetch_memory(result.memory_id)
            merged_metadata = {**graph_metadata.get("metadata", {}), **dict(result.metadata)}
            merged_metadata.update(
                {
                    "kind": graph_metadata.get("kind", kind),
                    "frame_id": graph_metadata.get("frame_id"),
                    "source": graph_metadata.get("source"),
                }
            )
            enriched.append(
                MemoryRecallResult(
                    memory_id=result.memory_id,
                    score=result.score,
                    metadata=merged_metadata,
                )
            )
        return enriched

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _queue_vector(
        self,
        collection: str,
        memory_id: str,
        embedding: Sequence[float],
        metadata: Mapping[str, object],
        timestamp: datetime,
        event: MemoryEventPayload,
    ) -> str:
        vector_id = self._generate_id()
        payload = {
            "memory_id": memory_id,
            "kind": collection,
            "timestamp": timestamp.isoformat(),
            "frame_id": event.header.frame_id,
            "source": event.header.source,
            "metadata": dict(metadata),
        }
        record = VectorRecord(vector_id=vector_id, values=list(embedding), payload=payload)
        self._vector_queue[collection].append(record)
        return vector_id

    def _pending_count(self, collection: str) -> int:
        return len(self._vector_queue.get(collection, ()))

    def _build_graph_payload(
        self,
        *,
        memory_id: str,
        vector_id: Optional[str],
        event: MemoryEventPayload,
        timestamp: datetime,
        metadata: Mapping[str, object],
    ) -> Dict[str, object]:
        return {
            "memory_id": memory_id,
            "vector_id": vector_id,
            "kind": event.kind,
            "timestamp": timestamp.isoformat(),
            "frame_id": event.header.frame_id,
            "source": event.header.source,
            "metadata": dict(metadata),
        }

    def _resolve_query_vector(
        self,
        *,
        kind: str,
        embedding: Optional[Sequence[float]],
        text: Optional[str],
    ) -> Sequence[float]:
        if embedding is not None:
            return embedding
        if text is not None:
            if self._embedding_provider is None:
                raise ValueError("text recall requested but no embedding provider is configured")
            return self._embedding_provider.embed_text(text, kind=kind)
        raise ValueError("either embedding or text must be provided")

    @staticmethod
    def _generate_id() -> str:
        return uuid4().hex
