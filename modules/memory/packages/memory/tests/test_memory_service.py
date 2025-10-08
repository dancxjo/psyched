"""Behavioral tests for :mod:`memory.service`.

These tests follow a Given/When/Then structure to encourage
behavior-driven development as required by the module guidelines.
"""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
from typing import Any, Dict, Iterable, List, Mapping, MutableSequence, Optional, Sequence

import pytest

from memory.models import MemoryEventPayload, MemoryHeader
from memory.service import MemoryService, MemoryRecallResult
from memory.stores import VectorRecord


class FakeVectorStore:
    """A predictable in-memory vector store used for tests.

    The store records every batch that is flushed so assertions can
    inspect the exact payload sent to Qdrant in production.
    """

    def __init__(self) -> None:
        self.batches: MutableSequence[tuple[str, List[VectorRecord]]] = []
        self._next_results: Dict[str, List[MemoryRecallResult]] = defaultdict(list)

    def queue_search_result(
        self, collection: str, results: Iterable[MemoryRecallResult]
    ) -> None:
        self._next_results[collection].extend(results)

    # API expected by MemoryService
    def upsert_batch(self, collection: str, records: Sequence[VectorRecord]) -> None:
        self.batches.append((collection, list(records)))

    def search(
        self, collection: str, vector: Sequence[float], limit: int
    ) -> List[MemoryRecallResult]:
        del vector, limit  # deterministic stub
        results = self._next_results.get(collection, [])
        return list(results)


class FakeGraphStore:
    """A fake Neo4j store capturing how the memory graph is updated."""

    def __init__(self) -> None:
        self.memories: Dict[str, Mapping[str, Any]] = {}
        self.temporal_links: List[tuple[Optional[str], str, datetime]] = []
        self.frame_links: List[tuple[str, str]] = []
        self.vector_links: List[tuple[str, Optional[str]]] = []
        self.associations: List[tuple[str, str, str, Mapping[str, Any]]] = []

    def upsert_memory(self, memory: Mapping[str, Any]) -> str:
        self.memories[memory["memory_id"]] = dict(memory)
        return memory["memory_id"]

    def link_temporal(
        self, previous_id: Optional[str], memory_id: str, timestamp: datetime
    ) -> None:
        self.temporal_links.append((previous_id, memory_id, timestamp))

    def link_source(self, memory_id: str, frame_id: str) -> None:
        self.frame_links.append((memory_id, frame_id))

    def link_vector(self, memory_id: str, vector_id: Optional[str]) -> None:
        self.vector_links.append((memory_id, vector_id))

    def create_association(
        self,
        source_id: str,
        target_id: str,
        relation_type: str,
        properties: Optional[Mapping[str, Any]] = None,
    ) -> None:
        self.associations.append((source_id, target_id, relation_type, dict(properties or {})))

    def fetch_memory(self, memory_id: str) -> Mapping[str, Any]:
        return self.memories[memory_id]


@pytest.fixture(name="service")
def fixture_memory_service() -> MemoryService:
    vector_store = FakeVectorStore()
    graph_store = FakeGraphStore()
    return MemoryService(
        vector_store=vector_store,
        graph_store=graph_store,
        batch_size=2,
    )


def _build_event(
    *,
    stamp: datetime,
    frame_id: str = "camera/front",
    kind: str = "face",
    json_data: Mapping[str, Any] | None = None,
    embedding: Optional[Sequence[float]] = None,
) -> MemoryEventPayload:
    return MemoryEventPayload(
        header=MemoryHeader(stamp=stamp, frame_id=frame_id, source=node_name(kind)),
        kind=kind,
        json_data=json_data or {"emotion": "happy"},
        embedding=list(embedding) if embedding is not None else None,
    )


def node_name(kind: str) -> str:
    return f"sensor/{kind}"


def test_memorize_batches_vectors_until_threshold(service: MemoryService) -> None:
    """Given two events with embeddings, both vectors flush together."""

    event_a = _build_event(
        stamp=datetime(2024, 5, 10, 12, 30, tzinfo=timezone.utc),
        embedding=[0.1, 0.2],
    )
    event_b = _build_event(
        stamp=datetime(2024, 5, 10, 12, 30, 1, tzinfo=timezone.utc),
        embedding=[0.5, 0.9],
    )

    record_a = service.memorize(event_a, flush=False)
    record_b = service.memorize(event_b, flush=True)

    vector_batches = service.vector_store.batches  # type: ignore[attr-defined]
    assert len(vector_batches) == 1
    collection, records = vector_batches[0]
    assert collection == "face"
    assert {r.vector_id for r in records} == {record_a.vector_id, record_b.vector_id}
    assert service.graph_store.memories[record_a.memory_id]["kind"] == "face"  # type: ignore[attr-defined]
    assert service.graph_store.memories[record_b.memory_id]["kind"] == "face"  # type: ignore[attr-defined]
    temporal_links = service.graph_store.temporal_links  # type: ignore[attr-defined]
    assert len(temporal_links) == 2
    assert temporal_links[0][0] is None
    assert temporal_links[1][0] == temporal_links[0][1]
    assert len(service.graph_store.frame_links) == 2  # type: ignore[attr-defined]


def test_memorize_without_embedding_skips_vector_storage(service: MemoryService) -> None:
    """Given an event lacking an embedding, only the graph is updated."""

    event = _build_event(
        stamp=datetime(2024, 5, 10, 12, 32, tzinfo=timezone.utc),
        embedding=None,
        json_data={"note": "no vector"},
    )

    record = service.memorize(event)

    vector_batches = service.vector_store.batches  # type: ignore[attr-defined]
    assert vector_batches == []
    graph_memory = service.graph_store.memories[record.memory_id]  # type: ignore[attr-defined]
    assert graph_memory["metadata"]["note"] == "no vector"
    assert graph_memory["vector_id"] is None


def test_associate_records_relationship(service: MemoryService) -> None:
    """Associations are forwarded to the graph store with metadata."""

    event = _build_event(
        stamp=datetime(2024, 5, 10, 12, 35, tzinfo=timezone.utc),
        embedding=[0.3, 0.4],
    )
    record = service.memorize(event)

    service.associate(record.memory_id, record.memory_id, relation_type="REMEMBERS", properties={"confidence": 0.8})

    associations = service.graph_store.associations  # type: ignore[attr-defined]
    assert associations == [(record.memory_id, record.memory_id, "REMEMBERS", {"confidence": 0.8})]


def test_recall_enriches_vector_results_with_graph_context(service: MemoryService) -> None:
    """Vector results are hydrated with metadata fetched from the graph store."""

    event = _build_event(
        stamp=datetime(2024, 5, 10, 12, 40, tzinfo=timezone.utc),
        embedding=[0.7, 0.4],
    )
    record = service.memorize(event)

    fake_result = MemoryRecallResult(memory_id=record.memory_id, score=0.92, metadata={"foo": "bar"})
    service.vector_store.queue_search_result("face", [fake_result])  # type: ignore[attr-defined]

    results = service.recall(kind="face", embedding=[0.7, 0.4])

    assert len(results) == 1
    assert results[0].memory_id == record.memory_id
    assert results[0].metadata["foo"] == "bar"
    assert results[0].metadata["kind"] == "face"
