"""High level orchestration for the memory module.

The :class:`MemoryService` coordinates vector storage in Qdrant with the
semantic graph maintained in Neo4j.  It exposes a friendly API that ROS
services, HTTP handlers, or CLI tools can depend on without needing to
know about database details.
"""

from __future__ import annotations

from collections import defaultdict
import math
import re
from datetime import datetime
from typing import Any, Dict, List, Mapping, MutableMapping, Optional, Sequence
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
        identity_links = self._prepare_identity_links(metadata)
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
        for identity_id, properties in identity_links:
            self.graph_store.link_identity(memory_id, identity_id, properties)
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
    # Identity tagging
    # ------------------------------------------------------------------
    def tag_identity(
        self,
        memory_id: str,
        label: str,
        *,
        identity_id: Optional[str] = None,
        aliases: Optional[Sequence[str]] = None,
    ) -> Dict[str, Any]:
        """Attach a human-friendly identity label to an existing memory."""

        memory_key = self._normalise_text(memory_id)
        if not memory_key:
            raise ValueError("memory_id must be a non-empty string")
        label_text = self._normalise_text(label)
        if not label_text:
            raise ValueError("label must be a non-empty string")

        try:
            record = self.graph_store.fetch_memory(memory_key)
        except KeyError as exc:  # pragma: no cover - defensive guard for fake stores
            raise ValueError(f"Memory '{memory_key}' not found") from exc
        if not record:
            raise ValueError(f"Memory '{memory_key}' not found")

        metadata_raw = record.get("metadata") if isinstance(record, Mapping) else {}
        metadata: Dict[str, Any] = dict(metadata_raw) if isinstance(metadata_raw, Mapping) else {}

        existing_identity = metadata.get("identity") if isinstance(metadata.get("identity"), Mapping) else None
        candidate_id = self._normalise_text(identity_id) if identity_id is not None else ""
        if not candidate_id and existing_identity:
            candidate_id = self._normalise_text(
                existing_identity.get("id"),
                existing_identity.get("identity_id"),
                existing_identity.get("uuid"),
            )
        identity_key = candidate_id or self._generate_identity_id(label_text, memory_key)

        alias_source: List[str] = []
        if existing_identity:
            alias_source.extend(self._normalise_text_list(existing_identity.get("aliases")))
        alias_source.extend(self._normalise_text_list(aliases))
        alias_source = self._extend_unique(alias_source, [label_text])

        memory_ids = []
        if existing_identity:
            memory_ids = self._normalise_text_list(existing_identity.get("memory_ids"))
        memory_ids = self._extend_unique(memory_ids, [memory_key])

        identity_summary: Dict[str, Any] = {"id": identity_key, "name": label_text}
        if alias_source:
            identity_summary["aliases"] = alias_source
        if memory_ids:
            identity_summary["memory_ids"] = memory_ids
        metadata["identity"] = identity_summary

        identities_field = metadata.get("identities")
        identities: List[Dict[str, Any]] = []
        if isinstance(identities_field, Sequence):
            for entry in identities_field:
                if isinstance(entry, Mapping):
                    summary = dict(entry)
                    key = self._normalise_text(
                        summary.get("id"),
                        summary.get("identity_id"),
                        summary.get("uuid"),
                    )
                    if key and key != identity_key:
                        identities.append(summary)
        identities.insert(0, identity_summary)
        metadata["identities"] = identities

        metadata["name"] = label_text
        tags_field = metadata.get("tags")
        if isinstance(tags_field, list):
            if label_text not in tags_field:
                tags_field.append(label_text)
        elif isinstance(tags_field, Sequence) and not isinstance(tags_field, (str, bytes)):
            tags_list = list(tags_field)
            if label_text not in tags_list:
                tags_list.append(label_text)
            metadata["tags"] = tags_list
        else:
            metadata["tags"] = [label_text]

        updated_payload = {
            "memory_id": record.get("memory_id", memory_key),
            "vector_id": record.get("vector_id"),
            "kind": record.get("kind", "Memory"),
            "timestamp": record.get("timestamp"),
            "frame_id": record.get("frame_id", ""),
            "source": record.get("source", ""),
            "metadata": metadata,
        }
        self.graph_store.upsert_memory(updated_payload)

        identity_properties: Dict[str, Any] = {"name": label_text}
        if alias_source:
            identity_properties["aliases"] = alias_source
        if memory_ids:
            identity_properties["memory_ids"] = memory_ids
        self.graph_store.link_identity(memory_key, identity_key, identity_properties)

        return {
            "memory_id": memory_key,
            "identity": identity_summary,
        }

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
            identities = graph_metadata.get("identities")
            if isinstance(identities, Sequence):
                identity_list = [
                    dict(identity)
                    for identity in identities
                    if isinstance(identity, Mapping)
                ]
                if identity_list:
                    merged_metadata.setdefault("identities", identity_list)
                    merged_metadata.setdefault("identity", identity_list[0])
                    primary_name = merged_metadata.get("identity", {}).get("name")
                    if primary_name and "name" not in merged_metadata:
                        merged_metadata["name"] = primary_name
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

    def _prepare_identity_links(
        self,
        metadata: MutableMapping[str, Any],
    ) -> List[tuple[str, Dict[str, Any]]]:
        identities: List[Mapping[str, Any]] = []
        primary = metadata.get("identity")
        if isinstance(primary, Mapping):
            identities.append(primary)
        candidates = metadata.get("identities")
        if isinstance(candidates, Sequence):
            for entry in candidates:
                if isinstance(entry, Mapping):
                    identities.append(entry)
        consolidated: Dict[str, Dict[str, Any]] = {}
        for candidate in identities:
            normalised = self._normalise_identity_candidate(candidate)
            if normalised is None:
                continue
            identity_id, graph_props, display_identity = normalised
            bucket = consolidated.get(identity_id)
            if bucket is None:
                consolidated[identity_id] = {
                    "props": dict(graph_props),
                    "display": dict(display_identity),
                }
                continue
            self._merge_identity_lists(bucket["props"], graph_props, "aliases")
            self._merge_identity_lists(bucket["props"], graph_props, "signatures")
            self._merge_identity_lists(bucket["props"], graph_props, "memory_ids")
            self._merge_identity_lists(bucket["display"], display_identity, "aliases")
            self._merge_identity_lists(bucket["display"], display_identity, "signatures")
            self._merge_identity_lists(bucket["display"], display_identity, "memory_ids")
            if display_identity.get("name"):
                bucket["display"].setdefault("name", display_identity["name"])
            candidate_confidence = self._coerce_float(display_identity.get("confidence"))
            if candidate_confidence is not None:
                existing_confidence = self._coerce_float(bucket["display"].get("confidence"))
                if existing_confidence is None or candidate_confidence > existing_confidence:
                    bucket["display"]["confidence"] = candidate_confidence
        cleaned: List[Dict[str, Any]] = []
        prepared: List[tuple[str, Dict[str, Any]]] = []
        for identity_id, bundle in consolidated.items():
            props = {
                key: value for key, value in bundle["props"].items() if value not in (None, [], {})
            }
            display = {
                key: value for key, value in bundle["display"].items() if value not in (None, [], {})
            }
            display.setdefault("id", identity_id)
            if props.get("signatures"):
                props["signatures"] = list(dict.fromkeys(props["signatures"]))
            if props.get("aliases"):
                props["aliases"] = list(dict.fromkeys(props["aliases"]))
            if props.get("memory_ids"):
                props["memory_ids"] = list(dict.fromkeys(props["memory_ids"]))
            cleaned.append(display)
            prepared.append((identity_id, props))
        if cleaned:
            metadata["identities"] = cleaned
            metadata["identity"] = cleaned[0]
            primary_name = cleaned[0].get("name")
            if isinstance(primary_name, str):
                name_text = primary_name.strip()
                if name_text:
                    metadata.setdefault("name", name_text)
                    tags_value = metadata.setdefault("tags", [])
                    if isinstance(tags_value, list) and name_text not in tags_value:
                        tags_value.append(name_text)
        else:
            metadata.pop("identity", None)
            metadata.pop("identities", None)
        return prepared

    def _normalise_identity_candidate(
        self,
        candidate: Mapping[str, Any],
    ) -> Optional[tuple[str, Dict[str, Any], Dict[str, Any]]]:
        identity_id = self._normalise_text(
            candidate.get("id"),
            candidate.get("identity_id"),
            candidate.get("uuid"),
            candidate.get("name"),
        )
        if not identity_id:
            return None
        name = self._normalise_text(candidate.get("name"))
        aliases = self._normalise_text_list(candidate.get("aliases"))
        labels = self._normalise_text_list(candidate.get("labels"))
        if labels:
            aliases = self._extend_unique(aliases, labels)
        signatures = self._normalise_text_list(candidate.get("signatures"))
        signature_history = self._normalise_text_list(candidate.get("signature_history"))
        if signature_history:
            signatures = self._extend_unique(signatures, signature_history)
        signature_hint = self._normalise_text(candidate.get("signature"))
        if signature_hint:
            signatures = self._extend_unique(signatures, [signature_hint])
        memory_ids = self._normalise_text_list(candidate.get("memory_ids"))
        memory_hint = self._normalise_text(candidate.get("memory_id"))
        if memory_hint:
            memory_ids = self._extend_unique(memory_ids, [memory_hint])
        confidence = self._coerce_float(candidate.get("confidence"))
        if confidence is None:
            confidence = self._coerce_float(candidate.get("score"))
        display_identity: Dict[str, Any] = {"id": identity_id}
        if name:
            display_identity["name"] = name
        if aliases:
            display_identity["aliases"] = aliases
        if signatures:
            display_identity["signatures"] = signatures
        if memory_ids:
            display_identity["memory_ids"] = memory_ids
        if confidence is not None:
            display_identity["confidence"] = confidence
        graph_props: Dict[str, Any] = {}
        if name:
            graph_props["name"] = name
        if aliases:
            graph_props["aliases"] = aliases
        if signatures:
            graph_props["signatures"] = signatures
        if memory_ids:
            graph_props["memory_ids"] = memory_ids
        if confidence is not None:
            graph_props["confidence"] = confidence
        return identity_id, graph_props, display_identity

    @staticmethod
    def _normalise_text(*values: object) -> str:
        for value in values:
            if isinstance(value, str):
                text = value.strip()
                if text:
                    return text
        return ""

    def _normalise_text_list(self, value: object) -> List[str]:
        items: List[str] = []
        if value is None:
            return items
        if isinstance(value, str):
            text = value.strip()
            if text:
                items.append(text)
            return items
        if isinstance(value, Sequence) and not isinstance(value, (bytes, bytearray, str)):
            for element in value:
                text = self._normalise_text(element)
                if text and text not in items:
                    items.append(text)
        return items

    @staticmethod
    def _extend_unique(base: List[str], additional: Sequence[str]) -> List[str]:
        for item in additional:
            if item not in base:
                base.append(item)
        return base

    @staticmethod
    def _slugify(value: str) -> str:
        cleaned = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
        return cleaned

    def _generate_identity_id(self, label: str, memory_id: str) -> str:
        slug_parts: List[str] = []
        label_slug = self._slugify(label)
        if label_slug:
            slug_parts.append(label_slug)
        memory_slug = self._slugify(memory_id)
        if memory_slug and memory_slug not in slug_parts:
            slug_parts.append(memory_slug)
        slug = "-".join(slug_parts) or uuid4().hex[:8]
        if not slug.startswith("person:"):
            slug = f"person:{slug}"
        return slug

    @staticmethod
    def _merge_identity_lists(
        target: MutableMapping[str, Any],
        source: Mapping[str, Any],
        key: str,
    ) -> None:
        incoming = source.get(key)
        if not isinstance(incoming, list):
            return
        existing = target.get(key)
        values: List[Any] = list(existing) if isinstance(existing, list) else []
        for item in incoming:
            if item not in values:
                values.append(item)
        if values:
            target[key] = values
        elif key in target:
            target.pop(key, None)

    @staticmethod
    def _coerce_float(value: object) -> Optional[float]:
        try:
            numeric = float(value)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return None
        if math.isnan(numeric) or math.isinf(numeric):
            return None
        return numeric

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
