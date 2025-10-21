from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Sequence

from .models import FeelingIntentData, SensationRecord


@dataclass(slots=True)
class MemoryBatch:
    """Container describing memory events and associations destined for ROS services."""

    feeling_id: str
    events: List["MemoryEventDraft"] = field(default_factory=list)
    associations: List["MemoryAssociationDraft"] = field(default_factory=list)


@dataclass(slots=True)
class MemoryEventDraft:
    """Draft payload for /memory/memorize."""

    tag: str
    kind: str
    frame_id: str
    source: str
    metadata: Dict[str, Any]
    embedding: List[float] | None = None


@dataclass(slots=True)
class MemoryAssociationDraft:
    """Draft payload describing a /memory/associate call."""

    source_tag: str
    target_tag: str
    relation_type: str
    properties: Dict[str, Any] = field(default_factory=dict)


def _safe_json_loads(payload: str) -> Dict[str, Any]:
    if not payload:
        return {}
    try:
        value = json.loads(payload)
    except json.JSONDecodeError:
        return {"raw": payload}
    return value if isinstance(value, dict) else {"value": value}


def _default_collection(value: str, fallback: str) -> str:
    value = value.strip()
    return value or fallback


def prepare_memory_batch(
    *,
    feeling: FeelingIntentData,
    sensations: Sequence[SensationRecord],
    source_topics: Sequence[str],
    timestamp: datetime,
    feeling_id: str,
) -> MemoryBatch:
    """Build the set of memory events and associations for the ROS memory module."""

    events: List[MemoryEventDraft] = []
    associations: List[MemoryAssociationDraft] = []
    source_topics = sorted(set(source_topics))
    timestamp_iso = timestamp.isoformat()

    text_collection = _default_collection(feeling.memory_collection_text, "thoughts")

    feeling_tag = "feeling"
    feeling_metadata: Dict[str, Any] = {
        "kind": "feeling_intent",
        "feeling_id": feeling_id,
        "episode_id": feeling.episode_id,
        "situation_id": feeling.situation_id,
        "source_topics": source_topics,
        "timestamp": timestamp_iso,
        "topic": "pilot/intent",
        "situation_overview": feeling.situation_overview,
        "attitude_emoji": feeling.attitude_emoji,
        "thought_sentence": feeling.thought_sentence,
        "spoken_sentence": feeling.spoken_sentence,
        "goals": list(feeling.goals),
        "mood_delta": feeling.mood_delta,
        "memory_collection_raw": feeling.memory_collection_raw,
        "memory_collection_text": feeling.memory_collection_text,
        "memory_collection_emoji": feeling.memory_collection_emoji,
        "command_script": feeling.command_script,
    }
    events.append(
        MemoryEventDraft(
            tag=feeling_tag,
            kind=text_collection,
            frame_id="pilot/intent",
            source="pilot",
            metadata=feeling_metadata,
            embedding=None,
        )
    )

    for record in sensations:
        if not record.vector:
            continue
        collection = _default_collection(record.collection_hint, feeling.memory_collection_raw or record.kind or "raw")
        tag = f"sensation:{record.topic}:{len(events)}"
        metadata = {
            "kind": record.kind or "sensation",
            "feeling_id": feeling_id,
            "episode_id": feeling.episode_id,
            "situation_id": feeling.situation_id,
            "source_topics": source_topics,
            "timestamp": timestamp_iso,
            "topic": record.topic,
            "payload": _safe_json_loads(record.json_payload),
            "collection_hint": record.collection_hint,
        }
        events.append(
            MemoryEventDraft(
                tag=tag,
                kind=collection,
                frame_id=record.topic or "sensation",
                source=record.topic or "sensation",
                metadata=metadata,
                embedding=list(record.vector),
            )
        )
        associations.append(
            MemoryAssociationDraft(
                source_tag=tag,
                target_tag=feeling_tag,
                relation_type="INFLUENCED",
                properties={
                    "feeling_id": feeling_id,
                    "topic": record.topic,
                    "kind": record.kind or "sensation",
                },
            )
        )

    return MemoryBatch(feeling_id=feeling_id, events=events, associations=associations)
