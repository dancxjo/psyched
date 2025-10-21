from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, Iterable, List, Sequence

from .models import FeelingIntentData, SensationRecord


@dataclass(slots=True)
class MemoryBatch:
    """Container for a combined rememberd vector + graph mutation request."""

    feeling_id: str
    vectors: List[Dict[str, Any]] = field(default_factory=list)
    graph_mutations: List[Dict[str, Any]] = field(default_factory=list)


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


def _sensation_ids(records: Iterable[SensationRecord]) -> List[str]:
    ids: List[str] = []
    for idx, record in enumerate(records):
        sid = record.sensation_id()
        if not sid:
            sid = f"{record.topic}:{idx}"
        ids.append(sid)
    return ids


def prepare_memory_batch(
    *,
    feeling: FeelingIntentData,
    sensations: Sequence[SensationRecord],
    source_topics: Sequence[str],
    timestamp: datetime,
    feeling_id: str,
) -> MemoryBatch:
    """Build the consolidated memory payload for rememberd."""

    vectors: List[Dict[str, Any]] = []
    source_topics = sorted(set(source_topics))
    timestamp_iso = timestamp.isoformat()

    emoji_collection = _default_collection(feeling.memory_collection_emoji, "emotions")
    text_collection = _default_collection(feeling.memory_collection_text, "thoughts")

    if feeling.situation_overview:
        vectors.append(
            {
                "collection": text_collection,
                "text": feeling.situation_overview,
                "payload": {
                    "kind": "situation_overview",
                    "feeling_id": feeling_id,
                    "episode_id": feeling.episode_id,
                    "situation_id": feeling.situation_id,
                    "source_topics": source_topics,
                    "timestamp": timestamp_iso,
                },
            }
        )

    if feeling.attitude_emoji:
        vectors.append(
            {
                "collection": emoji_collection,
                "text": feeling.attitude_emoji,
                "payload": {
                    "kind": "emoji_attitude",
                    "feeling_id": feeling_id,
                    "episode_id": feeling.episode_id,
                    "situation_id": feeling.situation_id,
                    "source_topics": source_topics,
                    "timestamp": timestamp_iso,
                },
            }
        )

    if feeling.thought_sentence:
        vectors.append(
            {
                "collection": text_collection,
                "text": feeling.thought_sentence,
                "payload": {
                    "kind": "thought",
                    "feeling_id": feeling_id,
                    "episode_id": feeling.episode_id,
                    "situation_id": feeling.situation_id,
                    "source_topics": source_topics,
                    "timestamp": timestamp_iso,
                },
            }
        )

    for record in sensations:
        if not record.vector:
            continue
        collection = _default_collection(record.collection_hint, feeling.memory_collection_raw or record.kind or "raw")
        vectors.append(
            {
                "collection": collection,
                "vector": list(record.vector),
                "payload": {
                    "kind": record.kind or "sensation",
                    "sensation_id": record.sensation_id(),
                    "topic": record.topic,
                    "metadata": _safe_json_loads(record.json_payload),
                    "feeling_id": feeling_id,
                    "episode_id": feeling.episode_id,
                    "situation_id": feeling.situation_id,
                    "source_topics": source_topics,
                    "timestamp": timestamp_iso,
                },
            }
        )

    sensation_ids = _sensation_ids(sensations)
    graph_mutations = [
        {
            "cypher": (
                "MERGE (e:Episodic {id:$episode_id})\n"
                "MERGE (s:Situation {id:$situation_id})\n"
                "MERGE (f:FeelingIntent {id:$feeling_id})\n"
                "SET f.situation_overview=$situation_overview, f.attitude_emoji=$attitude_emoji, f.thought=$thought_sentence, "
                "f.spoken=$spoken_sentence, f.goals=$goals, f.mood_delta=$mood_delta, "
                "f.timestamp=$timestamp, f.source_topics=$source_topics, "
                "f.emoji_collection=$emoji_collection, f.text_collection=$text_collection\n"
                "MERGE (e)-[:HAS_SITUATION]->(s)\n"
                "MERGE (s)-[:EVOKED]->(f)\n"
                "WITH f\n"
                "UNWIND $sensation_ids AS sid\n"
                "MERGE (z:Sensation {id:sid})\n"
                "MERGE (z)-[:EVALUATED_AS]->(f)"
            ),
            "params": {
                "episode_id": feeling.episode_id,
                "situation_id": feeling.situation_id,
                "feeling_id": feeling_id,
                "situation_overview": feeling.situation_overview,
                "attitude_emoji": feeling.attitude_emoji,
                "thought_sentence": feeling.thought_sentence,
                "spoken_sentence": feeling.spoken_sentence,
                "goals": feeling.goals,
                "mood_delta": feeling.mood_delta,
                "timestamp": timestamp_iso,
                "source_topics": source_topics,
                "emoji_collection": emoji_collection,
                "text_collection": text_collection,
                "sensation_ids": sensation_ids,
            },
        }
    ]

    return MemoryBatch(feeling_id=feeling_id, vectors=vectors, graph_mutations=graph_mutations)
