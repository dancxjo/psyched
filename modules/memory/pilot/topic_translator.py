"""Prompt helpers surfacing the memory module to the pilot loop."""

from __future__ import annotations

import json
from typing import Any, Dict, Mapping


def _load_payload(payload: Any) -> Mapping[str, Any] | None:
    if isinstance(payload, Mapping):
        data = payload.get("data")
        if isinstance(data, Mapping):
            return data
        if isinstance(data, str):
            text = data.strip()
            if not text:
                return None
            try:
                parsed = json.loads(text)
            except json.JSONDecodeError:
                return None
            if isinstance(parsed, Mapping):
                return parsed
    elif isinstance(payload, str):
        text = payload.strip()
        if not text:
            return None
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return None
        if isinstance(parsed, Mapping):
            return parsed
    return None


def summarise_pilot_feed(payload: Any) -> str:
    """Summarise JSON records streamed on ``/memory/pilot_feed``.

    Examples
    --------
    >>> summarise_pilot_feed({"data": '{"kind": "thought", "summary": "heard hello"}'})
    'Memory update (thought): "heard hello".'
    >>> summarise_pilot_feed({})
    'Memory update channel is quiet.'
    """

    record = _load_payload(payload)
    if not record:
        return "Memory update channel is quiet."
    kind = str(record.get("kind") or "entry").strip()
    summary = str(record.get("summary") or "").strip()
    if summary:
        return f'Memory update ({kind}): "{summary}".'
    metadata = record.get("metadata")
    if isinstance(metadata, Mapping):
        keys = sorted(str(key) for key in metadata.keys())
        if keys:
            return f"Memory update ({kind}) with metadata keys: {', '.join(keys)}."
    return f"Memory update ({kind})."


TOPIC_TRANSLATORS: Dict[str, Any] = {
    "/memory/pilot_feed": summarise_pilot_feed,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Memory module persists Pete's sensations and pilot outputs via "
        "ROS services (/memory/memorize, /memory/associate, /memory/recall). "
        "Store events with vector embeddings so future recalls can ground the "
        "pilot's reasoning in past observations linked through Neo4j."
    ),
    (
        "When recalling, specify the collection name (faces, thoughts, emotions, "
        "etc.) so Qdrant searches the right neighbourhood. Returned memories "
        "include metadata describing origin topics, timestamps, and sources."
    ),
]


__all__ = ["TOPIC_TRANSLATORS", "STATIC_PROMPT_SECTIONS", "summarise_pilot_feed"]
