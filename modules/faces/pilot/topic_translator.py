"""Natural language summaries for face detection triggers."""

from __future__ import annotations

import json
from typing import Any, Mapping


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


def summarise_face_trigger(payload: Any) -> str:
    """Summarize the JSON trigger emitted on ``/vision/face_detected``.

    Examples
    --------
    >>> summarise_face_trigger({'data': '{"name": "Stranger"}'})
    'Face trigger: Stranger.'
    >>> summarise_face_trigger({})
    'Face trigger: awaiting identification.'
    """

    parsed = _load_payload(payload)
    if not parsed:
        return "Face trigger: awaiting identification."

    name = str(parsed.get("name") or "").strip() or "Unknown"
    signature = str(parsed.get("signature") or "").strip()
    memory_hint = str(parsed.get("memory_hint") or "").strip()
    vector_hint = str(parsed.get("vector_hint") or "").strip()
    memory_id = str(parsed.get("memory_id") or "").strip()
    vector_id = str(parsed.get("vector_id") or "").strip()

    if not memory_id:
        if memory_hint:
            memory_id = memory_hint
        elif signature:
            memory_id = f"mem-{signature}"
    if not vector_id:
        if vector_hint:
            vector_id = vector_hint
        elif signature:
            vector_id = signature

    extras: list[str] = []
    if memory_id:
        extras.append(f"memory {memory_id}")
    if vector_id:
        extras.append(f"vector {vector_id}")

    if extras:
        return f"Face trigger: {name} ({', '.join(extras)})."
    return f"Face trigger: {name}."


TOPIC_TRANSLATORS = {
    "/vision/face_detected": summarise_face_trigger,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Faces publishes /vision/face_detected when a new embedding crosses the "
        "cooldown window. Summaries surface the recognised label and any linked "
        "memory/vector identifiers so the pilot can recall context." 
    )
]


__all__ = ["TOPIC_TRANSLATORS", "STATIC_PROMPT_SECTIONS", "summarise_face_trigger"]
