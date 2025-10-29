"""Natural language summaries for ear module pilot topics."""

from __future__ import annotations

import json
from typing import Any, Mapping, Sequence


def _extract_text(payload: Any) -> str | None:
    """Return a trimmed string from *payload* when possible."""

    if isinstance(payload, Mapping):
        data = payload.get("data")
        if isinstance(data, Mapping):
            data = data.get("data")
        if isinstance(data, str):
            text = data.strip()
            return text or None
    elif isinstance(payload, str):
        text = payload.strip()
        return text or None
    return None


_LISTENING_MESSAGE = "ASR is listening for the next utterance."


def summarise_asr_transcript(payload: Any) -> str:
    """Describe the latest ASR transcript in conversational language.

    Examples
    --------
    >>> summarise_asr_transcript({"data": "hello there"})
    'ASR heard: "hello there".'
    >>> summarise_asr_transcript({})
    'ASR is listening for the next utterance.'
    """

    text = _extract_text(payload)
    if text:
        return f"ASR heard: {json.dumps(text, ensure_ascii=False)}."
    return _LISTENING_MESSAGE


def _segments_to_text(segments: Any) -> str:
    """Return a whitespace-joined transcript extracted from segment payloads."""

    if not isinstance(segments, Sequence):
        return ""
    parts = []
    for entry in segments:
        if not isinstance(entry, Mapping):
            continue
        text = str(entry.get("text", "")).strip()
        if text:
            parts.append(text)
    return " ".join(parts)


def summarise_asr_event(payload: Any) -> str:
    """Describe partial and final ASR events for the prompt builder."""

    raw_text = _extract_text(payload)
    if not raw_text:
        return _LISTENING_MESSAGE
    try:
        event_payload = json.loads(raw_text)
    except json.JSONDecodeError:
        quoted = json.dumps(raw_text, ensure_ascii=False)
        return f"I heard: {quoted}."

    if not isinstance(event_payload, Mapping):
        return _LISTENING_MESSAGE

    event_kind = str(event_payload.get("event", "")).strip().lower()
    transcript = str(event_payload.get("text", "")).strip()
    if not transcript:
        transcript = _segments_to_text(event_payload.get("segments"))
    if not transcript:
        return _LISTENING_MESSAGE

    quoted = json.dumps(transcript, ensure_ascii=False)
    if event_kind == "partial":
        return f"I think I'm hearing: {quoted}."
    return f"I heard: {quoted}."


TOPIC_TRANSLATORS = {
    "/ear/hole": summarise_asr_transcript,
    "/ear/asr_event": summarise_asr_event,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Ear module streams /ear/hole transcripts summarising the freshest words "
        "humans speak nearby. Each entry reflects the most recent snippet inside "
        "the rolling instant window—older utterances scroll away automatically."
    )
]
