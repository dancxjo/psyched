"""Natural language summaries for ear module pilot topics."""

from __future__ import annotations

import json
from typing import Any, Mapping


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
    return "ASR is listening for the next utterance."


TOPIC_TRANSLATORS = {
    "/ear/hole": summarise_asr_transcript,
}
