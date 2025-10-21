"""Natural language summaries for voice module pilot topics."""

from __future__ import annotations

import json
from typing import Any, Mapping


def _extract_text(payload: Any) -> str | None:
    """Return the underlying string from *payload* if available."""

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


def summarise_queue(payload: Any) -> str:
    """Summarise the queued speech request for ``/voice``.

    Examples
    --------
    >>> summarise_queue({"data": "please hold"})
    'Voice queue: "please hold".'
    >>> summarise_queue({})
    'Voice queue is idle.'
    """

    text = _extract_text(payload)
    if text:
        return f"Voice queue: {json.dumps(text, ensure_ascii=False)}."
    return "Voice queue is idle."


def summarise_spoken(payload: Any) -> str:
    """Summarise the most recently spoken text for ``/voice/spoken``.

    Examples
    --------
    >>> summarise_spoken({"data": "affirmative"})
    'Just said: "affirmative".'
    >>> summarise_spoken({})
    'No speech has been spoken yet.'
    """

    text = _extract_text(payload)
    if text:
        return f"Just said: {json.dumps(text, ensure_ascii=False)}."
    return "No speech has been spoken yet."


TOPIC_TRANSLATORS = {
    "/voice": summarise_queue,
    "/voice/spoken": summarise_spoken,
}
