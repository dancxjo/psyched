"""Prompt translators surfacing Conversant topics to the pilot."""

from __future__ import annotations

from typing import Any, Mapping


def _extract_topic_text(payload: Any) -> str | None:
    """Return the underlying topic text when available."""

    if isinstance(payload, Mapping):
        data = payload.get("data")
        if isinstance(data, str):
            text = data.strip()
            if text:
                return text
    elif isinstance(payload, str):
        text = payload.strip()
        if text:
            return text
    return None


def summarise_conversation_topic(payload: Any) -> str:
    """Summarise the active Conversant topic for ``/conversant/topic``.

    Examples
    --------
    >>> summarise_conversation_topic({"data": "Mission planning"})
    'Conversant topic: "Mission planning".'
    >>> summarise_conversation_topic({"data": "   "})
    'Conversant is awaiting a topic update.'
    """

    text = _extract_topic_text(payload)
    if text is None:
        return "Conversant is awaiting a topic update."
    return f"Conversant topic: \"{text}\"."


TOPIC_TRANSLATORS = {
    "/conversant/topic": summarise_conversation_topic,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Conversant reports the active discussion topic via /conversant/topic. "
        "Treat each update as the freshest conversational thread so the pilot "
        "can respond in sync with nearby humans."
    )
]


__all__ = ["TOPIC_TRANSLATORS", "STATIC_PROMPT_SECTIONS", "summarise_conversation_topic"]
