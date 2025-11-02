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
    """Summarize the active Conversant stream for ``/conversant/topic``.

    Examples
    --------
    >>> summarise_conversation_topic({"data": "/conversation/default"})
    'Conversant conversation stream: /conversation/default.'
    >>> summarise_conversation_topic({"data": "   "})
    'Conversant is awaiting a conversation stream update.'
    """

    text = _extract_topic_text(payload)
    if text is None:
        return "Conversant is awaiting a conversation stream update."
    return f"Conversant conversation stream: {text}."


TOPIC_TRANSLATORS = {
    "/conversant/topic": summarise_conversation_topic,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Conversant publishes the active transcript stream via /conversant/topic. "
        "Each update points to a /conversation/<thread> topic carrying the "
        "assistant and user messages."
    )
]


__all__ = ["TOPIC_TRANSLATORS", "STATIC_PROMPT_SECTIONS", "summarise_conversation_topic"]
