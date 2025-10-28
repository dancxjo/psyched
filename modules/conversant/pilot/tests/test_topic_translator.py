"""Tests for the Conversant module topic translators."""

from modules.conversant.pilot.topic_translator import summarise_conversation_topic


def test_topic_summary_with_text() -> None:
    """The translator should reflect the active topic text."""

    payload = {"data": "Discussing mission objectives"}
    assert (
        summarise_conversation_topic(payload)
        == 'Conversant topic: "Discussing mission objectives".'
    )


def test_topic_summary_when_empty() -> None:
    """The translator should fall back to a neutral status when empty."""

    payload = {"data": "   "}
    assert summarise_conversation_topic(payload) == "Conversant is awaiting a topic update."


def test_topic_summary_with_raw_string() -> None:
    """Raw string payloads should be accepted for resilience."""

    assert (
        summarise_conversation_topic("Chit-chat about weather")
        == 'Conversant topic: "Chit-chat about weather".'
    )
