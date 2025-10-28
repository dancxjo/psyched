"""Tests for Memory module topic translator."""

from modules.memory.pilot.topic_translator import summarise_pilot_feed


def test_summarise_pilot_feed_includes_kind_and_summary() -> None:
    """Pilot feed entries should surface their kind and summary text."""

    payload = {
        "data": '{"kind": "thought", "summary": "Logged ear transcript"}'
    }
    assert (
        summarise_pilot_feed(payload)
        == 'Memory update (thought): "Logged ear transcript".'
    )


def test_summarise_pilot_feed_handles_invalid_payload() -> None:
    """Ensure we fall back when the feed has not published yet."""

    assert summarise_pilot_feed({}) == "Memory update channel is quiet."
