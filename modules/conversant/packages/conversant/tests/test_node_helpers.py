"""Tests for lightweight helper functions in :mod:`conversant.node`."""

from __future__ import annotations

from conversant.node import _enrich_metadata_with_topic


def test_enrich_metadata_preserves_existing_fields() -> None:
    """The helper keeps existing metadata while attaching the topic."""

    metadata = {"origin": "user"}
    result = _enrich_metadata_with_topic(metadata, topic=" Safety Check ")
    assert result["origin"] == "user"
    assert result["topic"] == "Safety Check"
    # Original dictionary should remain untouched for callers that reuse it.
    assert "topic" not in metadata


def test_enrich_metadata_ignores_empty_topic() -> None:
    """Empty or whitespace-only topics are excluded from the payload."""

    metadata = {"origin": "conversant"}
    result = _enrich_metadata_with_topic(metadata, topic="   ")
    assert "topic" not in result
    assert result["origin"] == "conversant"


def test_enrich_metadata_respects_existing_topic() -> None:
    """Existing topic metadata wins over the helper's suggestion."""

    metadata = {"topic": "Operator override"}
    result = _enrich_metadata_with_topic(metadata, topic="Conversant hint")
    assert result["topic"] == "Operator override"
