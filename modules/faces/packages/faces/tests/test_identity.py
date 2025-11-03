"""Tests for identity resolution helpers in the faces module."""
from __future__ import annotations

from typing import Any, Mapping, Sequence

from faces.face_detector_node import resolve_identity_from_recall


def _make_recall(
    *,
    memory_id: str,
    score: float,
    metadata: Mapping[str, Any],
) -> Mapping[str, Any]:
    return {
        "memory_id": memory_id,
        "score": score,
        "metadata": metadata,
    }


def test_resolve_identity_selects_highest_scoring_match() -> None:
    """Identity resolution should surface the best scoring recall entry."""

    recall: Sequence[Mapping[str, Any]] = [
        _make_recall(
            memory_id="mem-1",
            score=0.91,
            metadata={
                "identity": {
                    "id": "person:alice",
                    "name": "Alice Example",
                    "signatures": ["sig-old"],
                },
            },
        ),
        _make_recall(
            memory_id="mem-2",
            score=0.72,
            metadata={
                "identity": {
                    "id": "person:bob",
                    "name": "Bob Example",
                },
            },
        ),
    ]

    identity, matches = resolve_identity_from_recall("sig-new", recall)

    assert identity is not None
    assert identity["id"] == "person:alice"
    assert identity["name"] == "Alice Example"
    assert identity["confidence"] == 0.91
    assert "sig-new" in identity.get("signatures", [])
    assert matches and matches[0]["identity"]["name"] == "Alice Example"


def test_resolve_identity_returns_none_when_scores_too_low() -> None:
    """Low similarity scores should not produce an identity match."""

    recall: Sequence[Mapping[str, Any]] = [
        _make_recall(
            memory_id="mem-3",
            score=0.45,
            metadata={
                "identity": {
                    "id": "person:low",
                    "name": "Low Confidence",
                },
            },
        ),
    ]

    identity, matches = resolve_identity_from_recall("sig-low", recall, min_score=0.8)

    assert identity is None
    assert matches and matches[0]["memory_id"] == "mem-3"
