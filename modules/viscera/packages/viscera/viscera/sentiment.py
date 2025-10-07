"""Sentiment representations emitted by the viscera module."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Mapping, Tuple


@dataclass(frozen=True)
class Sentiment:
    """Narrative that describes how Pete feels about a subsystem.

    Parameters
    ----------
    name:
        Machine friendly identifier for the feeling.
    intensity:
        Normalised strength between ``0`` (background) and ``1`` (urgent).
    narrative:
        Human readable sentence that can be surfaced in higher reasoning layers.
    evidence:
        Optional key/value pairs explaining what data triggered the feeling.
    tags:
        Semantic hints (e.g. ``("energy", "hunger")``) that other modules can
        consume.

    Examples
    --------
    >>> Sentiment(name="content", intensity=0.2, narrative="I feel content.")
    Sentiment(name='content', intensity=0.2, narrative='I feel content.', evidence={}, tags=())
    """

    name: str
    intensity: float
    narrative: str
    evidence: Mapping[str, object] = field(default_factory=dict)
    tags: Tuple[str, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if not 0.0 <= self.intensity <= 1.0:
            raise ValueError("Sentiment intensity must be in the [0.0, 1.0] range")
        object.__setattr__(self, "evidence", dict(self.evidence))
        object.__setattr__(self, "tags", tuple(self.tags))

    def with_intensity(self, intensity: float) -> "Sentiment":
        """Return a new :class:`Sentiment` with a different intensity."""
        return Sentiment(
            name=self.name,
            intensity=intensity,
            narrative=self.narrative,
            evidence=self.evidence,
            tags=self.tags,
        )


__all__ = ["Sentiment"]
