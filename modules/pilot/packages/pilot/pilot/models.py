from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass(slots=True)
class FeelingIntentData:
    """Structured representation of the feeling + will response.

    Attributes mirror the :mod:`psyched_msgs.msg.FeelingIntent` message but are kept
    in a plain-Python form so the validation logic and memory pipeline can operate
    without importing ROS dependencies.  The class is intentionally lightweight so
    unit tests can freely instantiate it.

    Example
    -------
    >>> FeelingIntentData(
    ...     situation_overview="Pete is greeting a friendly face in the atrium.",
    ...     attitude_emoji="🙂",
    ...     thought_sentence="I should wave back.",
    ...     spoken_sentence="Hello there!",
    ...     command_script="nav.move_to(target='person_estimated')",
    ... )
    FeelingIntentData(situation_overview='Pete is greeting a friendly face in the atrium.', attitude_emoji='🙂', thought_sentence='I should wave back.', spoken_sentence='Hello there!', command_script="nav.move_to(target='person_estimated')", goals=[], mood_delta='', memory_collection_raw='', memory_collection_text='', memory_collection_emoji='', episode_id='', situation_id='')

    Notes
    -----
    ``spoken_sentence`` is automatically enqueued for speech, so scripts should
    avoid calling :func:`voice.say` with the same text unless repetition is
    deliberate.
    """

    situation_overview: str
    attitude_emoji: str
    thought_sentence: str
    spoken_sentence: str
    command_script: str = ""
    goals: List[str] = field(default_factory=list)
    mood_delta: str = ""
    memory_collection_raw: str = ""
    memory_collection_text: str = ""
    memory_collection_emoji: str = ""
    episode_id: str = ""
    situation_id: str = ""


@dataclass(slots=True)
class SensationSummary:
    """Concise description of an incoming sensation for prompt construction."""

    topic: str
    kind: str
    collection_hint: str
    json_payload: str
    vector_length: int

    def prompt_payload(self) -> Dict[str, Any]:
        """Return a JSON-serialisable payload summarising the sensation.

        The summary keeps the raw metadata (if the JSON is parseable) but avoids
        including the actual embedding vector which can be large and potentially
        sensitive.  Instead the vector length is surfaced so the LLM is aware that
        a raw vector exists.
        """

        import json

        try:
            payload = json.loads(self.json_payload) if self.json_payload else {}
        except json.JSONDecodeError:
            payload = {"raw": self.json_payload}

        return {
            "topic": self.topic,
            "kind": self.kind,
            "collection": self.collection_hint,
            "payload": payload,
            "vector_length": self.vector_length,
        }


@dataclass(slots=True)
class SensationRecord:
    """Representation of a sensation suitable for memory persistence."""

    topic: str
    kind: str
    collection_hint: str
    json_payload: str
    vector: List[float] = field(default_factory=list)

    @property
    def vector_length(self) -> int:
        """Return the length of the raw vector (0 if none)."""

        return len(self.vector)

    def to_summary(self) -> SensationSummary:
        """Create a :class:`SensationSummary` for prompt construction."""

        return SensationSummary(
            topic=self.topic,
            kind=self.kind,
            collection_hint=self.collection_hint,
            json_payload=self.json_payload,
            vector_length=self.vector_length,
        )

    def sensation_id(self) -> Optional[str]:
        """Extract a stable identifier from ``json_payload`` if available."""

        import json

        try:
            payload = json.loads(self.json_payload) if self.json_payload else {}
        except json.JSONDecodeError:
            return None
        value = payload.get("memory_id") or payload.get("vector_id") or payload.get("id")
        return str(value) if value is not None else None
