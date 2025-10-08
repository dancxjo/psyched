"""Data models for the memory module.

The classes in this module are deliberately lightweight so they can be
used within ROS 2 callbacks as well as plain unit tests.  They also
include inline usage examples to document the contract between the
memory node and its collaborators.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Mapping, MutableMapping, Optional, Sequence

__all__ = [
    "MemoryEventPayload",
    "MemoryHeader",
    "MemoryRecord",
    "MemoryRecallResult",
]


@dataclass(frozen=True)
class MemoryHeader:
    """Metadata describing the origin of a memory event.

    Parameters
    ----------
    stamp:
        The moment the observation was produced.  Values without an
        explicit timezone are assumed to be UTC to avoid subtle
        cross-host drift.
    frame_id:
        Reference frame (typically a TF frame) that produced the event.
    source:
        Human-readable identifier for the ROS node or sensor that
        created the memory event.

    Examples
    --------
    >>> from datetime import datetime, timezone
    >>> MemoryHeader(
    ...     stamp=datetime(2024, 5, 10, 12, 30, tzinfo=timezone.utc),
    ...     frame_id="camera/front",
    ...     source="sensor/face",
    ... )
    MemoryHeader(stamp=datetime.datetime(2024, 5, 10, 12, 30, tzinfo=datetime.timezone.utc), frame_id='camera/front', source='sensor/face')
    """

    stamp: datetime
    frame_id: str
    source: str

    def __post_init__(self) -> None:
        if self.stamp.tzinfo is None:
            object.__setattr__(self, "stamp", self.stamp.replace(tzinfo=timezone.utc))


@dataclass(frozen=True)
class MemoryEventPayload:
    """Structured representation of a :class:`MemoryEvent` ROS message.

    The dataclass normalises the raw JSON metadata and ensures that the
    optional embedding is represented as a list of floats.  The class is
    intentionally agnostic to the transport mechanism so it can be used
    by ROS services, HTTP handlers, or standalone scripts.

    Examples
    --------
    >>> from datetime import datetime, timezone
    >>> payload = MemoryEventPayload(
    ...     header=MemoryHeader(
    ...         stamp=datetime(2024, 5, 10, 12, 30, tzinfo=timezone.utc),
    ...         frame_id="camera/front",
    ...         source="sensor/face",
    ...     ),
    ...     kind="face",
    ...     json_data={"emotion": "happy"},
    ...     embedding=[0.1, 0.2, 0.3],
    ... )
    >>> payload.kind
    'face'
    >>> payload.embedding
    [0.1, 0.2, 0.3]
    """

    header: MemoryHeader
    kind: str
    json_data: Mapping[str, Any] = field(default_factory=dict)
    embedding: Optional[Sequence[float]] = None

    def __post_init__(self) -> None:
        if not self.kind:
            raise ValueError("kind must be a non-empty string")
        normalised: MutableMapping[str, Any] = dict(self.json_data)
        object.__setattr__(self, "json_data", normalised)
        if self.embedding is not None:
            object.__setattr__(self, "embedding", [float(value) for value in self.embedding])

    @property
    def metadata(self) -> Mapping[str, Any]:
        """Return a read-only mapping containing the event metadata."""

        return self.json_data


@dataclass(frozen=True)
class MemoryRecord:
    """Descriptor returned once a memory has been persisted."""

    memory_id: str
    vector_id: Optional[str]
    kind: str
    timestamp: datetime
    frame_id: str
    source: str
    metadata: Mapping[str, Any]


@dataclass(frozen=True)
class MemoryRecallResult:
    """Contextualised recall output.

    Attributes
    ----------
    memory_id:
        Identifier of the memory node in Neo4j.
    score:
        Similarity score returned by the vector database.
    metadata:
        Combined metadata from Qdrant and the graph node.
    """

    memory_id: str
    score: float
    metadata: Mapping[str, Any]
