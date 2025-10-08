"""Core helpers for the Psyched memory module."""

from .models import MemoryEventPayload, MemoryHeader, MemoryRecallResult, MemoryRecord
from .service import MemoryService

__all__ = [
    "MemoryEventPayload",
    "MemoryHeader",
    "MemoryRecord",
    "MemoryRecallResult",
    "MemoryService",
]


def __getattr__(name: str):
    if name == "MemoryNode":
        from .node import MemoryNode  # pragma: no cover - import is side effect free

        return MemoryNode
    raise AttributeError(name)


__all__.append("MemoryNode")
