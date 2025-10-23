"""Conversation thread helpers for the Conversant module."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, Iterable, List, MutableSequence, Optional
from uuid import uuid4

__all__ = ["ConversationTurn", "ConversationThread", "ThreadStore", "serialise_turn"]


@dataclass(slots=True)
class ConversationTurn:
    """Single conversational exchange captured by Conversant."""

    role: str
    text: str
    intent: str = ""
    timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    metadata: Dict[str, str] = field(default_factory=dict)


@dataclass(slots=True)
class ConversationThread:
    """Mutable collection of turns bound to a logical conversation."""

    thread_id: str
    created_at: float
    updated_at: float
    turns: MutableSequence[ConversationTurn] = field(default_factory=list)

    def append(
        self,
        *,
        role: str,
        text: str,
        intent: str = "",
        metadata: Optional[Dict[str, str]] = None,
        timestamp: Optional[datetime] = None,
        max_turns: Optional[int] = None,
    ) -> ConversationTurn:
        """Record a new turn and return it.

        Parameters
        ----------
        role:
            Conversational role (e.g. ``"user"`` or ``"conversant"``).
        text:
            Utterance content.
        intent:
            Optional XML-tagged intent block such as ``<intend>``.
        metadata:
            Lightweight supplementary fields persisted alongside the turn.
        timestamp:
            Explicit timestamp; defaults to :func:`datetime.now` in UTC.
        max_turns:
            Optional maximum number of turns to keep; older entries are trimmed
            from the front when exceeded.
        """

        stamp = timestamp or datetime.now(timezone.utc)
        turn = ConversationTurn(
            role=role,
            text=text,
            intent=intent,
            timestamp=stamp,
            metadata=dict(metadata or {}),
        )
        self.turns.append(turn)
        if isinstance(max_turns, int) and max_turns > 0:
            excess = len(self.turns) - max_turns
            if excess > 0:
                del self.turns[:excess]
        self.updated_at = time.monotonic()
        return turn

    def is_expired(self, *, now: Optional[float] = None, ttl_seconds: float) -> bool:
        """Return ``True`` when the thread exceeded ``ttl_seconds`` of inactivity."""

        if ttl_seconds <= 0:
            return False
        current = now if now is not None else time.monotonic()
        return current - self.updated_at > ttl_seconds

    def snapshot(self) -> Dict[str, object]:
        """Return a serialisable snapshot of the thread."""

        return {
            "thread_id": self.thread_id,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "turns": [serialise_turn(turn) for turn in self.turns],
        }


class ThreadStore:
    """Manage the lifecycle of :class:`ConversationThread` objects."""

    def __init__(self, *, ttl_seconds: float, max_turns: int = 12) -> None:
        self._ttl_seconds = max(ttl_seconds, 0.0)
        self._max_turns = max(max_turns, 1)
        self._threads: Dict[str, ConversationThread] = {}

    @property
    def ttl_seconds(self) -> float:
        return self._ttl_seconds

    @property
    def max_turns(self) -> int:
        return self._max_turns

    def get(self, thread_id: Optional[str] = None) -> ConversationThread:
        """Return an existing thread or create a new one."""

        now = time.monotonic()
        if thread_id:
            thread = self._threads.get(thread_id)
            if thread and not thread.is_expired(now=now, ttl_seconds=self._ttl_seconds):
                thread.updated_at = now
                return thread
        new_id = thread_id or str(uuid4())
        thread = ConversationThread(thread_id=new_id, created_at=now, updated_at=now)
        self._threads[new_id] = thread
        return thread

    def prune(self) -> List[str]:
        """Remove expired threads and return their identifiers."""

        now = time.monotonic()
        expired: List[str] = []
        for thread_id, thread in list(self._threads.items()):
            if thread.is_expired(now=now, ttl_seconds=self._ttl_seconds):
                expired.append(thread_id)
                del self._threads[thread_id]
        return expired

    def append(
        self,
        *,
        thread_id: Optional[str],
        role: str,
        text: str,
        intent: str = "",
        metadata: Optional[Dict[str, str]] = None,
    ) -> tuple[ConversationThread, ConversationTurn]:
        """Append a turn to a thread, creating the thread when necessary."""

        thread = self.get(thread_id)
        turn = thread.append(
            role=role,
            text=text,
            intent=intent,
            metadata=metadata,
            max_turns=self._max_turns,
        )
        return thread, turn

    def snapshot(self) -> Dict[str, Dict[str, object]]:
        """Return a serialisable snapshot for debugging or cockpit export."""

        return {thread_id: thread.snapshot() for thread_id, thread in self._threads.items()}

    def __len__(self) -> int:
        return len(self._threads)

    def __iter__(self) -> Iterable[ConversationThread]:
        return iter(self._threads.values())


def serialise_turn(turn: ConversationTurn) -> Dict[str, object]:
    """Return a dictionary representation of *turn* suitable for JSON dumps."""

    return {
        "role": turn.role,
        "text": turn.text,
        "intent": turn.intent,
        "timestamp": turn.timestamp.isoformat(),
        "metadata": dict(turn.metadata),
    }
