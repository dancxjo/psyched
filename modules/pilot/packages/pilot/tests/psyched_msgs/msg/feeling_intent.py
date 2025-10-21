"""Simplified ``FeelingIntent`` message for tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class _Stamp:
    sec: int = 0
    nanosec: int = 0


@dataclass
class FeelingIntent:
    stamp: _Stamp = field(default_factory=_Stamp)
    source_topics: List[str] = field(default_factory=list)
    situation_overview: str = ""
    attitude_emoji: str = ""
    thought_sentence: str = ""
    spoken_sentence: str = ""
    command_script: str = ""
    goals: List[str] = field(default_factory=list)
    mood_delta: str = ""
    memory_collection_raw: str = ""
    memory_collection_text: str = ""
    memory_collection_emoji: str = ""
    episode_id: str = ""
    situation_id: str = ""
