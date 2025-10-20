"""Simplified ``SensationStamped`` message for tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class SensationStamped:
    kind: str = ""
    collection_hint: str = ""
    json_payload: str = ""
    vector: List[float] = field(default_factory=list)
