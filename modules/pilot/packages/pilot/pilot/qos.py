"""Shared QoS configuration helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping


@dataclass(slots=True)
class QosConfig:
    """Declarative QoS settings shared between manifest parsing and runtime."""

    history: str = "keep_last"
    depth: int = 10
    reliability: str = "reliable"
    durability: str = "volatile"

    @classmethod
    def from_mapping(cls, data: Mapping[str, object] | None) -> "QosConfig":
        if not data:
            return cls()
        return cls(
            history=str(data.get("history", "keep_last")),
            depth=int(data.get("depth", 10)),
            reliability=str(data.get("reliability", "reliable")),
            durability=str(data.get("durability", "volatile")),
        )

    def asdict(self) -> Dict[str, object]:
        return {
            "history": self.history,
            "depth": self.depth,
            "reliability": self.reliability,
            "durability": self.durability,
        }
