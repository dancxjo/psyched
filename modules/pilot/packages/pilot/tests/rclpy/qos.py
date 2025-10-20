"""Simplified QoS stubs for tests."""

from __future__ import annotations


class QoSHistoryPolicy:
    KEEP_LAST = 1


class ReliabilityPolicy:
    RELIABLE = 1


class QoSProfile:
    def __init__(self, depth: int = 10) -> None:
        self.depth = depth
        self.history = None
        self.reliability = None
