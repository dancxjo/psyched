"""Simplified ``String`` message for tests."""

from dataclasses import dataclass


@dataclass
class String:
    data: str = ""
