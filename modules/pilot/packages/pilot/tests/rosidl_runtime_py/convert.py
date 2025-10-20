"""Stub conversion utilities for ROS messages."""

from collections import OrderedDict
from typing import Any, Mapping


def message_to_ordereddict(msg: Any) -> Mapping[str, Any]:  # pragma: no cover - deterministic
    if hasattr(msg, "__dict__"):
        return OrderedDict(msg.__dict__)
    data = getattr(msg, "data", None)
    return OrderedDict({"data": data})
