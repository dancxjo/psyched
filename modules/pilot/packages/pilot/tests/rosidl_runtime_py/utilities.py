"""Stub utilities for message imports."""

from typing import Any


def get_message(type_name: str) -> Any:  # pragma: no cover - deterministic stub
    class _Message:
        __slots__ = ("_type",)

        def __init__(self) -> None:
            self._type = type_name

    return _Message
