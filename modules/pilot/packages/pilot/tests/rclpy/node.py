"""Minimal :mod:`rclpy.node` stub for unit testing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


@dataclass
class _Parameter:
    value: Any


class _Logger:
    def info(self, message: str) -> None:  # pragma: no cover - trivial logging stub
        return None

    def warning(self, message: str) -> None:  # pragma: no cover - trivial
        return None

    def error(self, message: str) -> None:  # pragma: no cover - trivial
        return None

    def debug(self, message: str) -> None:  # pragma: no cover - trivial
        return None


class Node:
    """Stubbed ROS node exposing the minimal interface needed for tests."""

    def __init__(self, name: str) -> None:
        self.name = name

    def declare_parameter(self, name: str, default: Any) -> _Parameter:
        return _Parameter(default)

    def create_publisher(self, *args: Any, **kwargs: Any) -> Any:  # pragma: no cover - stub
        return object()

    def create_subscription(
        self,
        msg_type: Any,
        topic: str,
        callback: Callable[[Any], None],
        qos: Any,
    ) -> Any:  # pragma: no cover - stub
        return object()

    def create_timer(self, interval: float, callback: Callable[[], None]) -> Any:  # pragma: no cover - stub
        return object()

    def destroy_node(self) -> bool:  # pragma: no cover - stub
        return True

    def get_clock(self) -> Any:  # pragma: no cover - stub
        class _Clock:
            def now(self_inner) -> Any:
                class _Now:
                    def to_msg(self_now) -> Any:
                        return type("_Stamp", (), {"sec": 0, "nanosec": 0})()

                return _Now()

        return _Clock()

    def get_logger(self) -> _Logger:
        return _Logger()
