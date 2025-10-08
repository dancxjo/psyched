"""Tests covering the pilot cockpit foot telemetry bridge."""

from __future__ import annotations

import builtins
import sys
from types import ModuleType
from typing import Callable, List

import pytest

from pilot.cockpit.foot import FootEvent, FootTelemetryBridge


class _FakeClock:
    @property
    def nanoseconds(self) -> int:
        return 0


class _FakeClockSource:
    def now(self) -> _FakeClock:
        return _FakeClock()


class _FakeLogger:
    def __init__(self) -> None:
        self.messages: List[str] = []

    def warning(self, message: str) -> None:
        self.messages.append(message)


class _FakeNode:
    def __init__(self) -> None:
        self._logger = _FakeLogger()
        self.subscriptions: List[tuple[str, Callable[..., None]]] = []

    def create_subscription(self, _msg_type, topic: str, callback: Callable[..., None], _qos: int):
        self.subscriptions.append((topic, callback))
        return object()

    def get_logger(self) -> _FakeLogger:
        return self._logger

    def get_clock(self) -> _FakeClockSource:
        return _FakeClockSource()


def test_bridge_initialises_without_create_msgs(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pilot should start even when create_msgs is unavailable."""

    original_import = builtins.__import__

    def guard(name, globals=None, locals=None, fromlist=(), level=0):  # type: ignore[override]
        if name.startswith("create_msgs"):
            raise ModuleNotFoundError("create_msgs missing for test")
        return original_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guard)

    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs.__path__ = []  # type: ignore[attr-defined]
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = type("Twist", (), {})
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)

    nav_msgs = ModuleType("nav_msgs")
    nav_msgs.__path__ = []  # type: ignore[attr-defined]
    nav_msgs_msg = ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = type("Odometry", (), {})
    monkeypatch.setitem(sys.modules, "nav_msgs", nav_msgs)
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", nav_msgs_msg)

    std_msgs = ModuleType("std_msgs")
    std_msgs.__path__ = []  # type: ignore[attr-defined]
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.Float32 = type("Float32", (), {})
    std_msgs_msg.Int16 = type("Int16", (), {})
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)

    node = _FakeNode()
    events: List[FootEvent] = []

    FootTelemetryBridge(node, lambda message: events.append(message), now=lambda: 0)

    topics = {topic for topic, _ in node.subscriptions}
    assert topics == {
        "/battery/charge_ratio",
        "/battery/capacity",
        "/battery/charge",
        "/battery/current",
        "/battery/voltage",
        "/battery/temperature",
        "/cmd_vel",
        "/odom",
    }
    assert any("create_msgs" in message for message in node.get_logger().messages)
