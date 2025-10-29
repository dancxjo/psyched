from __future__ import annotations

import json
from dataclasses import dataclass
from typing import List

import pytest

from pilot.node import PilotNode
from pilot.command_script import CommandInvocation


@dataclass
class _PublishedMessage:
    data: str


class _DummyPublisher:
    """Capture published std_msgs/String messages for assertions."""

    def __init__(self) -> None:
        self.messages: List[_PublishedMessage] = []

    def publish(self, message) -> None:  # type: ignore[override]
        self.messages.append(_PublishedMessage(data=getattr(message, "data", "")))


class _DummyLogger:
    def __init__(self) -> None:
        self.entries: List[tuple[str, str]] = []

    def info(self, message: str) -> None:  # pragma: no cover - log capture only
        self.entries.append(("info", message))

    def warning(self, message: str) -> None:  # pragma: no cover - log capture only
        self.entries.append(("warning", message))

    def error(self, message: str) -> None:  # pragma: no cover - log capture only
        self.entries.append(("error", message))

    def debug(self, message: str) -> None:  # pragma: no cover - log capture only
        self.entries.append(("debug", message))


@pytest.fixture
def pilot_stub() -> PilotNode:
    """Return a minimally initialised :class:`PilotNode` for queue tests."""

    node = object.__new__(PilotNode)
    node._voice_publisher = _DummyPublisher()
    node._voice_topic_name = "/voice"
    node._last_spoken_sentence = None
    node.get_logger = lambda: _DummyLogger()  # type: ignore[attr-defined]
    return node


def test_queue_spoken_sentence_publishes_and_deduplicates(pilot_stub: PilotNode) -> None:
    """Pilot should publish each new spoken sentence exactly once."""

    publisher: _DummyPublisher = pilot_stub._voice_publisher  # type: ignore[attr-defined]

    pilot_stub._queue_spoken_sentence("Hello there")  # type: ignore[attr-defined]
    pilot_stub._queue_spoken_sentence("Hello there")  # type: ignore[attr-defined]
    pilot_stub._queue_spoken_sentence("Different line")  # type: ignore[attr-defined]

    assert [entry.data for entry in publisher.messages] == [
        "Hello there",
        "Different line",
    ]


def test_queue_spoken_sentence_ignores_empty_payloads(pilot_stub: PilotNode) -> None:
    """Blank spoken sentences should not reach the voice topic."""

    publisher: _DummyPublisher = pilot_stub._voice_publisher  # type: ignore[attr-defined]

    pilot_stub._queue_spoken_sentence("   ")  # type: ignore[attr-defined]
    pilot_stub._queue_spoken_sentence("")  # type: ignore[attr-defined]

    assert publisher.messages == []


def test_dispatch_voice_say_enqueues_via_voice_queue(monkeypatch) -> None:
    """voice.say command should enqueue speech using the same voice topic queue."""

    node = object.__new__(PilotNode)
    node._action_schemas = {}
    node._voice_publisher = _DummyPublisher()
    node._voice_topic_name = "/voice"
    node._last_spoken_sentence = None
    node.get_logger = lambda: _DummyLogger()  # type: ignore[attr-defined]

    def _forbid_urlopen(*args, **kwargs):
        raise AssertionError("urlopen should not be used for voice.say dispatch")

    monkeypatch.setattr("pilot.node.request.urlopen", _forbid_urlopen)

    invocation = CommandInvocation(module="voice", action="say", arguments={"text": "Hello there"})
    success, response = node._dispatch_action(invocation, ["voice.say"])  # type: ignore[attr-defined]

    assert success
    payload = json.loads(response)
    assert payload["status"] == "queued"
    assert payload["topic"] == "/voice"
    publisher: _DummyPublisher = node._voice_publisher  # type: ignore[attr-defined]
    assert [entry.data for entry in publisher.messages] == ["Hello there"]
