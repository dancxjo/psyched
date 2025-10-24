"""Tests for the ConcernResponder helper."""

from __future__ import annotations

from typing import Any

from conversant.node import ConcernResponder
from conversant.threads import ConversationThread


class _DummyLogger:
    def warning(self, *_: Any, **__: Any) -> None:
        pass

    def info(self, *_: Any, **__: Any) -> None:
        pass

    def debug(self, *_: Any, **__: Any) -> None:
        pass


class _DummyNode:
    def __init__(self) -> None:
        self._logger = _DummyLogger()

    def get_logger(self) -> _DummyLogger:
        return self._logger


def _make_thread() -> ConversationThread:
    thread = ConversationThread(thread_id="thread", created_at=0.0, updated_at=0.0)
    thread.append(role="user", text="Hi Pete", intent="")
    thread.append(role="conversant", text="Hello!", intent="")
    return thread


def test_concern_responder_detects_ollama_endpoint() -> None:
    node = _DummyNode()
    responder = ConcernResponder(
        node,
        "http://motherbrain.local:11434/api/generate",
        model="",
    )
    assert responder.enabled() is True
    assert responder.describe() == "ollama(gemma3:latest)"


def test_concern_responder_prompt_includes_history() -> None:
    node = _DummyNode()
    thread = _make_thread()
    responder = ConcernResponder(
        node,
        "http://motherbrain.local:11434/api/generate",
        model="gemma3:latest",
    )
    prompt = responder._build_ollama_prompt(  # type: ignore[attr-defined]
        concern="Check the battery",
        thread=thread,
        state={"battery": "low"},
    )
    assert "Operator: Hi Pete" in prompt
    assert "Current concern: Check the battery" in prompt
    assert "battery" in prompt


def test_concern_responder_normalises_response() -> None:
    node = _DummyNode()
    thread = _make_thread()
    responder = ConcernResponder(
        node,
        "http://motherbrain.local:11434/api/generate",
    )
    data = {
        "speak": "Working on it.",
        "intent": "",
        "escalate": "false",
    }
    response = responder._normalise_response(  # type: ignore[attr-defined]
        data,
        concern="Charge battery",
        thread=thread,
    )
    assert response.speak == "Working on it."
    assert response.intent == ""
    assert response.escalate is False
