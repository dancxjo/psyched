from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

import pytest

from chat.ollama_client import OllamaServiceClient, OllamaServiceError


class FakeResponse:
    """Minimal stand-in for ``requests.Response`` supporting the client contract."""

    def __init__(
        self,
        *,
        lines: Optional[List[str]] = None,
        body: Optional[Dict[str, Any]] = None,
        status_code: int = 200,
    ) -> None:
        self._lines = list(lines or [])
        self._body = body
        self.status_code = status_code
        self.closed = False

    def __enter__(self) -> "FakeResponse":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # noqa: ANN001
        self.close()

    def raise_for_status(self) -> None:
        if self.status_code >= 400:
            raise RuntimeError(f"status {self.status_code}")

    def iter_lines(self, decode_unicode: bool = False):  # noqa: ANN001
        for line in self._lines:
            yield line if decode_unicode else line.encode("utf-8")

    def json(self) -> Dict[str, Any]:
        if self._body is None:
            raise ValueError("No JSON body")
        return self._body

    def close(self) -> None:
        self.closed = True


class FakeSession:
    """Captures POST calls and returns a canned response."""

    def __init__(self, response: FakeResponse) -> None:
        self._response = response
        self.calls: List[Dict[str, Any]] = []

    def post(self, url: str, *, json: Dict[str, Any], stream: bool, timeout: float):  # noqa: ANN001
        self.calls.append({"url": url, "json": json, "stream": stream, "timeout": timeout})
        return self._response


def test_streaming_chat_accumulates_tokens_and_invokes_callback() -> None:
    response = FakeResponse(
        lines=[
            json.dumps({"message": {"role": "assistant", "content": "Hello"}}),
            json.dumps({"message": {"role": "assistant", "content": " world"}}),
            json.dumps({"done": True}),
        ]
    )
    session = FakeSession(response)
    client = OllamaServiceClient(host="http://localhost:11434", session=session)

    tokens: List[str] = []
    result = client.chat(
        model="gpt",
        messages=[{"role": "user", "content": "hi"}],
        stream_callback=tokens.append,
    )

    assert result == "Hello world"
    assert tokens == ["Hello", " world"]
    assert session.calls[0]["json"]["stream"] is True


def test_non_streaming_chat_returns_message_content() -> None:
    response = FakeResponse(
        body={"choices": [{"message": {"content": "Affirmative"}}]},
    )
    session = FakeSession(response)
    client = OllamaServiceClient(host="http://localhost:11434", session=session)

    result = client.chat(model="gpt", messages=[{"role": "user", "content": "status"}])

    assert result == "Affirmative"
    assert session.calls[0]["json"]["stream"] is False


def test_raises_ollama_service_error_on_failure() -> None:
    response = FakeResponse(status_code=503)
    session = FakeSession(response)
    client = OllamaServiceClient(host="http://localhost:11434", session=session)

    with pytest.raises(OllamaServiceError):
        client.chat(model="gpt", messages=[{"role": "user", "content": "ping"}])
