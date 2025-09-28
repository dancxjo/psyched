from __future__ import annotations

import asyncio
import json
from pathlib import Path
import sys
from typing import Any, List

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from chat.llm_client import ForebrainLLMClient, ForebrainUnavailable


class StubConnection:
    """Async context manager that mimics the websocket protocol."""

    def __init__(self, responses: List[Any]) -> None:
        self.responses = list(responses)
        self.sent: List[str] = []
        self.closed = False

    async def __aenter__(self) -> "StubConnection":
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:  # noqa: ANN001
        self.closed = True

    async def send(self, message: str) -> None:
        self.sent.append(message)

    async def recv(self) -> Any:
        if not self.responses:
            raise asyncio.TimeoutError
        item = self.responses.pop(0)
        if isinstance(item, Exception):
            raise item
        return item


async def stub_connector(url: str, **_kwargs: Any) -> StubConnection:  # noqa: ANN001
    raise AssertionError("stub_connector requires fixture override")


class TestForebrainClient:
    def test_collects_streamed_tokens_and_returns_sentence(self, monkeypatch):
        messages = [
            json.dumps({"role": "system", "content": "session reset"}),
            json.dumps({"role": "assistant", "content": "Hello"}),
            json.dumps({"role": "assistant", "content": " world"}),
            asyncio.TimeoutError(),
        ]
        connection = StubConnection(messages)

        def connector(_url: str, **_kwargs: Any) -> StubConnection:  # noqa: ANN001
            return connection

        client = ForebrainLLMClient(
            uri="ws://example.invalid/chat",
            connect_timeout=0.1,
            response_timeout=0.1,
            logger=None,
            connector=connector,
        )

        history = [
            {"role": "system", "content": "You are helpful"},
            {"role": "user", "content": "Say hi"},
        ]
        result = client.generate(history)

        assert result == "Hello world"
        assert connection.closed is True
        sent_payloads = [json.loads(msg) for msg in connection.sent]
        assert sent_payloads[0] == {"command": "reset"}
        assert sent_payloads[1] == history[0]
        assert sent_payloads[2] == history[1]

    def test_raises_when_connection_fails(self):
        def connector(_url: str, **_kwargs: Any) -> StubConnection:  # noqa: ANN001
            raise OSError("connection refused")

        client = ForebrainLLMClient(
            uri="ws://example.invalid/chat",
            connect_timeout=0.1,
            response_timeout=0.1,
            logger=None,
            connector=connector,
        )

        with pytest.raises(ForebrainUnavailable):
            client.generate([
                {"role": "system", "content": "test"},
                {"role": "user", "content": "hello"},
            ])

