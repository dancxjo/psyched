from __future__ import annotations

import io
import json
from typing import Any
from unittest.mock import patch

from pilot.node import OllamaLLMClient


class FakeResponse(io.BytesIO):
    """Minimal context manager returning a static body."""

    def __enter__(self) -> "FakeResponse":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # pragma: no cover - trivial cleanup
        self.close()

    def __iter__(self):  # pragma: no cover - force non-iterable path
        raise TypeError("not iterable")


def test_ollama_client_attaches_images_payload(monkeypatch: Any) -> None:
    captured: dict[str, bytes] = {}

    def fake_urlopen(request_obj, timeout: float):  # type: ignore[override]
        captured["payload"] = request_obj.data
        return FakeResponse(b"{\"response\": \"ok\"}")

    client = OllamaLLMClient(model="test", host="http://ollama")

    with patch("pilot.node.request.urlopen", side_effect=fake_urlopen):
        response = client.generate("hello", images=["AAE="])

    assert response == "ok"
    payload = json.loads(captured["payload"].decode("utf-8"))
    assert payload["images"] == ["AAE="]
    assert payload["prompt"] == "hello"
