import audioop
import json
import sys
from pathlib import Path
from typing import Any

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from voice.providers import ProviderUnavailable, WebsocketConfig, WebsocketTTSProvider


class StubConnection:
    """Async context manager that mimics a websocket connection for tests."""

    def __init__(self, messages: list[Any]) -> None:
        self._messages = list(messages)
        self.sent: list[str] = []
        self.closed = False

    async def __aenter__(self) -> "StubConnection":
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        self.closed = True

    async def send(self, message: str) -> None:
        self.sent.append(message)

    async def recv(self) -> Any:
        if not self._messages:
            pytest.fail("recv called with no remaining messages")
        return self._messages.pop(0)


class StubPlayer:
    """Capture PCM bytes written by the provider."""

    def __init__(self) -> None:
        self.stdin = self
        self.buffer = bytearray()
        self.closed = False

    def write(self, data: bytes) -> int:
        self.buffer.extend(data)
        return len(data)

    def close(self) -> None:
        self.closed = True

    def flush(self) -> None:  # pragma: no cover - compatibility shim.
        pass

    def wait(self, timeout: float | None = None) -> None:
        return None

    def terminate(self) -> None:
        self.closed = True


@pytest.fixture()
def default_config() -> WebsocketConfig:
    return WebsocketConfig(
        uri="ws://example.invalid/tts",
        speaker=None,
        language=None,
        open_timeout=5.0,
        close_timeout=5.0,
        metadata_timeout=1.0,
        chunk_timeout=1.0,
        ping_interval=20.0,
        ping_timeout=20.0,
        playback_backend="test",
        playback_device=None,
    )


class TestWebsocketProvider:
    def test_streaming_updates_duration_and_writes_audio(self, default_config: WebsocketConfig) -> None:
        sample_rate = 24000
        chunk = (b"\x01\x02" * 2400)  # 2400 samples (~100ms)
        messages = [
            json.dumps(
                {
                    "event": "start",
                    "sample_rate": sample_rate,
                    "channels": 1,
                    "format": "pcm_s16le",
                    "num_samples": len(chunk) // 2,
                }
            ),
            chunk,
            json.dumps({"event": "end", "num_samples": len(chunk) // 2}),
        ]
        stub_ws = StubConnection(messages)
        players: list[StubPlayer] = []

        def player_factory(*_args: Any, **_kwargs: Any) -> StubPlayer:
            player = StubPlayer()
            players.append(player)
            return player

        provider = WebsocketTTSProvider(
            logger=None,
            config_getter=lambda: default_config,
            connector_factory=lambda *_args, **_kwargs: stub_ws,
            player_factory=player_factory,
        )

        durations: list[int] = []

        def on_duration(ms: int) -> None:
            durations.append(ms)

        success = provider.speak(
            "Hello there",
            volume=0.5,
            on_duration=on_duration,
        )

        assert success is True
        assert stub_ws.sent == [
            json.dumps({"text": "Hello there"}),
        ]
        assert durations[-1] == pytest.approx(100, rel=1e-2, abs=1)
        # Audio is scaled using audioop.mul
        expected = audioop.mul(chunk, 2, 0.5)
        assert players, "player_factory should have been invoked"
        assert bytes(players[0].buffer) == expected

    def test_error_event_raises(self, default_config: WebsocketConfig) -> None:
        messages = [
            json.dumps({"event": "error", "message": "boom"}),
        ]
        stub_ws = StubConnection(messages)
        provider = WebsocketTTSProvider(
            logger=None,
            config_getter=lambda: default_config,
            connector_factory=lambda *_args, **_kwargs: stub_ws,
            player_factory=lambda *_args, **_kwargs: StubPlayer(),
        )

        with pytest.raises(ProviderUnavailable):
            provider.speak("fail", volume=1.0, on_duration=lambda _ms: None)

    def test_missing_url_is_rejected(self, default_config: WebsocketConfig) -> None:
        empty_config = dataclass_replace(default_config, uri="")
        provider = WebsocketTTSProvider(
            logger=None,
            config_getter=lambda: empty_config,
            connector_factory=lambda *_args, **_kwargs: StubConnection([]),
            player_factory=lambda *_args, **_kwargs: StubPlayer(),
        )

        with pytest.raises(ProviderUnavailable):
            provider.speak("hello", volume=1.0, on_duration=lambda _ms: None)


def dataclass_replace(config: WebsocketConfig, **updates: Any) -> WebsocketConfig:
    data = config.__dict__.copy()
    data.update(updates)
    return WebsocketConfig(**data)
