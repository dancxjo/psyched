import asyncio
import base64
from typing import Any

import pytest

pytest.importorskip("aiohttp")
rclpy = pytest.importorskip("rclpy")
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from ear.backends import TranscriptionEvent
from ear.browser_server import BridgeConfig, create_app


class _StubRelay:
    def __init__(self) -> None:
        self.transcripts: list[str] = []
        self.events: list[dict[str, object]] = []
        self.eye_payloads: list[dict[str, object]] = []

    def publish_transcription(self, event: TranscriptionEvent) -> None:
        self.events.append(event.to_dict())
        if not event.is_partial:
            self.transcripts.append(event.text)

    def publish_eye_frame(self, payload: dict[str, object]) -> None:
        self.eye_payloads.append(dict(payload))


class _StubBackend:
    def __init__(self, text: str = "hello world") -> None:
        self.text = text
        self.publish = None
        self.stop_event: asyncio.Event | None = None

    def run(self, publish, stop_event) -> None:  # type: ignore[override]
        self.publish = publish
        self.stop_event = stop_event
        stop_event.wait()

    def close(self) -> None:
        if self.stop_event:
            self.stop_event.set()

    def submit_audio(self, _pcm: bytes, _sample_rate: int, _channels: int) -> None:
        if self.publish:
            self.publish(TranscriptionEvent.final(self.text))


@pytest.mark.asyncio
async def test_ear_websocket_forwards_transcripts() -> None:
    relay = _StubRelay()
    app = create_app(
        config=BridgeConfig(),
        relay=relay,  # type: ignore[arg-type]
        backend_factory=lambda: _StubBackend("browser test"),
    )
    server = TestServer(app)
    async with server:
        client = TestClient(server)
        async with client:
            ws = await client.ws_connect("/ear/stream")
            await ws.send_json(
                {
                    "type": "chunk",
                    "sample_rate": 16000,
                    "channels": 1,
                    "pcm": base64.b64encode(b"1234").decode("ascii"),
                }
            )
            message = await ws.receive_json(timeout=2)
            assert message["event"] == "final"
            assert "browser test" in relay.transcripts[-1]
            await ws.close()


@pytest.mark.asyncio
async def test_eye_clip_endpoint_relays_payload() -> None:
    relay = _StubRelay()
    app = create_app(config=BridgeConfig(), relay=relay, backend_factory=lambda: _StubBackend())
    server = TestServer(app)
    async with server:
        client = TestClient(server)
        async with client:
            payload = {
                "mime_type": "video/webm",
                "bytes": 8,
                "captured_at": "2024-01-01T00:00:00Z",
                "data": base64.b64encode(b"clipdata").decode("ascii"),
            }
            response = await client.post("/eye/clip", json=payload)
            assert response.status == 200
            assert relay.eye_payloads
            assert relay.eye_payloads[-1]["mime_type"] == "video/webm"
            body = await response.json()
            assert body["published"] is True


@pytest.mark.asyncio
async def test_eye_websocket_streams_frames() -> None:
    relay = _StubRelay()
    app = create_app(config=BridgeConfig(), relay=relay, backend_factory=lambda: _StubBackend())
    server = TestServer(app)
    async with server:
        client = TestClient(server)
        async with client:
            ws = await client.ws_connect("/eye/stream")
            payload = {
                "type": "frame",
                "mime_type": "video/webm",
                "bytes": 4,
                "data": base64.b64encode(b"test").decode("ascii"),
                "captured_at": "2024-01-01T00:00:00Z",
            }
            await ws.send_json(payload)
            response = await ws.receive_json()
            assert response["event"] == "ack"
            assert relay.eye_payloads
            assert relay.eye_payloads[-1]["sequence"] == 1
            await ws.close()
