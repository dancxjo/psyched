"""Tests for ear backends and workers."""

from __future__ import annotations

import asyncio
import base64
import json
import io
import sys
import threading
from collections.abc import Callable
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

import pytest

from ear.backends import (
    BackendUnavailableError,
    ConsoleEarBackend,
    ServiceASREarBackend,
    TranscriptionEvent,
)
from ear.worker import EarWorker


class _PublishCollector:
    """Helper that records published transcript events for assertions."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._items: list[TranscriptionEvent] = []

    def __call__(self, event: TranscriptionEvent) -> None:
        with self._lock:
            self._items.append(event)

    @property
    def items(self) -> list[TranscriptionEvent]:
        with self._lock:
            return list(self._items)


def test_console_backend_emits_non_empty_lines() -> None:
    """Console backend should strip whitespace and ignore blank lines."""

    input_stream = io.StringIO(" Hello\n\nworld  \n   \nBYE\n")
    output_stream = io.StringIO()
    backend = ConsoleEarBackend(input_stream=input_stream, output_stream=output_stream)
    stop_event = threading.Event()
    published: list[TranscriptionEvent] = []

    backend.run(published.append, stop_event)

    assert [event.text for event in published] == ["Hello", "world", "BYE"]
    prompt_text = output_stream.getvalue()
    assert "\u001b[" in prompt_text  # ANSI color prefix
    assert prompt_text.endswith("\u001b[0m")


class _BlockingBackend:
    """Backend used to verify worker start/stop semantics."""

    def __init__(self) -> None:
        self.stop_event: threading.Event | None = None
        self.started = threading.Event()
        self._publish: Callable[[TranscriptionEvent], None] | None = None

    def run(
        self,
        publish: Callable[[TranscriptionEvent], None],
        stop_event: threading.Event,
    ) -> None:  # pragma: no cover - interface
        self._publish = publish
        self.stop_event = stop_event
        self.started.set()
        stop_event.wait()
        publish(TranscriptionEvent.final("done"))


def test_worker_stops_backend_on_shutdown() -> None:
    """EarWorker should signal stop and wait for the backend to finish."""

    backend = _BlockingBackend()
    collector = _PublishCollector()
    worker = EarWorker(backend=backend, publisher=collector, logger=None)

    worker.start()
    backend.started.wait(timeout=1)
    assert backend.started.is_set()

    worker.stop()

    assert collector.items and collector.items[0].text == "done"
    assert backend.stop_event is not None and backend.stop_event.is_set()


class _StubFallbackBackend:
    """Audio-aware backend used to assert fallback behaviour."""

    def __init__(self) -> None:
        self.submissions: list[tuple[bytes, int, int]] = []
        self.run_started = threading.Event()
        self.closed = threading.Event()

    def submit_audio(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        self.submissions.append((pcm, sample_rate, channels))

    def close(self) -> None:  # pragma: no cover - trivial setter
        self.closed.set()

    def run(
        self,
        publish: Callable[[TranscriptionEvent], None],
        stop_event: threading.Event,
    ) -> None:
        self.run_started.set()
        while not stop_event.is_set():
            stop_event.wait(0.01)


def test_service_backend_falls_back_when_service_unavailable(monkeypatch: pytest.MonkeyPatch) -> None:
    """The service backend should activate the fallback when `_transcribe` cannot connect."""

    async def _raise_backend_unavailable(
        self: ServiceASREarBackend,
        publish: Callable[[TranscriptionEvent], None],
        stop_event: threading.Event,
    ) -> None:
        raise BackendUnavailableError("simulated outage")

    monkeypatch.setattr(
        ServiceASREarBackend,
        "_transcribe",
        _raise_backend_unavailable,
        raising=False,
    )

    fallback_backend = _StubFallbackBackend()
    backend = ServiceASREarBackend(
        uri="ws://127.0.0.1:65535/asr",
        connect_timeout=0.01,
        fallback_factory=lambda: fallback_backend,
    )

    # Audio submitted before the backend starts should be replayed into the fallback backend.
    backend.submit_audio(b"first", 16000, 1)
    collector = _PublishCollector()
    stop_event = threading.Event()
    thread = threading.Thread(
        target=backend.run,
        args=(collector, stop_event),
        name="ServiceBackendTest",
        daemon=True,
    )
    thread.start()

    assert fallback_backend.run_started.wait(timeout=1.0)

    backend.submit_audio(b"second", 16000, 1)
    stop_event.set()
    backend.close()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert fallback_backend.submissions == [
        (b"first", 16000, 1),
        (b"second", 16000, 1),
    ]
    assert fallback_backend.closed.is_set()


def test_service_backend_processes_partial_and_final_payloads() -> None:
    """Service backend should publish deduplicated partial and final transcripts."""

    backend = ServiceASREarBackend(uri="ws://127.0.0.1:65535/asr")
    collector = _PublishCollector()

    partial_payload = json.dumps(
        {
            "event": "partial",
            "segments": [
                {"text": "hello"},
                {"text": "world"},
            ],
        },
    )
    backend._process_service_message(partial_payload, collector)
    # Duplicate partial payloads should be ignored.
    backend._process_service_message(partial_payload, collector)

    final_payload = json.dumps(
        {
            "event": "final",
            "text": "hello world!",
            "segments": [
                {"text": "hello"},
                {"text": "world!"},
            ],
            "start_ms": 0,
            "end_ms": 1500,
        },
    )
    backend._process_service_message(final_payload, collector)

    assert len(collector.items) == 2


def test_service_backend_encodes_pcm_frames() -> None:
    """PCM chunks should be base64 encoded with intact payload metadata."""

    backend = ServiceASREarBackend(uri="ws://127.0.0.1:65535/asr")
    pcm = b"\x01\x02\x03\x04"
    backend._queue.put((pcm, 22050, 2))
    backend._queue.put(None)

    class _FakeWebSocket:
        def __init__(self) -> None:
            self.sent: list[str] = []

        async def send(self, payload: str) -> None:
            self.sent.append(payload)

    fake_ws = _FakeWebSocket()
    stop_event = threading.Event()
    asyncio.run(backend._pump_audio_chunks(fake_ws, stop_event))

    assert fake_ws.sent, "No websocket payloads were emitted"
    payload = json.loads(fake_ws.sent[0])
    assert payload["type"] == "chunk"
    assert payload["sample_rate"] == 22050
    assert payload["channels"] == 2
    decoded = base64.b64decode(payload["pcm"])
    assert decoded == pcm


def test_service_backend_emits_silence_when_queue_is_idle() -> None:
    """Silence padding should keep the websocket stream active between utterances."""

    backend = ServiceASREarBackend(uri="ws://127.0.0.1:65535/asr")
    backend.submit_audio(b"\x00\x00", 16000, 1)

    stop_event = threading.Event()

    class _CollectingWebSocket:
        def __init__(self) -> None:
            self.sent: list[dict[str, object]] = []

        async def send(self, payload: str) -> None:
            self.sent.append(json.loads(payload))
            if len(self.sent) >= 2:
                stop_event.set()
                backend._queue.put(None)

    fake_ws = _CollectingWebSocket()

    async def _run() -> None:
        await backend._pump_audio_chunks(fake_ws, stop_event)

    asyncio.run(asyncio.wait_for(_run(), timeout=1.0))

    assert len(fake_ws.sent) >= 2, "Expected a silence frame to be synthesized after the live chunk"
    silence_payload = fake_ws.sent[1]
    assert silence_payload["sample_rate"] == 16000
    assert silence_payload["channels"] == 1
    silence_bytes = base64.b64decode(silence_payload["pcm"])
    assert silence_bytes == b"\x00\x00"
