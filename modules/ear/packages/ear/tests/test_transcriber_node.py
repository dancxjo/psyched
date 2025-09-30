import base64
import json
import types
from dataclasses import dataclass
from typing import List

import pytest

from ear.transcription_backends import (
    ChainedTranscriptionBackend,
    RemoteAsrBackend,
    _load_websocket_dependencies,
)
from ear.transcription_pipeline import TranscriptPublisher, TranscriptionPipeline
from ear.transcription_types import SegmentData, TranscriptionResult, WordData
from ear.transcription_worker import TranscriptionWorker
from ear.transcriber_long_node import LongTranscriberApp
from ear.transcriber_medium_node import MediumTranscriberApp
from ear.transcriber_short_node import ShortTranscriberApp


class DummyBackend:
    def __init__(self, response: TranscriptionResult | None):
        self.response = response
        self.calls: List[bytes] = []

    def transcribe(self, pcm_bytes: bytes, sample_rate: int):
        self.calls.append(pcm_bytes)
        return self.response


class DummyFailingBackend(DummyBackend):
    def __init__(self, response: TranscriptionResult | None):
        super().__init__(response)
        self.failures = 0

    def transcribe(self, pcm_bytes: bytes, sample_rate: int):
        self.calls.append(pcm_bytes)
        self.failures += 1
        raise RuntimeError("remote backend offline")


class DummyBackendNoText(DummyBackend):
    def __init__(self):
        result = TranscriptionResult(
            text=" ",
            confidence=0.0,
            segments=[],
            words=[],
        )
        super().__init__(response=result)


@dataclass
class FakeLogRecord:
    level: str
    message: str


class FakeLogger:
    def __init__(self) -> None:
        self.records: List[FakeLogRecord] = []

    def info(self, message: str, *args: object) -> None:
        self.records.append(FakeLogRecord("info", message % args if args else message))

    def warning(self, message: str, *args: object) -> None:
        self.records.append(FakeLogRecord("warning", message % args if args else message))

    def error(self, message: str, *args: object) -> None:
        self.records.append(FakeLogRecord("error", message % args if args else message))


class FakePublisher:
    def __init__(self) -> None:
        self.messages: List[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class FakeSubscription:
    def __init__(self, topic: str, callback):
        self.topic = topic
        self.callback = callback


class FakeNode:
    def __init__(self, parameters: dict[str, object] | None = None) -> None:
        self.parameters = parameters or {}
        self.publishers: dict[str, FakePublisher] = {}
        self.subscriptions: list[FakeSubscription] = []
        self.logger = FakeLogger()

    def declare_parameter(self, name: str, default_value: object):
        value = self.parameters.get(name, default_value)
        return types.SimpleNamespace(value=value)

    def create_publisher(self, _msg_type, topic: str, _queue_size: int):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, _msg_type, topic: str, callback, _queue_size: int):
        subscription = FakeSubscription(topic, callback)
        self.subscriptions.append(subscription)
        return subscription

    def get_logger(self) -> FakeLogger:
        return self.logger


def test_transcriber_worker_emits_transcript():
    backend = DummyBackend(
        TranscriptionResult(
            text="hello world",
            confidence=0.8,
            segments=[SegmentData(start=0.0, end=1.0, text="hello world", speaker="user")],
            words=[
                WordData(start=0.0, end=0.5, text="hello"),
                WordData(start=0.5, end=1.0, text="world"),
            ],
        )
    )
    captured: List[TranscriptionResult] = []

    worker = TranscriptionWorker(
        backend=backend,
        sample_rate=16000,
        speaker='user',
        on_result=captured.append,
    )

    worker.handle_segment(b'\x00\x01')

    assert backend.calls, "Backend should receive audio bytes"
    assert len(captured) == 1
    result = captured[0]
    assert result.text == "hello world"
    assert result.confidence == 0.8
    assert result.segments == [SegmentData(start=0.0, end=1.0, text="hello world", speaker="user")]
    assert result.words == [
        WordData(start=0.0, end=0.5, text="hello"),
        WordData(start=0.5, end=1.0, text="world"),
    ]


def test_transcriber_worker_skips_blank_transcripts():
    backend = DummyBackendNoText()
    captured: List[TranscriptionResult] = []

    worker = TranscriptionWorker(
        backend=backend,
        sample_rate=16000,
        speaker='user',
        on_result=captured.append,
    )

    worker.handle_segment(b'\x00\x00')

    assert not captured


def test_chained_backend_prefers_primary():
    primary = DummyBackend(
        TranscriptionResult(
            text="primary",
            confidence=0.9,
            segments=[],
            words=[],
        )
    )
    fallback = DummyBackend(
        TranscriptionResult(
            text="fallback",
            confidence=0.5,
            segments=[],
            words=[],
        )
    )

    backend = ChainedTranscriptionBackend(primary=primary, fallback=fallback, cooldown_seconds=1.0, logger=None)

    result = backend.transcribe(b"pcm", 16000)

    assert isinstance(result, TranscriptionResult)
    assert result.text == "primary"
    assert result.confidence == 0.9
    assert len(primary.calls) == 1
    assert not fallback.calls


def test_chained_backend_falls_back_after_failure():
    primary = DummyFailingBackend(
        TranscriptionResult(text="primary", confidence=0.9, segments=[], words=[])
    )
    fallback = DummyBackend(
        TranscriptionResult(text="fallback", confidence=0.5, segments=[], words=[])
    )

    backend = ChainedTranscriptionBackend(primary=primary, fallback=fallback, cooldown_seconds=10.0, logger=None)

    result = backend.transcribe(b"pcm", 16000)

    assert isinstance(result, TranscriptionResult)
    assert result.text == "fallback"
    assert result.confidence == 0.5
    assert len(fallback.calls) == 1
    assert primary.failures == 1


def test_chained_backend_skips_primary_during_cooldown():
    primary = DummyFailingBackend(
        TranscriptionResult(text="primary", confidence=0.9, segments=[], words=[])
    )
    fallback = DummyBackend(
        TranscriptionResult(text="fallback", confidence=0.5, segments=[], words=[])
    )

    fake_now = [100.0]

    def fake_monotonic() -> float:
        return fake_now[0]

    backend = ChainedTranscriptionBackend(
        primary=primary,
        fallback=fallback,
        cooldown_seconds=30.0,
        logger=None,
        monotonic=fake_monotonic,
    )

    first = backend.transcribe(b"pcm", 16000)
    assert isinstance(first, TranscriptionResult)
    assert first.text == "fallback"
    assert first.confidence == 0.5
    assert primary.failures == 1

    fake_now[0] += 5.0
    second = backend.transcribe(b"pcm2", 16000)

    assert isinstance(second, TranscriptionResult)
    assert second.text == "fallback"
    assert second.confidence == 0.5
    assert primary.failures == 1, "primary should not be retried during cooldown"
    assert len(fallback.calls) == 2


def test_chained_backend_retries_fallback_when_primary_returns_placeholder():
    primary = DummyBackend(
        TranscriptionResult(text="samples=4 sum=6", confidence=0.3, segments=[], words=[])
    )
    fallback = DummyBackend(
        TranscriptionResult(text="resolved text", confidence=0.8, segments=[], words=[])
    )

    fake_now = [42.0]

    def fake_monotonic() -> float:
        return fake_now[0]

    backend = ChainedTranscriptionBackend(
        primary=primary,
        fallback=fallback,
        cooldown_seconds=30.0,
        logger=None,
        monotonic=fake_monotonic,
    )

    first = backend.transcribe(b"pcm", 16000)

    assert isinstance(first, TranscriptionResult)
    assert first.text == "resolved text"
    assert len(primary.calls) == 1
    assert len(fallback.calls) == 1

    fake_now[0] += 5.0
    second = backend.transcribe(b"pcm-more", 16000)

    assert isinstance(second, TranscriptionResult)
    assert second.text == "resolved text"
    assert len(primary.calls) == 1, "primary should be skipped while placeholder cooldown active"
    assert len(fallback.calls) == 2


def test_load_websocket_dependencies_prefers_asyncio_namespace():
    class DummyConnectionClosed(Exception):
        """Sentinel exception used to emulate websockets' ConnectionClosed."""

    modules = {
        "websockets.asyncio.client": types.SimpleNamespace(connect="async-connect"),
        "websockets.exceptions": types.SimpleNamespace(ConnectionClosed=DummyConnectionClosed),
    }

    def fake_import(name: str):
        if name not in modules:
            raise ImportError(name)
        return modules[name]

    connect, connection_closed = _load_websocket_dependencies(fake_import)

    assert connect == "async-connect"
    assert connection_closed is DummyConnectionClosed


def test_load_websocket_dependencies_falls_back_to_client_module():
    class DummyConnectionClosed(Exception):
        """Sentinel exception used to emulate websockets' ConnectionClosed."""

    modules = {
        "websockets.client": types.SimpleNamespace(connect="client-connect"),
        "websockets.exceptions": types.SimpleNamespace(ConnectionClosed=DummyConnectionClosed),
    }

    def fake_import(name: str):
        if name not in modules:
            raise ImportError(name)
        return modules[name]

    connect, connection_closed = _load_websocket_dependencies(fake_import)

    assert connect == "client-connect"
    assert connection_closed is DummyConnectionClosed


def test_load_websocket_dependencies_handles_missing_modules():
    def fake_import(name: str):  # pragma: no cover - invoked for completeness
        raise ImportError(name)

    connect, connection_closed = _load_websocket_dependencies(fake_import)

    assert connect is None
    assert connection_closed is RuntimeError


def test_transcript_publisher_emits_segments_and_words():
    publisher = TranscriptPublisher(
        publishers=[FakePublisher()],
        speaker_label="user",
        include_timing=True,
        include_words=True,
    )

    result = TranscriptionResult(
        text="refined speech",
        confidence=0.75,
        segments=[
            SegmentData(start=0.0, end=1.0, text="refined", speaker="user"),
            SegmentData(start=1.0, end=2.0, text="speech", speaker="user"),
        ],
        words=[
            WordData(start=0.0, end=0.5, text="refined"),
            WordData(start=0.5, end=1.0, text="speech"),
        ],
    )

    publisher.publish(result)

    assert len(publisher.publishers[0].messages) == 1
    message = publisher.publishers[0].messages[0]
    assert message.text == "refined speech"
    assert message.speaker == "user"
    assert pytest.approx(message.confidence, rel=1e-6) == 0.75
    assert [segment.text for segment in message.segments] == ["refined", "speech"]
    assert [word.text for word in message.words] == ["refined", "speech"]


def test_transcription_pipeline_processes_audio_synchronously():
    backend = DummyBackend(
        TranscriptionResult(
            text="hello",
            confidence=0.9,
            segments=[SegmentData(start=0.0, end=1.0, text="hello", speaker="user")],
            words=[WordData(start=0.0, end=1.0, text="hello")],
        )
    )
    publisher = TranscriptPublisher(
        publishers=[FakePublisher()],
        speaker_label="user",
        include_timing=False,
        include_words=False,
    )

    pipeline = TranscriptionPipeline(
        tier="short",
        backend=backend,
        sample_rate=16000,
        publisher=publisher,
        logger=FakeLogger(),
        keep_latest=True,
        start_worker=False,
    )

    pipeline.enqueue(b"data")

    assert backend.calls == [b"data"]
    assert len(publisher.publishers[0].messages) == 1
    assert publisher.publishers[0].messages[0].text == "hello"


def test_remote_backend_returns_refine_transcripts():
    refine_payload = json.dumps(
        {
            "type": "refine",
            "text": "refined speech",
            "segments": [
                {
                    "start": 0.0,
                    "end": 1.0,
                    "text": "refined",
                    "speaker": "user",
                    "words": [{"start": 0.0, "end": 0.5, "text": "refined"}],
                },
                {
                    "start": 1.0,
                    "end": 2.0,
                    "text": "speech",
                    "speaker": "user",
                },
            ],
            "words": [
                {"start": 1.0, "end": 1.6, "text": "speech"},
            ],
        }
    )

    class DummyWebSocket:
        def __init__(self, responses):
            self._responses = iter(responses)
            self.sent_messages: List[str] = []

        async def __aenter__(self):
            return self

        async def __aexit__(self, exc_type, exc, tb):
            return False

        async def send(self, message: str):
            self.sent_messages.append(message)

        async def recv(self) -> str:
            try:
                return next(self._responses)
            except StopIteration as exc:  # pragma: no cover - defensive; loop exits on refine
                raise RuntimeError("no more responses") from exc

    websocket = DummyWebSocket([
        json.dumps({"type": "stats", "rtf": 0.5, "queue_len": 1, "gpu": False}),
        refine_payload,
    ])

    class DummyConnectorContext:
        def __init__(self, socket: DummyWebSocket):
            self._socket = socket

        async def __aenter__(self) -> DummyWebSocket:
            return self._socket

        async def __aexit__(self, exc_type, exc, tb) -> bool:
            return False

    def dummy_connector(uri: str, **_: object) -> DummyConnectorContext:
        assert uri == "ws://example.test/ws"
        return DummyConnectorContext(websocket)

    def fake_monotonic() -> float:
        fake_monotonic.current += 0.1
        return fake_monotonic.current

    fake_monotonic.current = 0.0

    backend = RemoteAsrBackend(
        uri="ws://example.test/ws",
        language=None,
        connect_timeout=0.5,
        response_timeout=1.0,
        logger=None,
        connector=dummy_connector,
        monotonic=fake_monotonic,
    )

    result = backend.transcribe(b"\x00\x01", 16000)

    assert len(websocket.sent_messages) == 3
    init_message = json.loads(websocket.sent_messages[0])
    audio_message = json.loads(websocket.sent_messages[1])
    commit_message = json.loads(websocket.sent_messages[2])

    assert init_message["type"] == "init"
    assert init_message["stream_id"].startswith("ear-")
    assert init_message["content_type"] == "audio/pcm; rate=16000"
    assert init_message["sample_rate"] == 16000
    assert init_message["extras"]["chunk_id"] == commit_message["chunk_id"]
    assert audio_message == {
        "type": "audio",
        "stream_id": init_message["stream_id"],
        "seq": 0,
        "payload_b64": base64.b64encode(b"\x00\x01").decode("ascii"),
    }
    assert commit_message["type"] == "commit"
    assert commit_message["stream_id"] == init_message["stream_id"]

    assert isinstance(result, TranscriptionResult)
    assert result.text == "refined speech"
    assert result.confidence == 0.0
    assert [segment.text for segment in result.segments] == ["refined", "speech"]
    assert result.segments[0].start == 0.0
    assert result.segments[1].end == 2.0
    assert [word.text for word in result.words] == ["refined", "speech"]


def _build_dummy_result() -> TranscriptionResult:
    return TranscriptionResult(
        text="dummy",
        confidence=0.6,
        segments=[SegmentData(start=0.0, end=1.0, text="dummy", speaker="user")],
        words=[WordData(start=0.0, end=1.0, text="dummy")],
    )


def _simulate_subscription(fake_node: FakeNode, data: bytes) -> None:
    assert fake_node.subscriptions, "node should create a subscription"
    subscription = fake_node.subscriptions[0]
    msg = types.SimpleNamespace(data=list(data))
    subscription.callback(msg)


def test_short_transcriber_routes_to_short_topic():
    fake_node = FakeNode()
    backend = DummyBackend(_build_dummy_result())
    app = ShortTranscriberApp(fake_node, backend=backend, start_worker=False)

    assert fake_node.subscriptions[0].topic == "/audio/speech_segment_accumulating"
    _simulate_subscription(fake_node, b"ab")

    short_pub = fake_node.publishers["/audio/transcript/short"]
    assert len(short_pub.messages) == 1
    assert short_pub.messages[0].text == "dummy"


def test_medium_transcriber_broadcasts_to_general_topic():
    fake_node = FakeNode()
    backend = DummyBackend(_build_dummy_result())
    app = MediumTranscriberApp(fake_node, backend=backend, start_worker=False)

    assert fake_node.subscriptions[0].topic == "/audio/speech_segment"
    _simulate_subscription(fake_node, b"cd")

    medium_pub = fake_node.publishers["/audio/transcript/medium"]
    general_pub = fake_node.publishers["/audio/transcription"]
    assert len(medium_pub.messages) == 1
    assert len(general_pub.messages) == 1
    assert medium_pub.messages[0] is general_pub.messages[0]


def test_long_transcriber_routes_to_long_topic():
    fake_node = FakeNode()
    backend = DummyBackend(_build_dummy_result())
    app = LongTranscriberApp(fake_node, backend=backend, start_worker=False)

    assert fake_node.subscriptions[0].topic == "/audio/speech_accumulating"
    _simulate_subscription(fake_node, b"ef")

    long_pub = fake_node.publishers["/audio/transcript/long"]
    assert len(long_pub.messages) == 1
    assert long_pub.messages[0].text == "dummy"
