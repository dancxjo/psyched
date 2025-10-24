"""Unit tests for speech backend implementations."""

from __future__ import annotations

import io
import json
import socket
import threading
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Callable, Iterable, Iterator, List

import pytest

from voice.backends import FailoverSpeechBackend, SpeechInterrupted, WebsocketTTSSpeechBackend


@dataclass
class _FakeWebsocket:
    """Simple stand-in for a websocket client connection."""

    responses: Iterator[object]
    sent_messages: List[object]

    def send(self, message: object) -> None:
        self.sent_messages.append(message)

    def recv(self) -> object:
        return next(self.responses)

    def close(self) -> None:  # pragma: no cover - required by context manager
        pass


def _websocket_context(responses: Iterable[object], sent: List[object]):
    """Create a context manager returning a fake websocket connection."""

    @contextmanager
    def factory():
        ws = _FakeWebsocket(iter(responses), sent)
        try:
            yield ws
        finally:
            ws.close()

    return factory


class _RecordingStream(io.BytesIO):
    """BytesIO variant that keeps data accessible after ``close``."""

    def close(self) -> None:  # pragma: no cover - trivial override
        self.flush()


class _FakeProcess:
    """Pretend :class:`subprocess.Popen` instance writing to an in-memory sink."""

    def __init__(self) -> None:
        self.stdin = _RecordingStream()
        self.terminated = False
        self.waited = False
        self.killed = False

    def terminate(self) -> None:
        self.terminated = True

    def wait(self, timeout: float | None = None) -> None:
        self.waited = True

    def kill(self) -> None:
        self.killed = True


def _process_factory(process: _FakeProcess):
    def factory(sample_rate: int, channels: int) -> _FakeProcess:  # pragma: no cover - simple wrapper
        return process

    return factory


class _RecordingBackend:
    """Test helper that records texts passed to ``speak``."""

    def __init__(self) -> None:
        self.spoken: list[str] = []

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Callable[[str], None] | None = None,
    ) -> None:
        self.spoken.append(text)
        if progress_callback:
            progress_callback(text)


def test_websocket_backend_streams_audio_and_reports_progress() -> None:
    """The backend should send requests, stream PCM frames, and call back."""

    responses = [
        json.dumps(
            {
                "event": "start",
                "sample_rate": 22050,
                "channels": 1,
                "format": "pcm_s16le",
                "num_samples": 4,
            }
        ),
        b"\x01\x02\x03\x04",
        json.dumps({"event": "end", "num_samples": 4, "duration_s": 4 / 22050}),
    ]
    sent: List[object] = []
    process = _FakeProcess()
    backend = WebsocketTTSSpeechBackend(
        url="ws://tts.local:5002/tts",
        speaker="demo",
        language="en",
        connection_factory=_websocket_context(responses, sent),
        player_factory=_process_factory(process),
    )
    stop_event = threading.Event()
    progress: list[str] = []

    backend.speak("Hello", stop_event, progress.append)

    assert sent[0] == json.dumps({"text": "Hello", "speaker": "demo", "language": "en"})
    assert process.stdin.getvalue() == b"\x01\x02\x03\x04"
    assert progress == ["Hello"]


def test_websocket_backend_honours_stop_event() -> None:
    """Setting the stop event during playback should interrupt synthesis."""

    def _response_iter(stop: threading.Event) -> Iterator[object]:
        yield json.dumps(
            {
                "event": "start",
                "sample_rate": 16000,
                "channels": 1,
                "format": "pcm_s16le",
                "num_samples": 2,
            }
        )
        stop.set()
        yield b"\x01\x02"
        yield json.dumps({"event": "end", "num_samples": 2, "duration_s": 2 / 16000})

    stop_event = threading.Event()
    responses = _response_iter(stop_event)
    sent: List[object] = []
    process = _FakeProcess()
    backend = WebsocketTTSSpeechBackend(
        connection_factory=_websocket_context(responses, sent),
        player_factory=_process_factory(process),
    )

    with pytest.raises(SpeechInterrupted):
        backend.speak("stop me", stop_event)

    assert process.terminated


def test_websocket_backend_raises_on_service_error() -> None:
    """The backend should surface server-side error messages."""

    responses = [json.dumps({"event": "error", "message": "boom"})]
    sent: List[object] = []
    process = _FakeProcess()
    backend = WebsocketTTSSpeechBackend(
        connection_factory=_websocket_context(responses, sent),
        player_factory=_process_factory(process),
    )

    with pytest.raises(RuntimeError, match="boom"):
        backend.speak("oops", threading.Event())


def test_failover_backend_switches_to_fallback_on_connection_error() -> None:
    """Connection failures trigger the fallback backend and stick permanently."""

    class _FailingBackend:
        def __init__(self) -> None:
            self.calls = 0

        def speak(
            self,
            text: str,
            stop_event: threading.Event,
            progress_callback: Callable[[str], None] | None = None,
        ) -> None:
            self.calls += 1
            raise ConnectionRefusedError("no transport")

    primary = _FailingBackend()
    fallback = _RecordingBackend()
    backend = FailoverSpeechBackend(primary, fallback)
    progress: list[str] = []
    backend.speak("hello", threading.Event(), progress.append)

    assert primary.calls == 1
    assert fallback.spoken == ["hello"]
    assert progress == ["hello"]

    backend.speak("again", threading.Event())
    assert primary.calls == 1  # primary no longer used
    assert fallback.spoken[-1] == "again"


def test_failover_backend_switches_to_fallback_on_socket_error() -> None:
    """Socket-level failures should also activate the fallback backend."""

    class _FailingBackend:
        def __init__(self) -> None:
            self.calls = 0

        def speak(
            self,
            text: str,
            stop_event: threading.Event,
            progress_callback: Callable[[str], None] | None = None,
        ) -> None:
            self.calls += 1
            raise socket.gaierror(socket.EAI_NONAME, "name or service not known")

    primary = _FailingBackend()
    fallback = _RecordingBackend()
    backend = FailoverSpeechBackend(primary, fallback)

    backend.speak("dns failure", threading.Event())

    assert primary.calls == 1
    assert fallback.spoken == ["dns failure"]

    backend.speak("still fallback", threading.Event())
    assert primary.calls == 1
    assert fallback.spoken[-1] == "still fallback"

def test_failover_backend_propagates_interrupts() -> None:
    """Speech interruptions should bubble up without activating the fallback."""

    class _InterruptingBackend:
        def speak(
            self,
            text: str,
            stop_event: threading.Event,
            progress_callback: Callable[[str], None] | None = None,
        ) -> None:
            raise SpeechInterrupted("stop requested")

    fallback = _RecordingBackend()
    backend = FailoverSpeechBackend(_InterruptingBackend(), fallback)

    with pytest.raises(SpeechInterrupted):
        backend.speak("hello", threading.Event())

    assert fallback.spoken == []
