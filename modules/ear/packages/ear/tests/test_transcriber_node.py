from pathlib import Path
import sys
from typing import List, Tuple

package_root = str(Path(__file__).resolve().parents[1])
if package_root not in sys.path:
    sys.path.insert(0, package_root)

from ear.transcriber_node import ChainedTranscriptionBackend, TranscriptionWorker


class DummyBackend:
    def __init__(self, response: Tuple[str, float]):
        self.response = response
        self.calls: List[bytes] = []

    def transcribe(self, pcm_bytes: bytes, sample_rate: int):
        self.calls.append(pcm_bytes)
        return self.response


class DummyFailingBackend(DummyBackend):
    def __init__(self, response: Tuple[str, float]):
        super().__init__(response)
        self.failures = 0

    def transcribe(self, pcm_bytes: bytes, sample_rate: int):
        self.calls.append(pcm_bytes)
        self.failures += 1
        raise RuntimeError("remote backend offline")


class DummyBackendNoText(DummyBackend):
    def __init__(self):
        super().__init__(response=(" ", 0.0))


def test_transcriber_worker_emits_transcript():
    backend = DummyBackend(("hello world", 0.8))
    captured: List[Tuple[str, float]] = []

    worker = TranscriptionWorker(
        backend=backend,
        sample_rate=16000,
        speaker='user',
        on_result=lambda text, conf: captured.append((text, conf)),
    )

    worker.handle_segment(b'\x00\x01')

    assert backend.calls, "Backend should receive audio bytes"
    assert captured == [("hello world", 0.8)]


def test_transcriber_worker_skips_blank_transcripts():
    backend = DummyBackendNoText()
    captured: List[Tuple[str, float]] = []

    worker = TranscriptionWorker(
        backend=backend,
        sample_rate=16000,
        speaker='user',
        on_result=lambda text, conf: captured.append((text, conf)),
    )

    worker.handle_segment(b'\x00\x00')

    assert not captured


def test_chained_backend_prefers_primary():
    primary = DummyBackend(("primary", 0.9))
    fallback = DummyBackend(("fallback", 0.5))

    backend = ChainedTranscriptionBackend(primary=primary, fallback=fallback, cooldown_seconds=1.0, logger=None)

    result = backend.transcribe(b"pcm", 16000)

    assert result == ("primary", 0.9)
    assert len(primary.calls) == 1
    assert not fallback.calls


def test_chained_backend_falls_back_after_failure():
    primary = DummyFailingBackend(("primary", 0.9))
    fallback = DummyBackend(("fallback", 0.5))

    backend = ChainedTranscriptionBackend(primary=primary, fallback=fallback, cooldown_seconds=10.0, logger=None)

    result = backend.transcribe(b"pcm", 16000)

    assert result == ("fallback", 0.5)
    assert len(fallback.calls) == 1
    assert primary.failures == 1


def test_chained_backend_skips_primary_during_cooldown():
    primary = DummyFailingBackend(("primary", 0.9))
    fallback = DummyBackend(("fallback", 0.5))

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
    assert first == ("fallback", 0.5)
    assert primary.failures == 1

    fake_now[0] += 5.0
    second = backend.transcribe(b"pcm2", 16000)

    assert second == ("fallback", 0.5)
    assert primary.failures == 1, "primary should not be retried during cooldown"
    assert len(fallback.calls) == 2

