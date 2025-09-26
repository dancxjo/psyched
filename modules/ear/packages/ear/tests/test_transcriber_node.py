from pathlib import Path
import sys
from typing import List, Tuple

package_root = str(Path(__file__).resolve().parents[1])
if package_root not in sys.path:
    sys.path.insert(0, package_root)

from ear.transcriber_node import TranscriptionWorker


class DummyBackend:
    def __init__(self, response: Tuple[str, float]):
        self.response = response
        self.calls: List[bytes] = []

    def transcribe(self, pcm_bytes: bytes, sample_rate: int):
        self.calls.append(pcm_bytes)
        return self.response


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
