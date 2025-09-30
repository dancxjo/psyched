from __future__ import annotations

import threading
import time
from collections import deque
from pathlib import Path
import sys
from typing import Callable, Deque, Iterable, List, Sequence

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from ear.pyaudio_ear_node import ByteMultiArray, PyAudioEarNode


class _FakeStdout:
    """Minimal file-like object returning predetermined audio chunks."""

    def __init__(self, stop_ref: List[threading.Event], chunks: Iterable[bytes]) -> None:
        self._stop_ref = stop_ref
        self._chunks: Deque[bytes] = deque(chunks)

    def read(self, size: int) -> bytes:
        if self._chunks:
            return self._chunks.popleft()
        if self._stop_ref and self._stop_ref[0].wait(0.05):
            return b""
        time.sleep(0.01)
        return b""

    def close(self) -> None:  # pragma: no cover - compatibility shim
        pass


class _FakeProcess:
    """Subprocess stand-in exposing the subset used by the node."""

    def __init__(self, stdout: _FakeStdout) -> None:
        self.stdout = stdout
        self._terminated = False

    def poll(self) -> int | None:
        return 0 if self._terminated else None

    def terminate(self) -> None:
        self._terminated = True

    def kill(self) -> None:
        self._terminated = True

    def wait(self, timeout: float | None = None) -> int:
        self._terminated = True
        return 0


class _RecordingFactory:
    """Capture the commands issued to ``subprocess.Popen``."""

    def __init__(self, stop_ref: List[threading.Event], chunks: Sequence[bytes]) -> None:
        self.commands: List[Sequence[str]] = []
        self._stop_ref = stop_ref
        self._chunks = chunks

    def __call__(self, cmd: Sequence[str], **_: object) -> _FakeProcess:
        self.commands.append(tuple(cmd))
        return _FakeProcess(_FakeStdout(self._stop_ref, self._chunks))


def _create_node(
    *,
    overrides: dict[str, object] | None = None,
    factory_builder: Callable[[List[threading.Event]], Callable[..., _FakeProcess]] | None = None,
) -> PyAudioEarNode:
    """Helper constructing a node with deterministic subprocess behaviour."""

    stop_ref: List[threading.Event] = []

    if factory_builder is None:
        chunks = (b"\x01\x02\x03\x04", b"")
        recording_factory = _RecordingFactory(stop_ref, chunks)
        factory = recording_factory
    else:
        recording_factory = factory_builder(stop_ref)
        factory = recording_factory

    node = PyAudioEarNode(
        popen_factory=factory,
        parameter_overrides=overrides or {},
    )
    stop_ref.append(node._stop_evt)  # type: ignore[attr-defined]
    node._factory = recording_factory  # type: ignore[attr-defined]
    return node


def test_arecord_node_publishes_audio_frames() -> None:
    """Chunks from arecord should be forwarded to ``/audio/raw``."""
    node = _create_node()
    publisher = node.audio_pub  # type: ignore[attr-defined]

    for _ in range(50):
        if getattr(publisher, "published", []):
            break
        time.sleep(0.02)

    assert publisher.published, "expected at least one published frame"
    first = publisher.published[0]
    assert isinstance(first, ByteMultiArray)
    assert first.data == b"\x01\x02\x03\x04"

    node.destroy_node()


def test_arecord_node_uses_device_id_for_hw_mapping() -> None:
    """An integer ``device_id`` should resolve to ``hw:<id>,0`` for ALSA."""

    def build_factory(stop_ref: List[threading.Event]) -> _RecordingFactory:
        return _RecordingFactory(stop_ref, (b"\x00\x00", b""))

    node = _create_node(overrides={"device_id": 3, "channels": 2}, factory_builder=build_factory)
    factory = getattr(node, "_factory")
    assert factory is not None

    for _ in range(50):
        if factory.commands:
            break
        time.sleep(0.02)

    assert factory.commands, "arecord should have been spawned"
    cmd = factory.commands[0]
    assert "arecord" in cmd[0]
    assert "-D" in cmd
    device_index = cmd.index("-D") + 1
    assert cmd[device_index] == "hw:3,0"
    channel_index = cmd.index("-c") + 1
    assert cmd[channel_index] == "2"

    node.destroy_node()
