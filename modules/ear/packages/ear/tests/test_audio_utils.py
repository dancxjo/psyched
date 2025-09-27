"""Tests for audio utility helpers."""
from pathlib import Path
import sys

package_root = str(Path(__file__).resolve().parents[1])
if package_root not in sys.path:
    sys.path.insert(0, package_root)

from ear.audio_utils import coerce_pcm_bytes


def test_coerce_from_sequence_of_ints():
    data = [0, 255, 128]
    assert coerce_pcm_bytes(data) == bytes(data)


def test_coerce_from_bytes_like_objects():
    payload = bytearray(b"abc")
    assert coerce_pcm_bytes(payload) == b"abc"
    view = memoryview(b"xyz")
    assert coerce_pcm_bytes(view) == b"xyz"


class DummyArray:
    """Array-like object exposing ``tolist`` (mimics numpy arrays)."""

    def __init__(self, values):
        self._values = list(values)

    def tolist(self):
        return list(self._values)


def test_coerce_from_array_like():
    arr = DummyArray([1, 2, 3])
    assert coerce_pcm_bytes(arr) == b"\x01\x02\x03"
