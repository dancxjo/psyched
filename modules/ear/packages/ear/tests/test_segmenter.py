"""Behaviour-driven tests for the speech segmentation helpers."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

package_root = str(Path(__file__).resolve().parents[1])
if package_root not in sys.path:
    sys.path.insert(0, package_root)

from ear.segmenter_node import SegmentUpdate, SpeechSegmenter  # noqa: E402


@dataclass
class FakeVadFrame:
    """Minimal stand-in for :class:`psyched_msgs.msg.VadFrame` used in tests."""

    stamp: float
    sample_rate: int
    frame_samples: int
    is_speech: bool
    audio: bytes


def _pcm(value: int, samples: int) -> bytes:
    return (value.to_bytes(2, "little", signed=True)) * samples


def test_segmenter_emits_segment_when_voice_transitions_to_silence() -> None:
    """A continuous voiced run should emit a segment when silence arrives."""
    segmenter = SpeechSegmenter()

    frames = [
        FakeVadFrame(stamp=float(index) * 0.03, sample_rate=16000, frame_samples=480, is_speech=True, audio=_pcm(1000, 480))
        for index in range(3)
    ]
    frames.append(
        FakeVadFrame(stamp=0.09, sample_rate=16000, frame_samples=480, is_speech=False, audio=_pcm(0, 480))
    )

    updates = [segmenter.process_frame(frame) for frame in frames]

    # The duration should increase for speech frames and reset once silence arrives.
    assert [update.duration_ms for update in updates] == [30, 60, 90, 0]

    # The final update should contain the full concatenated PCM payload.
    assert isinstance(updates[-1], SegmentUpdate)
    assert updates[-1].segment == b"".join(frame.audio for frame in frames[:3])


def test_segmenter_disregards_short_noise_bursts_between_speech() -> None:
    """A single silent frame between speech should close the first segment and start a new one."""
    segmenter = SpeechSegmenter()

    speech_a = FakeVadFrame(0.0, 16000, 480, True, _pcm(1200, 480))
    silence = FakeVadFrame(0.03, 16000, 480, False, _pcm(0, 480))
    speech_b = FakeVadFrame(0.06, 16000, 480, True, _pcm(800, 480))

    first = segmenter.process_frame(speech_a)
    second = segmenter.process_frame(silence)
    third = segmenter.process_frame(speech_b)

    assert first.segment is None
    assert second.segment == speech_a.audio
    # A new speech burst should restart the duration counter.
    assert third.duration_ms == 30
    assert third.segment is None


def test_segmenter_handles_consecutive_silence_without_active_segment() -> None:
    """Silence-only frames should keep the duration at zero without emitting segments."""
    segmenter = SpeechSegmenter()

    silent_frames = [
        FakeVadFrame(0.0, 16000, 480, False, _pcm(0, 480)),
        FakeVadFrame(0.03, 16000, 480, False, _pcm(0, 480)),
    ]

    updates = [segmenter.process_frame(frame) for frame in silent_frames]

    assert all(update.duration_ms == 0 for update in updates)
    assert all(update.segment is None for update in updates)
