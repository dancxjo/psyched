"""Behavioural specs for the websocket TTS streaming helpers."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.append(str(Path(__file__).resolve().parents[3]))

from tools.tts_websocket.websocket_server import (
    SynthesisRequest,
    TTSSynthesizer,
    chunk_pcm_bytes,
    encode_pcm16,
    parse_client_message,
)


def test_parse_client_message_accepts_json_payload_with_overrides():
    """JSON payloads should override the default speaker and language."""
    request = parse_client_message(
        '{"text": "Hello", "speaker": "p330", "language": "en"}',
        default_speaker="p330",
        default_language="en-us",
    )

    assert request == SynthesisRequest(text="Hello", speaker="p330", language="en")


def test_parse_client_message_accepts_plain_text_and_uses_defaults():
    """Plain text messages should fall back to the default speaker."""
    request = parse_client_message("Streaming is quick!", default_speaker="p330", default_language=None)

    assert request == SynthesisRequest(text="Streaming is quick!", speaker="p330", language=None)


@pytest.mark.parametrize(
    "samples, expected",
    [
        (np.array([-1.5, -1.0, 0.0, 0.5, 1.0, 1.5], dtype=np.float32), np.array([-32767, -32767, 0, 16383, 32767, 32767], dtype="<i2")),
        (np.array([], dtype=np.float32), np.array([], dtype="<i2")),
    ],
)
def test_encode_pcm16_clips_and_scales_to_int16(samples: np.ndarray, expected: np.ndarray) -> None:
    """PCM encoding should clip to [-1, 1] and scale to 16-bit integers."""
    pcm = encode_pcm16(samples)

    assert pcm == expected.tobytes()


def test_chunk_pcm_bytes_yields_fixed_size_frames() -> None:
    """Chunk helper should respect the requested frame size."""
    raw = bytes(range(64))
    chunks = list(chunk_pcm_bytes(raw, chunk_samples=8))

    # 8 samples * 2 bytes/sample -> 16 bytes per frame.
    assert all(len(chunk) == 16 for chunk in chunks[:-1])
    assert len(chunks) == 4


def test_tts_synthesizer_wraps_tts_api_and_flattens_audio() -> None:
    """The wrapper should flatten list outputs and expose the sample rate."""

    class DummyTTS:
        def __init__(self) -> None:
            self.synthesizer = type("Synth", (), {"output_sample_rate": 22050})()
            self.arguments = None

        def tts(self, text: str, speaker=None, language=None):  # noqa: D401, ANN001 - mimic Coqui signature.
            """Return a deterministic waveform for testing."""
            self.arguments = (text, speaker, language)
            return [0.0, 0.25, -0.25]

    wrapper = TTSSynthesizer(DummyTTS())
    request = SynthesisRequest(text="Speed matters", speaker="p330", language="en")

    audio = wrapper.synthesize(request)

    assert wrapper.sample_rate == 22050
    assert audio.dtype == np.float32
    assert audio.tolist() == [0.0, 0.25, -0.25]
    # We intentionally omit the language hint when calling the Coqui API so
    # mono-lingual models keep working even if clients send a language.
    assert wrapper.tts.arguments == ("Speed matters", "p330", None)
