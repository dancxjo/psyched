"""Helpers that adapt transcription backends into callables used by nodes."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from .transcription_backends import load_backend
from .transcription_types import (
    SegmentData,
    TranscriptionResult,
    approximate_words_from_segments,
    result_is_usable,
)


__all__ = ["TranscriptionWorker", "load_backend"]


@dataclass
class TranscriptionWorker:
    """Synchronous helper that wraps a backend ASR engine."""

    backend: Optional[object]
    sample_rate: int
    speaker: str
    on_result: Callable[[TranscriptionResult], None]
    logger: Optional[object] = None

    def handle_segment(self, pcm_bytes: bytes) -> None:
        if not pcm_bytes:
            return
        if self.backend is None:
            if self.logger:
                self.logger.warning("No ASR backend configured; dropping audio segment")
            return
        try:
            result = self.backend.transcribe(pcm_bytes, self.sample_rate)
        except Exception as exc:
            if self.logger:
                self.logger.error(f"ASR backend raised an error: {exc}")
            return
        normalised = self._normalise_result(result)
        if normalised is None:
            return
        self.on_result(normalised)

    def _normalise_result(self, result: object) -> Optional[TranscriptionResult]:
        if result is None:
            return None
        if isinstance(result, TranscriptionResult):
            if not result.text.strip():
                return None
            return result
        if isinstance(result, tuple) and len(result) >= 2:
            text = str(result[0] or "").strip()
            if not text:
                return None
            try:
                confidence = float(result[1])
            except Exception:
                confidence = 0.0
            segment = SegmentData(start=0.0, end=0.0, text=text, speaker=str(self.speaker))
            words = approximate_words_from_segments(text, [segment])
            candidate = TranscriptionResult(
                text=text,
                confidence=confidence,
                segments=[segment],
                words=words,
            )
            if not result_is_usable(candidate):
                return None
            return candidate
        if self.logger:
            self.logger.warning(f"Unexpected transcription result type: {type(result)!r}")
        return None
