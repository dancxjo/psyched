"""Shared data structures and helpers for ASR transcription results."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Mapping, Optional, Sequence, Tuple
import re


__all__ = [
    "SegmentData",
    "WordData",
    "TranscriptionResult",
    "looks_like_placeholder_transcript",
    "result_is_usable",
    "segments_and_words_from_payload",
    "words_from_sequence",
    "coalesce_words",
    "approximate_words_from_segments",
]


@dataclass(frozen=True)
class SegmentData:
    """Lightweight representation of a transcript segment."""

    start: float
    end: float
    text: str
    speaker: Optional[str] = None


@dataclass(frozen=True)
class WordData:
    """Word-level timing metadata."""

    start: float
    end: float
    text: str


@dataclass(frozen=True)
class TranscriptionResult:
    """Bundle of recognised text and its alignment metadata."""

    text: str
    confidence: float
    segments: List[SegmentData]
    words: List[WordData]


_PLACEHOLDER_TRANSCRIPT_RE = re.compile(r"^samples=\d+(?:\.\d+)?\s+sum=-?\d+(?:\.\d+)?$")


def looks_like_placeholder_transcript(text: str) -> bool:
    """Return ``True`` when *text* matches a known placeholder pattern."""

    candidate = text.strip().lower()
    if not candidate:
        return False
    return _PLACEHOLDER_TRANSCRIPT_RE.match(candidate) is not None


def result_is_usable(result: Optional[TranscriptionResult]) -> bool:
    """Determine whether *result* contains meaningful recognised text."""

    if result is None:
        return False
    text = str(getattr(result, "text", "") or "").strip()
    if not text:
        return False
    if looks_like_placeholder_transcript(text):
        return False
    return True


def _coerce_float(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _word_from_mapping(
    word: Mapping[str, object],
    fallback_start: float,
    fallback_end: float,
) -> Optional[WordData]:
    text_raw = word.get("text", word.get("word", ""))
    text = str(text_raw or "").strip()
    if not text:
        return None
    start = _coerce_float(word.get("start", word.get("t0")), fallback_start)
    end = _coerce_float(word.get("end", word.get("t1")), start)
    if end < start:
        end = start
    return WordData(start=start, end=end, text=text)


def words_from_sequence(
    words: object,
    fallback_start: float,
    fallback_end: float,
) -> List[WordData]:
    if not isinstance(words, Sequence) or isinstance(words, (str, bytes, bytearray)):
        return []
    result: List[WordData] = []
    last_end = fallback_start
    for word in words:
        if isinstance(word, Mapping):
            candidate = _word_from_mapping(word, last_end, fallback_end)
        else:
            text = str(getattr(word, "word", getattr(word, "text", "")) or "").strip()
            if not text:
                continue
            start = _coerce_float(
                getattr(word, "start", getattr(word, "t0", last_end)),
                last_end,
            )
            end = _coerce_float(
                getattr(word, "end", getattr(word, "t1", start)),
                start,
            )
            if end < start:
                end = start
            candidate = WordData(start=start, end=end, text=text)
        if candidate is None:
            continue
        result.append(candidate)
        last_end = max(last_end, candidate.end)
    return result


def segments_and_words_from_payload(
    raw_segments: object,
    *,
    default_speaker: Optional[str] = None,
) -> Tuple[List[SegmentData], List[WordData]]:
    if not isinstance(raw_segments, Sequence) or isinstance(raw_segments, (str, bytes, bytearray)):
        return [], []

    segments: List[SegmentData] = []
    words: List[WordData] = []
    for entry in raw_segments:
        if isinstance(entry, Mapping):
            text_raw = entry.get("text", "")
            speaker_raw = entry.get("speaker", default_speaker)
            start = _coerce_float(entry.get("start", entry.get("t0")), 0.0)
            end = _coerce_float(entry.get("end", entry.get("t1")), start)
            if end < start:
                end = start
            text = str(text_raw or "").strip()
            speaker = str(speaker_raw) if speaker_raw else None
            segments.append(
                SegmentData(start=start, end=end, text=text, speaker=speaker)
            )
            words.extend(words_from_sequence(entry.get("words"), start, end))
        else:
            text = str(getattr(entry, "text", "") or "").strip()
            start = _coerce_float(getattr(entry, "start", getattr(entry, "t0", 0.0)), 0.0)
            end = _coerce_float(getattr(entry, "end", getattr(entry, "t1", start)), start)
            if end < start:
                end = start
            speaker_attr = getattr(entry, "speaker", default_speaker)
            speaker = str(speaker_attr) if speaker_attr else None
            segments.append(
                SegmentData(start=start, end=end, text=text, speaker=speaker)
            )
            words.extend(
                words_from_sequence(getattr(entry, "words", []), start, end)
            )
    return segments, words


def approximate_words_from_segments(
    text: str,
    segments: Sequence[SegmentData],
) -> List[WordData]:
    words: List[WordData] = []
    for segment in segments:
        tokens = [token for token in segment.text.split() if token]
        if not tokens:
            continue
        duration = max(0.0, segment.end - segment.start)
        if duration <= 0.0:
            cursor = segment.start
            for token in tokens:
                words.append(WordData(start=cursor, end=cursor, text=token))
            continue
        step = duration / max(1, len(tokens))
        cursor = segment.start
        for index, token in enumerate(tokens):
            start = cursor
            if index == len(tokens) - 1:
                end = segment.end
            else:
                end = min(segment.end, cursor + step)
            words.append(WordData(start=start, end=end, text=token))
            cursor = end
    if words:
        return words
    tokens = [token for token in text.split() if token]
    cursor = 0.0
    for token in tokens:
        start = cursor
        end = cursor + 0.4
        words.append(WordData(start=start, end=end, text=token))
        cursor = end
    return words


def coalesce_words(
    text: str,
    segments: Sequence[SegmentData],
    words: Sequence[WordData],
) -> List[WordData]:
    collected = [WordData(start=w.start, end=w.end, text=w.text) for w in words if w.text]
    if collected:
        return collected
    return approximate_words_from_segments(text, segments)
