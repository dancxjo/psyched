"""Execution pipeline for tiered transcription nodes."""
from __future__ import annotations

import queue
import threading
from dataclasses import dataclass, field
from typing import List, Optional

from .transcription_types import TranscriptionResult, result_is_usable

try:  # pragma: no cover - only available within a ROS workspace
    from psyched_msgs.msg import Transcript, TranscriptSegment as MsgTranscriptSegment, TranscriptWord as MsgTranscriptWord
except ImportError:  # pragma: no cover - provide light-weight stubs for tests
    @dataclass
    class MsgTranscriptSegment:
        start: float = 0.0
        end: float = 0.0
        text: str = ""
        speaker: str = ""

    @dataclass
    class MsgTranscriptWord:
        start: float = 0.0
        end: float = 0.0
        text: str = ""

    @dataclass
    class Transcript:
        text: str = ""
        speaker: str = ""
        confidence: float = 0.0
        segments: List[MsgTranscriptSegment] = field(default_factory=list)
        words: List[MsgTranscriptWord] = field(default_factory=list)


__all__ = ["TranscriptPublisher", "TranscriptionPipeline", "Transcript", "MsgTranscriptSegment", "MsgTranscriptWord"]


class TranscriptPublisher:
    """Normalise :class:`TranscriptionResult` values into ROS messages."""

    def __init__(
        self,
        *,
        publishers: Sequence[object],
        speaker_label: str,
        include_timing: bool,
        include_words: bool,
    ) -> None:
        self.publishers = list(publishers)
        self._speaker_label = speaker_label
        self._include_timing = include_timing
        self._include_words = include_words

    def publish(self, result: TranscriptionResult) -> None:
        message = self._build_message(result)
        for publisher in self.publishers:
            publisher.publish(message)

    def _build_message(self, result: TranscriptionResult) -> Transcript:
        msg = Transcript()
        msg.text = result.text
        segment_speaker = next((segment.speaker for segment in result.segments if segment.speaker), None)
        msg.speaker = segment_speaker or str(self._speaker_label)
        msg.confidence = float(result.confidence)

        if self._include_timing:
            msg.segments = []
            for segment in result.segments:
                seg_msg = MsgTranscriptSegment()
                seg_msg.start = float(segment.start)
                seg_msg.end = float(segment.end)
                seg_msg.text = segment.text
                seg_msg.speaker = segment.speaker or str(self._speaker_label)
                msg.segments.append(seg_msg)
        else:
            msg.segments = []  # type: ignore[assignment]

        if self._include_words:
            msg.words = []
            for word in result.words:
                word_msg = MsgTranscriptWord()
                word_msg.start = float(word.start)
                word_msg.end = float(word.end)
                word_msg.text = word.text
                msg.words.append(word_msg)
        else:
            msg.words = []  # type: ignore[assignment]
        return msg


class TranscriptionPipeline:
    """Offload transcription work to a worker thread and publish results."""

    def __init__(
        self,
        *,
        tier: str,
        backend: Optional[object],
        sample_rate: int,
        publisher: TranscriptPublisher,
        logger=None,
        keep_latest: bool = False,
        start_worker: bool = True,
    ) -> None:
        self._tier = tier
        self._backend = backend
        self._sample_rate = int(sample_rate)
        self._publisher = publisher
        self._logger = logger
        self._keep_latest = keep_latest
        self._warned_backend_missing = False
        self._use_thread = bool(start_worker)
        self._stop_evt = threading.Event()
        self._queue: "queue.Queue[Optional[bytes]]" = queue.Queue(maxsize=1 if keep_latest else 0)
        self._thread: Optional[threading.Thread] = None
        if self._use_thread:
            self._thread = threading.Thread(target=self._worker_loop, daemon=True)
            self._thread.start()

    def enqueue(self, pcm_bytes: bytes) -> None:
        if not pcm_bytes:
            return
        if not self._use_thread:
            self._process(pcm_bytes)
            return
        try:
            self._queue.put_nowait(pcm_bytes)
        except queue.Full:
            if self._keep_latest:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                self._queue.put_nowait(pcm_bytes)
            elif self._logger:
                self._logger.warning(f"Dropping {self._tier} audio: queue full")

    def stop(self) -> None:
        if self._use_thread:
            self._stop_evt.set()
            try:
                self._queue.put_nowait(None)
            except Exception:
                pass
            if self._thread is not None:
                self._thread.join(timeout=2.0)

    def _worker_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            self._process(item)

    def _process(self, pcm_bytes: bytes) -> None:
        if self._backend is None:
            if not self._warned_backend_missing and self._logger:
                self._logger.warning(
                    f"No ASR backend configured for {self._tier} tier; dropping audio"
                )
            self._warned_backend_missing = True
            return
        try:
            result = self._backend.transcribe(pcm_bytes, self._sample_rate)  # type: ignore[attr-defined]
        except Exception as exc:
            if self._logger:
                self._logger.warning(
                    f"Backend {self._tier} raised error: {exc}"
                )
            return
        if not result_is_usable(result):
            return
        try:
            self._publisher.publish(result)  # type: ignore[arg-type]
        except Exception as exc:
            if self._logger:
                self._logger.error("Failed to publish %s transcript: %s", self._tier, exc)
