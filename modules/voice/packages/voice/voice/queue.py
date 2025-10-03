"""Queue management for serialized speech playback."""

from __future__ import annotations

import collections
import logging
import threading
from collections.abc import Callable
from typing import Optional

from .backends import SpeechBackend
from .exceptions import SpeechInterrupted

_LOGGER = logging.getLogger(__name__)

InterruptReason = str


class SpeechQueue:
    """Coordinate serialized speech playback on a background thread.

    The queue owns a worker thread that requests speech playback from the
    configured :class:`~voice.backends.SpeechBackend`. Incoming text is enqueued
    and processed one entry at a time. Control hooks allow callers to pause,
    resume, clear, or shut down the queue.

    Parameters
    ----------
    backend:
        Speech backend responsible for turning text into audio.
    on_spoken:
        Callback invoked with the text after playback completes successfully.
    """

    def __init__(
        self,
        backend: SpeechBackend,
        *,
        on_spoken: Callable[[str], None],
    ) -> None:
        self._backend = backend
        self._on_spoken = on_spoken
        self._queue: collections.deque[str] = collections.deque()
        self._lock = threading.Condition()
        self._paused = False
        self._shutdown = False
        self._current_text: Optional[str] = None
        self._current_stop_event: Optional[threading.Event] = None
        self._interrupt_reason: InterruptReason | None = None
        self._idle_event = threading.Event()
        self._idle_event.set()
        self._worker = threading.Thread(
            target=self._worker_loop,
            name="SpeechQueueWorker",
            daemon=True,
        )
        self._worker.start()

    def enqueue(self, text: str) -> None:
        """Add ``text`` to the playback queue."""
        with self._lock:
            self._queue.append(text)
            self._idle_event.clear()
            self._lock.notify_all()

    def pause(self) -> None:
        """Pause playback, requeueing the active utterance if necessary."""
        with self._lock:
            self._paused = True
            if self._current_stop_event and not self._current_stop_event.is_set():
                self._interrupt_reason = "pause"
                self._current_stop_event.set()

    def resume(self) -> None:
        """Resume playback if it was previously paused."""
        with self._lock:
            if not self._paused:
                return
            self._paused = False
            if not self._queue and self._current_text is None:
                self._idle_event.set()
            self._lock.notify_all()

    def clear(self) -> None:
        """Remove pending utterances and stop the current one."""
        with self._lock:
            self._queue.clear()
            if self._current_stop_event and not self._current_stop_event.is_set():
                self._interrupt_reason = "clear"
                self._current_stop_event.set()
            if self._current_text is None:
                self._idle_event.set()

    def join(self, timeout: float | None = None) -> bool:
        """Block until the queue is idle.

        Args:
            timeout: Maximum time to wait in seconds. ``None`` waits indefinitely.

        Returns
        -------
        bool
            ``True`` if the queue became idle before the timeout elapsed.
        """

        return self._idle_event.wait(timeout=timeout)

    def shutdown(self) -> None:
        """Terminate the worker thread and release resources."""
        with self._lock:
            if self._shutdown:
                return
            self._shutdown = True
            self._queue.clear()
            if self._current_stop_event and not self._current_stop_event.is_set():
                self._interrupt_reason = "shutdown"
                self._current_stop_event.set()
            self._lock.notify_all()
        self._worker.join()

    def _worker_loop(self) -> None:
        while True:
            with self._lock:
                while not self._shutdown and (self._paused or not self._queue):
                    if not self._queue and self._current_text is None and not self._paused:
                        self._idle_event.set()
                    self._lock.wait()
                if self._shutdown:
                    break
                text = self._queue.popleft()
                stop_event = threading.Event()
                self._current_text = text
                self._current_stop_event = stop_event
                self._interrupt_reason = None
                self._idle_event.clear()
            interrupted = False
            try:
                self._backend.speak(text, stop_event, None)
            except SpeechInterrupted:
                interrupted = True
            except Exception:  # pragma: no cover - defensive logging
                _LOGGER.exception("Speech backend failed for text: %s", text)
            else:
                try:
                    self._on_spoken(text)
                except Exception:  # pragma: no cover - avoid killing worker
                    _LOGGER.exception("on_spoken callback raised for text: %s", text)
            finally:
                with self._lock:
                    if interrupted and self._interrupt_reason == "pause":
                        self._queue.appendleft(text)
                        self._idle_event.clear()
                    self._current_text = None
                    self._current_stop_event = None
                    self._interrupt_reason = None
                    if not self._queue and not self._paused:
                        self._idle_event.set()
                    self._lock.notify_all()
            if self._shutdown:
                break

    close = shutdown

