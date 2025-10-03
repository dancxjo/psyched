"""Behavioural tests for :mod:`voice.queue`."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Optional

from voice.exceptions import SpeechInterrupted
from voice.queue import SpeechQueue


class EventDrivenBackend:
    """Speech backend used for testing queue coordination."""

    def __init__(self) -> None:
        self.started: list[str] = []
        self.completed: list[str] = []
        self.interruptions = 0
        self._events: deque[threading.Event] = deque()
        self._lock = threading.Lock()

    def queue_release(self) -> threading.Event:
        event = threading.Event()
        with self._lock:
            self._events.append(event)
        return event

    def _peek_event(self) -> threading.Event:
        with self._lock:
            if not self._events:
                self._events.append(threading.Event())
            return self._events[0]

    def _complete_event(self) -> None:
        with self._lock:
            if self._events:
                self._events.popleft()

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.started.append(text)
        event = self._peek_event()
        while not event.is_set():
            if stop_event.wait(0.01):
                self.interruptions += 1
                raise SpeechInterrupted()
        self.completed.append(text)
        self._complete_event()
        if progress_callback:
            progress_callback(text)


class RecordingBackend:
    """Backend that records invocations without blocking."""

    def __init__(self) -> None:
        self.calls: list[str] = []

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[Callable[[str], None]] = None,
    ) -> None:
        if stop_event.is_set():
            raise SpeechInterrupted()
        self.calls.append(text)
        if progress_callback:
            progress_callback(text)


def wait_for(predicate: Callable[[], bool], *, timeout: float = 1.0) -> None:
    """Wait until ``predicate`` returns ``True`` or raise ``AssertionError``."""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    if not predicate():  # pragma: no cover - defensive branch
        raise AssertionError("condition not met before timeout")


def test_queue_processes_messages_in_order() -> None:
    backend = RecordingBackend()
    spoken: list[str] = []
    queue = SpeechQueue(backend, on_spoken=spoken.append)
    try:
        queue.enqueue("one")
        queue.enqueue("two")
        assert queue.join(timeout=1.0)
        assert backend.calls == ["one", "two"]
        assert spoken == ["one", "two"]
    finally:
        queue.shutdown()


def test_pause_requeues_current_text_until_resumed() -> None:
    backend = EventDrivenBackend()
    release_event = backend.queue_release()
    spoken: list[str] = []
    queue = SpeechQueue(backend, on_spoken=spoken.append)
    try:
        queue.enqueue("alpha")
        wait_for(lambda: len(backend.started) == 1)
        queue.pause()
        wait_for(lambda: backend.interruptions == 1)
        assert spoken == []
        queue.resume()
        wait_for(lambda: len(backend.started) == 2)
        release_event.set()
        wait_for(lambda: spoken == ["alpha"])
    finally:
        queue.shutdown()


def test_clear_stops_current_and_discards_pending() -> None:
    backend = EventDrivenBackend()
    backend.queue_release()
    spoken: list[str] = []
    queue = SpeechQueue(backend, on_spoken=spoken.append)
    try:
        queue.enqueue("beta")
        wait_for(lambda: len(backend.started) == 1)
        queue.clear()
        wait_for(lambda: backend.interruptions == 1)
        assert queue.join(timeout=1.0)
        assert spoken == []
    finally:
        queue.shutdown()

