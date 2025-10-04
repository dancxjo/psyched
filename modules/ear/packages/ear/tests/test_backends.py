"""Tests for ear backends and workers."""

from __future__ import annotations

import io
import threading
from collections.abc import Callable

import pytest

from ear.backends import ConsoleEarBackend
from ear.worker import EarWorker


class _PublishCollector:
    """Helper that records published transcripts for assertions."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._items: list[str] = []

    def __call__(self, text: str) -> None:
        with self._lock:
            self._items.append(text)

    @property
    def items(self) -> list[str]:
        with self._lock:
            return list(self._items)


def test_console_backend_emits_non_empty_lines() -> None:
    """Console backend should strip whitespace and ignore blank lines."""

    input_stream = io.StringIO(" Hello\n\nworld  \n   \nBYE\n")
    output_stream = io.StringIO()
    backend = ConsoleEarBackend(input_stream=input_stream, output_stream=output_stream)
    stop_event = threading.Event()
    published: list[str] = []

    backend.run(published.append, stop_event)

    assert published == ["Hello", "world", "BYE"]
    prompt_text = output_stream.getvalue()
    assert "\u001b[" in prompt_text  # ANSI color prefix
    assert prompt_text.endswith("\u001b[0m")


class _BlockingBackend:
    """Backend used to verify worker start/stop semantics."""

    def __init__(self) -> None:
        self.stop_event: threading.Event | None = None
        self.started = threading.Event()
        self._publish: Callable[[str], None] | None = None

    def run(self, publish: Callable[[str], None], stop_event: threading.Event) -> None:  # pragma: no cover - interface
        self._publish = publish
        self.stop_event = stop_event
        self.started.set()
        stop_event.wait()
        publish("done")


def test_worker_stops_backend_on_shutdown() -> None:
    """EarWorker should signal stop and wait for the backend to finish."""

    backend = _BlockingBackend()
    collector = _PublishCollector()
    worker = EarWorker(backend=backend, publisher=collector, logger=None)

    worker.start()
    backend.started.wait(timeout=1)
    assert backend.started.is_set()

    worker.stop()

    assert collector.items == ["done"]
    assert backend.stop_event is not None and backend.stop_event.is_set()
