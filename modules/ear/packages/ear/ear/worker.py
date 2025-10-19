"""Utility for running ear backends in a managed thread."""

from __future__ import annotations

import logging
import threading
import traceback
from collections.abc import Callable

from .backends import AudioAwareBackend, EarBackend

PublishCallback = Callable[[str], None]


class EarWorker:
    """Run a transcription backend on a background thread."""

    def __init__(
        self,
        *,
        backend: EarBackend,
        publisher: PublishCallback,
        logger: logging.Logger | None,
    ) -> None:
        self._backend = backend
        self._publisher = publisher
        self._logger = logger or logging.getLogger(__name__)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._started = False
        self._lock = threading.Lock()

    def start(self) -> None:
        """Start the worker thread if it is not already running."""

        with self._lock:
            if self._started:
                return
            self._thread = threading.Thread(target=self._run, name="EarWorker", daemon=True)
            self._thread.start()
            self._started = True

    def stop(self) -> None:
        """Signal the backend to stop and wait for the thread to exit."""

        with self._lock:
            if not self._started:
                return
            self._stop_event.set()
            if isinstance(self._backend, AudioAwareBackend):
                try:
                    self._backend.close()
                except Exception:  # pragma: no cover - defensive logging
                    self._log_exception("Failed to close audio backend cleanly")
            thread = self._thread
            if thread is not None:
                thread.join(timeout=5)
            self._thread = None
            self._started = False
            self._stop_event = threading.Event()

    def _run(self) -> None:
        try:
            self._backend.run(self._publisher, self._stop_event)
        except Exception:  # pragma: no cover - defensive logging
            self._log_exception("Ear backend terminated with an error")
        finally:
            with self._lock:
                self._started = False

    def submit_audio(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Submit PCM audio to the backend if supported."""

        if isinstance(self._backend, AudioAwareBackend):
            self._backend.submit_audio(pcm, sample_rate, channels)
        else:
            self._logger.warning(f"Backend {self._backend} does not accept audio")

    def _log_exception(self, message: str) -> None:
        """Report exceptions through both ROS and stdlib loggers."""

        if hasattr(self._logger, "exception"):
            self._logger.exception(message)
            return

        formatted_traceback = traceback.format_exc()
        self._logger.error(f"{message}\n{formatted_traceback}")
