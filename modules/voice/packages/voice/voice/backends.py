"""Speech backend implementations.

Two backends are provided:

* :class:`PrintSpeechBackend` writes spoken text to a stream (stdout by
  default). It is useful for testing or systems without a speech synthesizer.
* :class:`EspeakSpeechBackend` wraps the ``espeak`` command line utility.

Both backends implement the :class:`SpeechBackend` protocol and cooperate with
:class:`~voice.queue.SpeechQueue` through the
:class:`~voice.exceptions.SpeechInterrupted` exception.
"""

from __future__ import annotations

import logging
import shutil
import subprocess
import sys
import threading
from dataclasses import dataclass
from typing import Callable, Optional, Protocol, Sequence, TextIO

from .exceptions import SpeechInterrupted

_LOGGER = logging.getLogger(__name__)

ProgressCallback = Callable[[str], None]


class SpeechBackend(Protocol):
    """Protocol implemented by speech backends.

    Backends must block until speech playback completes or raise
    :class:`SpeechInterrupted` when the supplied stop event is triggered.
    """

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> None:
        """Play ``text`` using the backend.

        Args:
            text: The text that should be spoken.
            stop_event: Event set when playback should stop immediately.
            progress_callback: Optional callback invoked with the spoken text
                once playback finishes.
        """


@dataclass(slots=True)
class PrintSpeechBackend:
    """Speech backend that prints text to a stream.

    Example
    -------
    >>> backend = PrintSpeechBackend()
    >>> backend.speak("Hello", threading.Event())
    Hello
    """

    stream: TextIO = sys.stdout

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> None:
        if stop_event.is_set():
            raise SpeechInterrupted("stopped before print")
        print(text, file=self.stream, flush=True)
        if progress_callback:
            progress_callback(text)


class EspeakSpeechBackend:
    """Speech backend powered by the ``espeak`` command line utility.

    Parameters
    ----------
    command:
        Sequence representing the base command. Defaults to ``["espeak"]``.
    voice:
        Optional voice identifier passed to ``espeak`` via ``-v``.
    rate:
        Optional speech rate passed to ``espeak`` via ``-s``.
    """

    def __init__(
        self,
        command: Sequence[str] | None = None,
        *,
        voice: str | None = None,
        rate: int | None = None,
    ) -> None:
        base_command = list(command) if command else ["espeak"]
        if not base_command:
            raise ValueError("command must contain at least one element")
        executable = shutil.which(base_command[0])
        if executable is None:
            raise FileNotFoundError(
                f"Unable to locate speech synthesizer '{base_command[0]}'",
            )
        self._command = [executable, *base_command[1:]]
        self._voice = voice
        self._rate = rate

    def _build_command(self) -> list[str]:
        command = [*self._command, "--stdin"]
        if self._voice:
            command.extend(["-v", self._voice])
        if self._rate is not None:
            command.extend(["-s", str(self._rate)])
        return command

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> None:
        process = subprocess.Popen(
            self._build_command(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        assert process.stdin is not None  # pragma: no cover - for type checkers

        try:
            process.stdin.write(text)
            process.stdin.close()
        except Exception:  # pragma: no cover - defensive logging
            _LOGGER.exception("Failed to send text to espeak")
            process.kill()
            raise

        try:
            while True:
                if stop_event.wait(timeout=0.05):
                    _LOGGER.debug("Stopping espeak process early")
                    process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()
                    raise SpeechInterrupted("stop requested")
                return_code = process.poll()
                if return_code is not None:
                    if return_code != 0:
                        stderr_output = process.stderr.read() if process.stderr else ""
                        raise RuntimeError(
                            f"espeak exited with code {return_code}: {stderr_output.strip()}",
                        )
                    if progress_callback:
                        progress_callback(text)
                    return
        finally:
            if process.stdout:
                process.stdout.close()
            if process.stderr:
                process.stderr.close()

