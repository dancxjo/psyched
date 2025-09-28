"""Utilities for tracking consecutive silence durations.

The :class:`SilenceTracker` encapsulates the logic that powers the
``/audio/silence_ms`` gauge so it can be validated independently of the
ROS 2 node.  It accepts RMS measurements along with optional autophony
signals and produces the number of milliseconds that have elapsed since
the input stream last contained non-silent audio.

Example
-------
>>> tracker = SilenceTracker(silence_threshold=0.5)
>>> tracker.update(rms=0.1, timestamp=0.0)
0
>>> tracker.update(rms=0.1, timestamp=0.5)
500
>>> tracker.update(rms=1.0, timestamp=0.6)
0
>>> tracker.update(rms=0.1, timestamp=1.0)
0
>>> tracker.update(rms=0.1, timestamp=1.4)
400
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

TimeSource = Callable[[], float]


@dataclass
class SilenceTracker:
    """Track how long the input audio stream has remained silent.

    Parameters
    ----------
    silence_threshold:
        The RMS value above which the stream is considered *not* silent.
    time_source:
        Optional callable that returns the current timestamp in seconds.
        Defaults to :func:`time.monotonic` for resilience against clock
        adjustments, but tests can inject a deterministic source.
    """

    silence_threshold: float
    time_source: TimeSource = field(default=None)

    def __post_init__(self) -> None:
        import time

        if self.time_source is None:
            self.time_source = time.monotonic
        self._silence_start: Optional[float] = None
        self._silence_ms: int = 0

    def update(self, *, rms: float, autophony_ms: int = 0, timestamp: Optional[float] = None) -> int:
        """Advance the tracker using the latest RMS sample.

        Parameters
        ----------
        rms:
            Root-mean-square amplitude of the audio frame.
        autophony_ms:
            Duration in milliseconds that the robot is emitting sound
            itself ("autophony").  Any positive value forces a reset to
            ``0`` because the robot cannot be considered silent while it
            is actively speaking.
        timestamp:
            Optional explicit timestamp in seconds.  If omitted the
            configured :attr:`time_source` is invoked.

        Returns
        -------
        int
            Milliseconds elapsed since the stream was last considered
            non-silent.
        """

        now = timestamp if timestamp is not None else self.time_source()

        if rms > self.silence_threshold or autophony_ms > 0:
            self._silence_start = None
            self._silence_ms = 0
            return 0

        if self._silence_start is None:
            # Transition into silence: seed the counter but keep it at zero
            # for this frame so downstream consumers observe a reset when
            # silence first returns.
            self._silence_start = now
            self._silence_ms = 0
            return 0

        silence_duration = max(0.0, now - self._silence_start)
        self._silence_ms = max(0, int(round(silence_duration * 1000)))
        return self._silence_ms
