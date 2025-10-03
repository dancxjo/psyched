"""Exception types for the voice module."""

class SpeechInterrupted(Exception):
    """Raised when speech playback should be interrupted.

    Speech backends should raise this exception when the provided stop event
    is triggered. The :class:`~voice.queue.SpeechQueue` catches the exception
    and handles requeueing logic as needed.
    """

    def __init__(self, reason: str | None = None) -> None:
        self.reason = reason
        message = "Speech interrupted" if reason is None else f"Speech interrupted: {reason}"
        super().__init__(message)
