"""Voice playback ROS node utilities."""

from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - only for type checking
    from .backends import (
        EspeakSpeechBackend,
        PrintSpeechBackend,
        SpeechBackend,
        WebsocketTTSSpeechBackend,
    )
    from .exceptions import SpeechInterrupted
    from .node import VoiceNode
    from .queue import SpeechQueue

__all__ = [
    "EspeakSpeechBackend",
    "PrintSpeechBackend",
    "SpeechBackend",
    "WebsocketTTSSpeechBackend",
    "SpeechInterrupted",
    "SpeechQueue",
    "VoiceNode",
]

_MODULE_MAP = {
    "EspeakSpeechBackend": "voice.backends",
    "PrintSpeechBackend": "voice.backends",
    "SpeechBackend": "voice.backends",
    "WebsocketTTSSpeechBackend": "voice.backends",
    "SpeechInterrupted": "voice.exceptions",
    "SpeechQueue": "voice.queue",
    "VoiceNode": "voice.node",
}


def __getattr__(name: str):  # pragma: no cover - trivial delegation
    module_name = _MODULE_MAP.get(name)
    if module_name is None:
        raise AttributeError(f"module 'voice' has no attribute '{name}'")
    module = import_module(module_name)
    return getattr(module, name)

