"""Tests for the ear module's prompt translation helpers."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import ModuleType


def _load_topic_translator_module() -> ModuleType:
    """Return the loaded ``topic_translator`` module for the ear package."""

    translator_path = Path(__file__).resolve().parents[3] / "pilot" / "topic_translator.py"
    spec = importlib.util.spec_from_file_location("ear_topic_translator", translator_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


translator_module = _load_topic_translator_module()


def test_summarise_asr_event_partial() -> None:
    """Partials should describe what the ear *thinks* it hears."""

    payload = {"data": json.dumps({"event": "partial", "text": "testing"})}
    summary = translator_module.summarise_asr_event(payload)
    assert summary == 'I think I\'m hearing: "testing".'


def test_summarise_asr_event_final_from_segments() -> None:
    """Final events should describe what the ear definitively heard."""

    payload = {
        "data": json.dumps(
            {
                "event": "final",
                "text": " ",
                "segments": [{"text": "hello"}, {"text": "world"}],
            }
        )
    }
    summary = translator_module.summarise_asr_event(payload)
    assert summary == 'I heard: "hello world".'


def test_summarise_asr_event_raw_text_fallback() -> None:
    """Plain strings should be treated as definitive transcripts."""

    payload = {"data": "raw text"}
    summary = translator_module.summarise_asr_event(payload)
    assert summary == 'I heard: "raw text".'


def test_summarise_asr_event_empty_payload_defaults() -> None:
    """Lacking transcript content should fall back to the listening message."""

    payload = {"data": json.dumps({"event": "partial"})}
    summary = translator_module.summarise_asr_event(payload)
    assert summary == "ASR is listening for the next utterance."
