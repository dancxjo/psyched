from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

from pilot.topic_translation import (
    TranslatorRegistry,
    apply_topic_translation,
    discover_topic_translators,
)


def test_discover_topic_translators_loads_translators(tmp_path: Path) -> None:
    translator_dir = tmp_path / "demo" / "pilot"
    translator_dir.mkdir(parents=True)
    (translator_dir / "topic_translator.py").write_text(
        """
from typing import Any

def summarise(payload: Any) -> str:
    return f"seen {payload}"

TOPIC_TRANSLATORS = {"/demo/topic": summarise}
STATIC_PROMPT_SECTIONS = [
    "Demo module offers /demo/topic which keeps a rolling instant summary."
]
""".strip(),
        encoding="utf-8",
    )

    translators = discover_topic_translators(tmp_path)
    assert "/demo/topic" in translators
    assert translators["/demo/topic"]({"value": 1}) == "seen {'value': 1}"
    assert (
        translators.static_sections
        == [
            "Demo module offers /demo/topic which keeps a rolling instant summary."
        ]
    )


def test_apply_topic_translation_enriches_mapping() -> None:
    payload: Dict[str, Any] = {"data": "hello"}
    translators = TranslatorRegistry({"/foo": lambda value: f"Summary of {value['data']}"})

    enriched = apply_topic_translation("/foo", payload, translators)

    assert enriched is not payload
    assert enriched["prompt_summary"] == "Summary of hello"
    assert payload == {"data": "hello"}


def test_apply_topic_translation_handles_raw_values() -> None:
    translators = TranslatorRegistry({"/foo": lambda value: "Summary"})

    enriched = apply_topic_translation("/foo", "raw", translators)

    assert enriched["prompt_summary"] == "Summary"
    assert enriched["raw"] == "raw"


def test_apply_topic_translation_supports_prefix_keys() -> None:
    translators = TranslatorRegistry({"/foo/*": lambda value: "Prefixed"})

    enriched = apply_topic_translation("/foo/bar", {"value": 1}, translators)

    assert enriched["prompt_summary"] == "Prefixed"


def test_apply_topic_translation_bails_on_none_summary() -> None:
    payload: Dict[str, Any] = {"data": "hello"}
    translators = {"/foo": lambda value: None}

    enriched = apply_topic_translation("/foo", payload, translators)

    assert enriched == payload
