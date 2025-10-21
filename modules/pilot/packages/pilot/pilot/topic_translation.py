"""Topic translation helpers for natural language prompt context.

This module discovers optional per-module translators that distil raw ROS topic
payloads into concise natural language summaries for the pilot's prompt
builder. Modules can drop a ``topic_translator.py`` file alongside their
``topic_suggestions.json`` manifest and expose a ``TOPIC_TRANSLATORS`` mapping
from topic names (or prefixes ending with ``*``) to callables. Each callable
receives the sanitised topic payload and returns a short descriptive string.

Translator modules may also publish ``STATIC_PROMPT_SECTIONS``—a collection of
strings describing the static capabilities or expectations for the module. The
pilot surfaces these sections verbatim inside the prompt so the LLM always
knows what each module provides, even when live data is momentarily absent.

Examples
--------
>>> import tempfile
>>> from pathlib import Path
>>> with tempfile.TemporaryDirectory() as tmp:
...     root = Path(tmp)
...     module_dir = root / "demo" / "pilot"
...     module_dir.mkdir(parents=True, exist_ok=True)
...     _ = (module_dir / "topic_translator.py").write_text(
...         "TOPIC_TRANSLATORS = {'/demo/topic': lambda payload: 'ok'}\n"
...         "STATIC_PROMPT_SECTIONS = ['demo topics scroll through the instant window']",
...         encoding='utf-8',
...     )
...     translators = discover_topic_translators(root)
...     apply_topic_translation('/demo/topic', {'value': 1}, translators)
{'value': 1, 'prompt_summary': 'ok'}
>>> translators.static_sections
['demo topics scroll through the instant window']
"""

from __future__ import annotations

import runpy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterator, List, Mapping, MutableMapping, Optional

TopicTranslator = Callable[[Any], Optional[str]]


@dataclass(slots=True)
class TranslatorRegistry(Mapping[str, TopicTranslator]):
    """Mapping wrapper that also stores prompt sections advertised by modules."""

    _translators: Dict[str, TopicTranslator] = field(default_factory=dict)
    static_sections: List[str] = field(default_factory=list)

    def __iter__(self) -> Iterator[str]:
        return iter(self._translators)

    def __len__(self) -> int:
        return len(self._translators)

    def __getitem__(self, key: str) -> TopicTranslator:
        return self._translators[key]

    def get(self, key: str, default: Any = None) -> Any:  # pragma: no cover - Mapping API parity
        return self._translators.get(key, default)

    def items(self):  # pragma: no cover - convenience helper
        return self._translators.items()

    def keys(self):  # pragma: no cover - convenience helper
        return self._translators.keys()

    def values(self):  # pragma: no cover - convenience helper
        return self._translators.values()


def _collect_static_sections(value: Any) -> List[str]:
    sections: List[str] = []
    if isinstance(value, str):
        text = value.strip()
        if text:
            sections.append(text)
        return sections
    if isinstance(value, (list, tuple, set)):
        for item in value:
            if isinstance(item, str):
                text = item.strip()
                if text:
                    sections.append(text)
    return sections


def discover_topic_translators(
    root: Optional[Path],
    *,
    logger: Optional[Any] = None,
) -> TranslatorRegistry:
    """Return translator callables and static prompt sections discovered under *root*."""

    if root is None:
        return TranslatorRegistry()

    try:
        base = Path(root)
    except TypeError:
        return TranslatorRegistry()

    translators: Dict[str, TopicTranslator] = {}
    static_sections: List[str] = []
    if not base.exists():
        return TranslatorRegistry(translators, static_sections)

    for module_dir in sorted(p for p in base.iterdir() if p.is_dir()):
        translator_path = module_dir / "pilot" / "topic_translator.py"
        if not translator_path.exists():
            continue
        try:
            module_globals = runpy.run_path(str(translator_path))
        except Exception as exc:  # pragma: no cover - defensive guard
            if logger:
                logger.warning("Failed to load %s: %s", translator_path, exc)
            continue
        mapping = module_globals.get("TOPIC_TRANSLATORS")
        if isinstance(mapping, Mapping):
            for key, translator in mapping.items():
                if not isinstance(key, str) or not callable(translator):
                    continue
                translators[key] = translator  # later entries override earlier ones
        static_sections.extend(
            _collect_static_sections(module_globals.get("STATIC_PROMPT_SECTIONS"))
        )
    if static_sections:
        seen: Dict[str, None] = {}
        for section in static_sections:
            if section not in seen:
                seen[section] = None
        static_sections = list(seen.keys())
    return TranslatorRegistry(translators, static_sections)


def _resolve_translator(
    topic: str,
    translators: Mapping[str, TopicTranslator],
) -> Optional[TopicTranslator]:
    translator = translators.get(topic)
    if translator is not None:
        return translator
    for key, candidate in translators.items():
        if not key.endswith("*"):
            continue
        prefix = key[:-1]
        if topic.startswith(prefix):
            return candidate
    return None


def apply_topic_translation(
    topic: str,
    payload: Any,
    translators: Mapping[str, TopicTranslator],
    *,
    logger: Optional[Any] = None,
) -> Any:
    """Return *payload* enriched with a ``prompt_summary`` when possible."""

    translator = _resolve_translator(topic, translators)
    if translator is None:
        return payload
    try:
        summary = translator(payload)
    except Exception as exc:  # pragma: no cover - defensive guard
        if logger:
            logger.warning("Translator for %s failed: %s", topic, exc)
        return payload
    if summary is None:
        return payload
    text = str(summary).strip()
    if not text:
        return payload

    enriched: MutableMapping[str, Any]
    if isinstance(payload, Mapping):
        enriched = dict(payload)
    else:
        enriched = {"raw": payload}
    enriched["prompt_summary"] = text
    return enriched
