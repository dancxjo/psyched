"""Topic translation helpers for natural language prompt context.

This module discovers optional per-module translators that distil raw ROS topic
payloads into concise natural language summaries for the pilot's prompt
builder. Modules can drop a ``topic_translator.py`` file alongside their
``topic_suggestions.json`` manifest and expose a ``TOPIC_TRANSLATORS`` mapping
from topic names (or prefixes ending with ``*``) to callables. Each callable
receives the sanitised topic payload and returns a short descriptive string.

Examples
--------
>>> import tempfile
>>> from pathlib import Path
>>> with tempfile.TemporaryDirectory() as tmp:
...     root = Path(tmp)
...     module_dir = root / "demo" / "pilot"
...     module_dir.mkdir(parents=True, exist_ok=True)
...     _ = (module_dir / "topic_translator.py").write_text(
...         "TOPIC_TRANSLATORS = {'/demo/topic': lambda payload: 'ok'}",
...         encoding='utf-8',
...     )
...     translators = discover_topic_translators(root)
...     apply_topic_translation('/demo/topic', {'value': 1}, translators)
{'value': 1, 'prompt_summary': 'ok'}
"""

from __future__ import annotations

import runpy
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, MutableMapping, Optional

TopicTranslator = Callable[[Any], Optional[str]]


def discover_topic_translators(
    root: Optional[Path],
    *,
    logger: Optional[Any] = None,
) -> Dict[str, TopicTranslator]:
    """Return translator callables declared under the repository *root*.

    The discovery process searches each ``modules/<name>/pilot`` directory for a
    ``topic_translator.py`` file exposing a ``TOPIC_TRANSLATORS`` mapping. Keys
    ending with ``*`` act as prefix matches, allowing a single translator to
    cover host-specific topic names.
    """

    if root is None:
        return {}

    try:
        base = Path(root)
    except TypeError:
        return {}

    translators: Dict[str, TopicTranslator] = {}
    if not base.exists():
        return translators

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
        if not isinstance(mapping, Mapping):
            continue
        for key, translator in mapping.items():
            if not isinstance(key, str) or not callable(translator):
                continue
            translators[key] = translator  # later entries override earlier ones
    return translators


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
