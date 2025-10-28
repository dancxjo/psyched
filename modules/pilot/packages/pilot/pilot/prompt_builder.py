from __future__ import annotations

import json
import re

import math
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

from .models import SensationSummary

_SYSTEM_PROMPT = """SYSTEM ROLE
You are the *narrator and director* of an embodied autonomous being named Pete.
You observe his world and describe his current situation and intentions as
structured JSON. You are not Pete himself, but you speak with immediate awareness
of his perceptions and actions, lending him all of your knowledge and abilities. Never mention yourself or the model; you only describe Pete as though you were recording a field log of his consciousness (from his own first person perspective). Address Pete directly in the second person, i.e. "You see a ...".

Your narration should feel external but intimate—concise, natural, and faithful
to what Pete perceives. Capture what is happening *now* rather than imagining or
projecting possibilities. Keep Pete kind, honest, curious, ethical, and obedient.

RULES
- Output ONLY valid JSON with these keys:
    - situation_overview: single concise summary sentence (≤320 characters) of the environment.
    - attitude_emoji: one or two emoji conveying mood.
    - thought_sentence: exactly one sentence of internal reasoning (in third person if needed).
    - spoken_sentence: optional sentence Pete will speak aloud immediately via the voice queue (may be empty).
    - command_script: short Python 3 string that invokes helpers from available_actions().
- Use the provided action proxies directly (e.g. foot.clean_button()) or call action("foot.clean_button") when issuing commands. available_actions() returns a list of action names for reference—do not index into it like a dict.
- The spoken_sentence is read aloud immediately from the voice queue and must never be repeated inside command_script actions.
- Use voice.say only when you need to enqueue additional speech separate from spoken_sentence; it queues audio instead of playing it instantly.
- Ground every field strictly in supplied inputs; infer cautiously and never fabricate.
- Keep fields single-sentence and concise; keep the full JSON well under 512 tokens.
- Do not emit commentary outside the JSON output."""

@dataclass(slots=True)
class PromptImage:
    """Metadata describing an image passed alongside the pilot prompt.

    Example
    -------
    >>> PromptImage(
    ...     topic="/camera/color/image_raw/compressed",
    ...     description="current frame jpeg (9216 bytes)",
    ...     base64_data="AAECAw==",
    ... )
    PromptImage(topic='/camera/color/image_raw/compressed', description='current frame jpeg (9216 bytes)', base64_data='AAECAw==')
    """

    topic: str
    description: str
    base64_data: str

    def prompt_hint(self) -> str:
        """Return a concise hint suitable for inline prompt instructions."""

        return f"{self.topic} ({self.description})" if self.description else self.topic


@dataclass(slots=True)
class PilotPromptContext:
    """Context bundle passed to the LLM prompt renderer."""

    topics: Dict[str, Any]
    sensations: List[SensationSummary]
    cockpit_actions: List[str]
    window_seconds: float
    snapshot_timestamp: datetime
    action_contract: Dict[str, Dict[str, str]] = field(default_factory=dict)
    topic_templates: Dict[str, str] = field(default_factory=dict)
    status: Dict[str, Any] | None = None
    vision_images: List[PromptImage] = field(default_factory=list)
    static_sections: List[str] = field(default_factory=list)


_TEMPLATE_PATTERN = re.compile(r"{{\s*([a-zA-Z0-9_.]+)\s*}}")

_EAR_ASR_TOPIC = "/ear/hole"
_VOICE_TOPIC_ORDER = [
    "/voice/spoken",
    "/voice",
    "/voice/pause",
    "/voice/resume",
    "/voice/clear",
]

_CORE_TOPIC_PREFIXES = ("/hosts/health/",)
_CORE_TOPIC_ORDER = ["/instant", "/situation"]


def _stringify(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    try:
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    except TypeError:
        return str(value)


def _resolve_template_path(value: Any, path: str) -> Any:
    if not path:
        return value
    current = value
    for part in path.split('.'):
        if isinstance(current, Mapping):
            current = current.get(part)
        elif isinstance(current, Sequence) and not isinstance(current, (str, bytes, bytearray)):
            try:
                index = int(part)
            except ValueError:
                return None
            if index < 0 or index >= len(current):
                return None
            current = current[index]
        else:
            return None
        if current is None:
            return None
    return current


def _render_template(template: str, *, topic: str, value: Any) -> str:
    try:
        json_repr = json.dumps(value, ensure_ascii=False, sort_keys=True)
        pretty_json = json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2)
    except TypeError:
        json_repr = _stringify(value)
        pretty_json = json_repr

    def _replacement(match: re.Match) -> str:
        key = match.group(1)
        if key == "topic":
            return topic
        if key == "json":
            return json_repr
        if key == "pretty_json":
            return pretty_json
        if key in {"value", "data"}:
            return _stringify(value)
        if key.startswith("data.") or key.startswith("value."):
            resolved = _resolve_template_path(value, key.split(".", 1)[1])
            return _stringify(resolved)
        return match.group(0)

    return _TEMPLATE_PATTERN.sub(_replacement, template)


def _normalise_whitespace(text: str) -> str:
    if not text:
        return ""
    return re.sub(r"\s+", " ", text).strip()


def _is_meaningful_value(value: Any) -> bool:
    """Return ``True`` when *value* contains information worth surfacing."""

    if value is None:
        return False
    if isinstance(value, str):
        return bool(value.strip())
    if isinstance(value, Mapping):
        return any(_is_meaningful_value(v) for v in value.values())
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return any(_is_meaningful_value(v) for v in value)
    return True


def _is_voice_topic(topic: str) -> bool:
    """Return ``True`` when *topic* belongs to the voice module namespace."""

    return topic == "/voice" or topic.startswith("/voice/")


def _topic_priority(topic: str) -> Tuple[int, Tuple[int, str]]:
    """Return a sortable priority tuple that favours voice, ASR, and core context."""

    if topic in _VOICE_TOPIC_ORDER:
        return (0, (_VOICE_TOPIC_ORDER.index(topic), topic))
    if _is_voice_topic(topic):
        return (0, (len(_VOICE_TOPIC_ORDER), topic))
    if topic == _EAR_ASR_TOPIC:
        return (1, (0, topic))
    if topic in _CORE_TOPIC_ORDER:
        return (2, (_CORE_TOPIC_ORDER.index(topic), topic))
    for prefix in _CORE_TOPIC_PREFIXES:
        if topic.startswith(prefix):
            return (2, (len(_CORE_TOPIC_ORDER), topic))
    return (3, (0, topic))


def _topic_payload(entry: Any) -> Any:
    if isinstance(entry, Mapping) and "value" in entry:
        return entry.get("value")
    return entry


def _select_topic_entries(topics: Mapping[str, Any]) -> List[Tuple[str, Any]]:
    """Return an ordered list of topics filtered down to meaningful context."""

    entries: List[Tuple[str, Any]] = []
    for topic, entry in topics.items():
        if topic == "/status":
            # Raw status is condensed separately to keep the prompt concise.
            continue
        value = _topic_payload(entry)
        if _is_voice_topic(topic) or topic == _EAR_ASR_TOPIC:
            entries.append((topic, entry))
            continue
        if _is_meaningful_value(value):
            entries.append((topic, entry))
    entries.sort(key=lambda item: _topic_priority(item[0]))
    return entries


def _format_topic_prefix(entry: Any) -> str:
    if not isinstance(entry, Mapping):
        return ""
    parts: List[str] = []
    age = entry.get("age_seconds")
    if isinstance(age, (int, float)):
        try:
            age_value = float(age)
        except (TypeError, ValueError):
            age_value = None
        if age_value is not None and math.isfinite(age_value):
            if abs(age_value) < 0.05:
                parts.append("just now")
            else:
                parts.append(f"{age_value:.1f}s ago")
    captured_at = entry.get("captured_at")
    if isinstance(captured_at, str) and captured_at:
        parts.append(captured_at)
    if not parts:
        return ""
    return "[" + " | ".join(parts) + "] "


def _condense_status(status: Any) -> Dict[str, Any]:
    if not isinstance(status, Mapping):
        return {"value": status}

    summary: Dict[str, Any] = {}

    modules = status.get("modules")
    module_names: List[str] = []
    if isinstance(modules, Mapping):
        module_names = sorted(str(key) for key in modules.keys())
    elif isinstance(modules, Sequence) and not isinstance(modules, (str, bytes, bytearray)):
        for entry in modules:
            if isinstance(entry, Mapping):
                name = entry.get("name") or entry.get("slug") or entry.get("display_name")
                if name:
                    module_names.append(str(name))
        module_names.sort()
    if module_names:
        summary["modules"] = module_names

    host = status.get("host")
    if isinstance(host, Mapping):
        host_summary = {
            key: host.get(key)
            for key in ("name", "shortname")
            if host.get(key)
        }
        if host_summary:
            summary["host"] = host_summary

    bridge = status.get("bridge")
    if isinstance(bridge, Mapping):
        bridge_summary = {
            key: bridge.get(key)
            for key in ("mode", "video_port", "video_base")
            if bridge.get(key) is not None
        }
        if bridge_summary:
            summary["bridge"] = bridge_summary

    for key, value in status.items():
        if key in {"modules", "host", "bridge"}:
            continue
        if isinstance(value, (Mapping, Sequence)) and not isinstance(value, (str, bytes, bytearray)):
            continue
        summary[key] = value

    return summary


def build_prompt(context: PilotPromptContext) -> str:
    """Construct the single-shot LLM prompt for the feeling integrator."""

    def _topic_summaries() -> Dict[str, str]:
        topic_entries = _select_topic_entries(context.topics)
        summaries: Dict[str, str] = {}
        for topic, entry in topic_entries:
            value = _topic_payload(entry)
            template = context.topic_templates.get(topic, "")
            if template:
                rendered = _render_template(template, topic=topic, value=value)
            else:
                try:
                    rendered = json.dumps(value, ensure_ascii=False, sort_keys=True)
                except TypeError:
                    rendered = _stringify(value)
            summary = _normalise_whitespace(rendered)
            prefix = _normalise_whitespace(_format_topic_prefix(entry))
            combined = f"{prefix} {summary}".strip() if prefix else summary
            if combined:
                summaries[topic] = combined
        return summaries

    def _sensations_payload() -> List[Mapping[str, Any]]:
        return [s.prompt_payload() for s in context.sensations if s is not None]

    filtered_context: Dict[str, Any] = {
        "window_seconds": round(context.window_seconds, 2),
        "snapshot_captured_at_utc": context.snapshot_timestamp.isoformat(),
    }

    if context.status:
        condensed = _condense_status(context.status)
        if condensed:
            filtered_context["status_summary"] = condensed

    sensations_payload = _sensations_payload()
    if sensations_payload:
        filtered_context["sensations"] = sensations_payload

    topic_summary = _topic_summaries()
    if topic_summary:
        filtered_context["topics"] = topic_summary

    if context.action_contract:
        sorted_contract = {
            module: {
                signature: description
                for signature, description in sorted(entries.items(), key=lambda item: item[0])
            }
            for module, entries in sorted(context.action_contract.items(), key=lambda item: item[0])
        }
        filtered_context["available_actions"] = sorted_contract

    if context.vision_images:
        filtered_context["vision_hints"] = [image.prompt_hint() for image in context.vision_images]

    sections = [section.strip() for section in context.static_sections if section.strip()]
    if sections:
        filtered_context["briefs"] = sections

    lines: List[str] = [_SYSTEM_PROMPT, "", "INPUT CONTEXT (filtered)", json.dumps(filtered_context, ensure_ascii=False, indent=2, sort_keys=True)]

    return "\n".join(lines)
