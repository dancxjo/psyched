from __future__ import annotations

import json
import re

import math
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

from .models import SensationSummary

_SYSTEM_PROMPT = """You are the narrator/director of an autonomous artificial embodied being named Pete. You take raw sensory input and produce a compact JSON representation of the being's current state. Make sure the character stays kind, pilot, honest, curious, ethical, and obedient. Produce one compact JSON ONLY.
Rules:
- "situation_overview": <=320 characters describing Pete's current environment from inputs. Place this as the FIRST key. This first observation field must translate the provided data (topics, sensations, status, images) from their raw formats into concise natural language, highlighting the most important details while filtering out noise or low-signal elements. Never fabricate information beyond the supplied inputs, but you may infer what is happening around you if it is reasonable. (Remember: this is real life, not a simulation.)
- "attitude_emoji": 1–2 Unicode emoji, NO WORDS. (Represent attitude/mood only)
- "thought_sentence": exactly 1 sentence.
- "spoken_sentence": 0 or 1 sentence (empty if none). This text is auto-queued for speech; do not repeat it via voice.say().
- "command_script": a Python 3 script (string) executed by a sandboxed interpreter.
  * Use cockpit actions as callables (e.g. voice.say(text="Hi"), nav.move_to(target="dock")).
  * Control flow is allowed (if/for/while/def). Avoid imports or filesystem access.
  * Use available_actions() to introspect options and gate behaviour.
  * Scripts execute asynchronously—log intent via voice.say when deferring or awaiting resources.
  * You may call action("module.action", **kwargs) as a fallback form.
- Base every observation and decision strictly on the provided topics, sensations, images, and status summaries; never assume or invent facts that are not explicitly present.
- Keep JSON under 512 tokens. No commentary outside JSON.
-- Additionally, stream the raw LLM response to STDOUT where possible for debugging.
"""

_SCHEMA_HINT = {
    "situation_overview": "The lab is calm; a teammate is waving from the workstation.",
    "attitude_emoji": "🙂",
    "thought_sentence": "I should greet them and step closer to see better.",
    "spoken_sentence": "Hey there—good to see you!",
    "command_script": "\n".join(
        [
            "# Resume the conversation and approach if navigation is idle.",
            "if 'nav.move_to' in available_actions():",
            "    voice.resume_speech()",
            "    nav.move_to(target='person_estimated')",
            "else:",
            "    voice.say(text=\"Standing by until movement is available.\")",
        ]
    ),
    "goals": ["greet", "improve_viewing_conditions"],
    "mood_delta": "uplifting",
}


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


def _format_topics(entries: Iterable[Tuple[str, Any]], templates: Mapping[str, str]) -> Iterable[str]:
    for topic, entry in entries:
        value = _topic_payload(entry)
        template = templates.get(topic)
        if template:
            rendered = _render_template(template, topic=topic, value=value)
            yield f"- {_format_topic_prefix(entry)}{topic}: {rendered}"
            continue
        serialised = json.dumps(value, ensure_ascii=False, sort_keys=True)
        yield f"- {_format_topic_prefix(entry)}{topic}: {serialised}"


def _format_sensations(sensations: Iterable[SensationSummary]) -> str:
    payloads = [s.prompt_payload() for s in sensations]
    return json.dumps(payloads, ensure_ascii=False, sort_keys=True)


def _format_vision_images(images: Iterable[PromptImage]) -> Iterable[str]:
    for image in images:
        yield f"- {image.prompt_hint()}"


def _group_actions_by_module(actions: Iterable[str]) -> Dict[str, List[str]]:
    grouped: Dict[str, List[str]] = {}
    for action in actions:
        if not action:
            continue
        module, _, name = action.partition(".")
        if not name:
            module, name = "shared", action
        grouped.setdefault(module, []).append(name)

    for module_actions in grouped.values():
        module_actions.sort()

    return dict(sorted(grouped.items()))


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

    lines: List[str] = [_SYSTEM_PROMPT, "", "Context", "-------"]

    lines.append(f"- rolling_window_seconds: {context.window_seconds:.2f}")
    lines.append(
        "- snapshot_captured_at_utc: "
        + context.snapshot_timestamp.isoformat()
    )
    lines.append(
        "- instant_note: "
        f"This is a rolling instant covering the last {context.window_seconds:.2f}s; "
        "entries show how recently they were seen and scroll away once older than this window."
    )
    if context.status:
        condensed = _condense_status(context.status)
        if condensed:
            lines.append(
                "- status_summary: "
                + json.dumps(condensed, ensure_ascii=False, sort_keys=True)
            )
    lines.append(
        "- recent_sensations: " + _format_sensations(context.sensations)
    )

    if context.static_sections:
        sections = [section.strip() for section in context.static_sections if section.strip()]
        if sections:
            lines.append("")
            lines.append("Module briefs")
            lines.append("-------------")
            for section in sections:
                lines.append(f"- {section}")

    lines.append("")
    lines.append("Topics (prioritised)")
    lines.append("--------------------")
    topic_entries = _select_topic_entries(context.topics)
    formatted_topics = list(_format_topics(topic_entries, context.topic_templates))
    if formatted_topics:
        lines.extend(formatted_topics)
    else:
        lines.append("- none")

    lines.append("")
    lines.append("Available actions")
    lines.append("-----------------")
    grouped_actions = _group_actions_by_module(sorted(set(context.cockpit_actions)))
    if grouped_actions:
        for module, actions in grouped_actions.items():
            lines.append(f"- {module}: {json.dumps(actions, ensure_ascii=False)}")
    else:
        lines.append("- none: []")
    if context.vision_images:
        lines.append("")
        lines.append("Vision")
        lines.append("------")
        lines.extend(_format_vision_images(context.vision_images))
        lines.append(
            "These frames show what the robot is seeing currently; reference them when reasoning."
        )
    lines.append("")
    lines.append("Schema")
    lines.append("------")
    lines.append(json.dumps(_SCHEMA_HINT, ensure_ascii=False, indent=2))
    return "\n".join(lines)
