from __future__ import annotations

import json
import re

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Sequence

from .models import SensationSummary

_SYSTEM_PROMPT = """You are PILOT, Pete’s feeling+will integrator. Produce one compact JSON ONLY.
Rules:
- "situation_overview": <=320 characters describing Pete's current environment from inputs. Place this as the FIRST key.
- "attitude_emoji": 1–2 Unicode emoji, NO WORDS. (Represent attitude/mood only)
- "thought_sentence": exactly 1 sentence.
- "spoken_sentence": 0 or 1 sentence (empty if none). This text is auto-queued for speech; do not repeat it via voice.say().
- "command_script": a Python 3 script (string) executed by a sandboxed interpreter.
  * Use cockpit actions as callables (e.g. voice.say(text="Hi"), nav.move_to(target="dock")).
  * Control flow is allowed (if/for/while/def). Avoid imports or filesystem access.
  * Use available_actions() to introspect options and gate behaviour.
  * Scripts execute asynchronously—log intent via voice.say when deferring or awaiting resources.
  * You may call action("module.action", **kwargs) as a fallback form.
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
            "# Resume chatting and approach if navigation is idle.",
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
    topic_templates: Dict[str, str] = field(default_factory=dict)
    status: Dict[str, Any] | None = None
    vision_images: List[PromptImage] = field(default_factory=list)


_TEMPLATE_PATTERN = re.compile(r"{{\s*([a-zA-Z0-9_.]+)\s*}}")


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


def _format_topics(topics: Dict[str, Any], templates: Mapping[str, str]) -> Iterable[str]:
    for topic, value in topics.items():
        template = templates.get(topic)
        if template:
            rendered = _render_template(template, topic=topic, value=value)
            yield f"- {topic}: {rendered}"
            continue
        serialised = json.dumps(value, ensure_ascii=False, sort_keys=True)
        yield f"- {topic}: {serialised}"


def _format_sensations(sensations: Iterable[SensationSummary]) -> str:
    payloads = [s.prompt_payload() for s in sensations]
    return json.dumps(payloads, ensure_ascii=False, sort_keys=True)


def _format_vision_images(images: Iterable[PromptImage]) -> Iterable[str]:
    for image in images:
        yield f"- {image.prompt_hint()}"


def build_prompt(context: PilotPromptContext) -> str:
    """Construct the single-shot LLM prompt for the feeling integrator."""

    lines: List[str] = [_SYSTEM_PROMPT, "", "Context (examples merged each cycle)", "", "topics:"]
    lines.extend(_format_topics(context.topics, context.topic_templates))

    if context.status and "/status" not in context.topics:
        lines.append(f"- /status: {json.dumps(context.status, ensure_ascii=False, sort_keys=True)}")

    lines.append(
        "- recent_sensations: "
        + _format_sensations(context.sensations)
    )
    lines.append(
        f"- window_seconds: {context.window_seconds:.2f}"
    )
    lines.append(
        "cockpit_actions (from cockpit /api/actions): "
            + json.dumps(sorted(set(context.cockpit_actions)), ensure_ascii=False)
    )
    if context.vision_images:
        lines.append("")
        lines.append(
            "vision_images (robot's current view provided via the Ollama images API):"
        )
        lines.extend(_format_vision_images(context.vision_images))
        lines.append(
            "These frames show what the robot is seeing currently; reference them when reasoning."
        )
    lines.append("")
    lines.append("Schema to emit")
    lines.append("")
    lines.append(json.dumps(_SCHEMA_HINT, ensure_ascii=False, indent=2))
    return "\n".join(lines)
