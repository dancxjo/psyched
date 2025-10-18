from __future__ import annotations

import json
import re
import unicodedata
from typing import Iterable, Sequence

from .models import FeelingIntentData


class FeelingIntentValidationError(ValueError):
    """Raised when the LLM response cannot be trusted."""


_SENTENCE_TERMINATOR_PATTERN = re.compile(r"[.?!]")
_MULTIPLE_SENTENCE_PATTERN = re.compile(r"[.?!].+?[.?!]")
def _is_single_sentence(text: str) -> bool:
    cleaned = " ".join(text.strip().split())
    if not cleaned:
        return False
    if _MULTIPLE_SENTENCE_PATTERN.search(cleaned):
        return False
    terminators = _SENTENCE_TERMINATOR_PATTERN.findall(cleaned)
    return len(terminators) <= 1


def _is_valid_emoji_string(text: str) -> bool:
    if not text:
        return False
    # Limit to 12 UTF-16 code units.
    if len(text.encode("utf-16-le")) // 2 > 12:
        return False
    base_count = 0
    for ch in text:
        if ch.isspace():
            return False
        cat = unicodedata.category(ch)
        if cat[0] in {"L", "N"} or cat[0] == "P":
            return False
        if cat not in {"Mn", "Me", "Cf"}:
            base_count += 1
    # Accept 1-2 base emoji glyphs for attitude/mood
    return 1 <= base_count <= 2


def _normalise_command(command: str) -> str:
    name = command.strip()
    if "(" in name:
        return name.split("(", 1)[0]
    return name


def _allowed_command_bases(actions: Iterable[str]) -> set[str]:
    """Return the set of allowed command bases.

    Actions may be provided as simple names ("say") or module-qualified
    entries ("voice.say"). We normalise by returning the base portion so the
    validator can accept either form in the LLM output.
    """
    bases = set()
    for action in actions:
        if not isinstance(action, str):
            continue
        action = action.strip()
        if not action:
            continue
        # If module-qualified like 'module.action', take last segment
        if "." in action:
            candidate = action.rsplit(".", 1)[-1]
        else:
            candidate = action
        bases.add(_normalise_command(candidate))
    return bases


def parse_feeling_intent_json(raw_json: str, allowed_actions: Sequence[str]) -> FeelingIntentData:
    try:
        payload = json.loads(raw_json)
    except json.JSONDecodeError as exc:
        raise FeelingIntentValidationError("LLM output is not valid JSON") from exc

    if not isinstance(payload, dict):
        raise FeelingIntentValidationError("LLM output must be a JSON object")

    attitude = str(payload.get("attitude_emoji", ""))
    if not _is_valid_emoji_string(attitude):
        raise FeelingIntentValidationError("attitude_emoji must contain only emoji (1-3 glyphs)")

    thought = str(payload.get("thought_sentence", "")).strip()
    if not _is_single_sentence(thought):
        raise FeelingIntentValidationError("thought_sentence must be exactly one sentence")

    spoken = str(payload.get("spoken_sentence", "")).strip()
    if spoken and not _is_single_sentence(spoken):
        raise FeelingIntentValidationError("spoken_sentence must be empty or a single sentence")

    commands_raw = payload.get("commands", [])
    if not isinstance(commands_raw, list) or not all(isinstance(c, str) for c in commands_raw):
        raise FeelingIntentValidationError("commands must be a list of strings")

    # Validate commands against the allowed action bases. Commands may be in
    # the form 'name', "name(arg)", or qualified 'module.name' / 'module.name(arg)'.
    allowed_bases = _allowed_command_bases(allowed_actions)
    validated_commands: list[str] = []
    for command in commands_raw:
        cmd = command.strip()
        # If module-qualified, extract the final name portion
        if "." in cmd and not cmd.startswith("{"):
            # split module qualification from any params: e.g. 'voice.say("hi")' -> 'voice.say("hi")'
            # isolate the base name after last dot
            # but preserve arguments for storage
            after = cmd.rsplit(".", 1)[-1]
            base = _normalise_command(after)
        else:
            base = _normalise_command(cmd)

        if base not in allowed_bases:
            raise FeelingIntentValidationError(f"Unknown command: {command}")

        # Basic sanitisation: ensure parentheses are balanced if present
        if "(" in cmd:
            open_count = cmd.count("(")
            close_count = cmd.count(")")
            if open_count != close_count:
                raise FeelingIntentValidationError(f"Malformed command arguments: {command}")

        validated_commands.append(cmd)

    goals_raw = payload.get("goals", [])
    if not isinstance(goals_raw, list) or not all(isinstance(goal, str) for goal in goals_raw):
        raise FeelingIntentValidationError("goals must be a list of strings")

    mood_delta = str(payload.get("mood_delta", "")).strip()
    memory_collection_raw = str(payload.get("memory_collection_raw", "")).strip()
    memory_collection_text = str(payload.get("memory_collection_text", "")).strip()
    memory_collection_emoji = str(payload.get("memory_collection_emoji", "")).strip()
    episode_id = str(payload.get("episode_id", "")).strip()
    situation_id = str(payload.get("situation_id", "")).strip()

    return FeelingIntentData(
        attitude_emoji=attitude,
        thought_sentence=thought,
        spoken_sentence=spoken,
        commands=validated_commands,
        goals=[goal.strip() for goal in goals_raw if goal.strip()],
        mood_delta=mood_delta,
        memory_collection_raw=memory_collection_raw,
        memory_collection_text=memory_collection_text,
        memory_collection_emoji=memory_collection_emoji,
        episode_id=episode_id,
        situation_id=situation_id,
    )
