from __future__ import annotations

import json
import re
import textwrap
import unicodedata
from typing import Sequence

from .command_script import CommandScriptError, CommandScriptInterpreter
from .models import FeelingIntentData


class FeelingIntentValidationError(ValueError):
    """Raised when the LLM response cannot be trusted."""


_SENTENCE_TERMINATOR_PATTERN = re.compile(r"[.?!]")
_MULTIPLE_SENTENCE_PATTERN = re.compile(r"[.?!].+?[.?!]")
_MAX_SCRIPT_CHARACTERS = 4096
_MAX_SCRIPT_LINES = 200
_MAX_OVERVIEW_CHARACTERS = 320
_COMMAND_INTERPRETER = CommandScriptInterpreter()

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
def parse_feeling_intent_json(raw_json: str, allowed_actions: Sequence[str]) -> FeelingIntentData:
    try:
        payload = json.loads(raw_json)
    except json.JSONDecodeError as exc:
        raise FeelingIntentValidationError("LLM output is not valid JSON") from exc

    if not isinstance(payload, dict):
        raise FeelingIntentValidationError("LLM output must be a JSON object")

    overview = str(payload.get("situation_overview", "")).strip()
    if not overview:
        raise FeelingIntentValidationError("situation_overview must describe the current environment")
    if len(overview) > _MAX_OVERVIEW_CHARACTERS:
        raise FeelingIntentValidationError(
            f"situation_overview must be <= {_MAX_OVERVIEW_CHARACTERS} characters"
        )

    attitude = str(payload.get("attitude_emoji", ""))
    if not _is_valid_emoji_string(attitude):
        raise FeelingIntentValidationError("attitude_emoji must contain only emoji (1-3 glyphs)")

    thought = str(payload.get("thought_sentence", "")).strip()
    if not _is_single_sentence(thought):
        raise FeelingIntentValidationError("thought_sentence must be exactly one sentence")

    spoken = str(payload.get("spoken_sentence", "")).strip()
    if spoken and not _is_single_sentence(spoken):
        raise FeelingIntentValidationError("spoken_sentence must be empty or a single sentence")

    command_script_raw = payload.get("command_script", "")
    if not isinstance(command_script_raw, str):
        raise FeelingIntentValidationError("command_script must be a string")
    command_script = textwrap.dedent(command_script_raw).strip()
    if not command_script:
        raise FeelingIntentValidationError("command_script must not be empty")
    if len(command_script) > _MAX_SCRIPT_CHARACTERS:
        raise FeelingIntentValidationError(
            f"command_script exceeds {_MAX_SCRIPT_CHARACTERS} characters"
        )
    if command_script.count("\n") + 1 > _MAX_SCRIPT_LINES:
        raise FeelingIntentValidationError(
            f"command_script exceeds {_MAX_SCRIPT_LINES} lines"
        )
    try:
        _COMMAND_INTERPRETER.analyse(command_script, allowed_actions)
    except CommandScriptError as exc:
        raise FeelingIntentValidationError(f"command_script invalid: {exc}") from exc

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
        situation_overview=overview,
        attitude_emoji=attitude,
        thought_sentence=thought,
        spoken_sentence=spoken,
        command_script=command_script,
        goals=[goal.strip() for goal in goals_raw if goal.strip()],
        mood_delta=mood_delta,
        memory_collection_raw=memory_collection_raw,
        memory_collection_text=memory_collection_text,
        memory_collection_emoji=memory_collection_emoji,
        episode_id=episode_id,
        situation_id=situation_id,
    )
