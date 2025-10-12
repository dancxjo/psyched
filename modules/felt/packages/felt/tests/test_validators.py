from __future__ import annotations

import json

import pytest

from felt.validators import FeelingIntentValidationError, parse_feeling_intent_json


@pytest.fixture(name="allowed_actions")
def _allowed_actions() -> list[str]:
    return ["say(text)", "pause_speech", "resume_speech", "move_to(target)"]


def test_parse_feeling_intent_json_success(allowed_actions):
    payload = {
        "attitude_emoji": "🙂🤔",
        "thought_sentence": "I should greet them calmly.",
        "spoken_sentence": "Hello there!",
        "commands": ["resume_speech", "say('Hello there!')"],
        "goals": ["greet", "inspect"],
        "mood_delta": "uplifting",
        "memory_collection_raw": "faces",
        "memory_collection_text": "thoughts",
        "memory_collection_emoji": "emotions",
        "episode_id": "ep123",
        "situation_id": "sit456",
    }

    result = parse_feeling_intent_json(json.dumps(payload), allowed_actions)

    assert result.attitude_emoji == "🙂🤔"
    assert result.spoken_sentence == "Hello there!"
    assert result.commands == ["resume_speech", "say('Hello there!')"]
    assert result.goals == ["greet", "inspect"]
    assert result.mood_delta == "uplifting"
    assert result.memory_collection_raw == "faces"
    assert result.memory_collection_text == "thoughts"
    assert result.memory_collection_emoji == "emotions"
    assert result.episode_id == "ep123"
    assert result.situation_id == "sit456"


def test_parse_feeling_intent_json_rejects_bad_emoji(allowed_actions):
    payload = {
        "attitude_emoji": "abc",
        "thought_sentence": "A single sentence only.",
        "spoken_sentence": "",
        "commands": [],
        "goals": [],
    }

    with pytest.raises(FeelingIntentValidationError):
        parse_feeling_intent_json(json.dumps(payload), allowed_actions)


def test_parse_feeling_intent_json_rejects_multi_sentence_thought(allowed_actions):
    payload = {
        "attitude_emoji": "🙂",
        "thought_sentence": "First sentence. Second sentence.",
        "spoken_sentence": "",
        "commands": [],
        "goals": [],
    }

    with pytest.raises(FeelingIntentValidationError):
        parse_feeling_intent_json(json.dumps(payload), allowed_actions)


def test_parse_feeling_intent_json_rejects_unknown_command(allowed_actions):
    payload = {
        "attitude_emoji": "🙂",
        "thought_sentence": "Respond with kindness.",
        "spoken_sentence": "",
        "commands": ["dance"],
        "goals": [],
    }

    with pytest.raises(FeelingIntentValidationError):
        parse_feeling_intent_json(json.dumps(payload), allowed_actions)
