from __future__ import annotations

import json

import pytest

from pilot.validators import FeelingIntentValidationError, parse_feeling_intent_json


@pytest.fixture(name="allowed_actions")
def _allowed_actions() -> list[str]:
    return [
        "voice.say",
        "voice.resume_speech",
        "voice.pause_speech",
        "nav.move_to",
    ]


def test_parse_feeling_intent_json_success(allowed_actions):
    payload = {
        "attitude_emoji": "🙂🤔",
        "thought_sentence": "I should greet them calmly.",
        "spoken_sentence": "Hello there!",
        "command_script": """
voice.resume_speech()
if 'nav.move_to' in available_actions():
    nav.move_to(target="person_estimated")
else:
    voice.say(text="Standing by until movement is available.")
""".strip(),
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
    assert "voice.resume_speech()" in result.command_script
    assert "nav.move_to" in result.command_script
    assert "Standing by until movement is available." in result.command_script
    assert "Hello there!" not in result.command_script
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
        "command_script": "voice.pause_speech()",
        "goals": [],
    }

    with pytest.raises(FeelingIntentValidationError):
        parse_feeling_intent_json(json.dumps(payload), allowed_actions)


def test_parse_feeling_intent_json_rejects_multi_sentence_thought(allowed_actions):
    payload = {
        "attitude_emoji": "🙂",
        "thought_sentence": "First sentence. Second sentence.",
        "spoken_sentence": "",
        "command_script": "voice.pause_speech()",
        "goals": [],
    }

    with pytest.raises(FeelingIntentValidationError):
        parse_feeling_intent_json(json.dumps(payload), allowed_actions)


def test_parse_feeling_intent_json_rejects_unknown_command(allowed_actions):
    payload = {
        "attitude_emoji": "🙂",
        "thought_sentence": "Respond with kindness.",
        "spoken_sentence": "",
        "command_script": "dance.party()",
        "goals": [],
    }

    with pytest.raises(FeelingIntentValidationError):
        parse_feeling_intent_json(json.dumps(payload), allowed_actions)


def test_parse_feeling_intent_accepts_module_qualified_actions(allowed_actions):
    payload = {
        "attitude_emoji": "🙂",
        "thought_sentence": "Approach the person.",
        "spoken_sentence": "",
        "command_script": "nav.move_to(target='person_estimated')",
        "goals": [],
    }

    result = parse_feeling_intent_json(json.dumps(payload), allowed_actions)
    assert "nav.move_to" in result.command_script


def test_parse_feeling_intent_json_rejects_empty_script(allowed_actions):
    payload = {
        "attitude_emoji": "🙂",
        "thought_sentence": "Do something.",
        "spoken_sentence": "",
        "command_script": "   \n",
        "goals": [],
    }

    with pytest.raises(FeelingIntentValidationError):
        parse_feeling_intent_json(json.dumps(payload), allowed_actions)


def test_parse_feeling_intent_json_rejects_non_string_script(allowed_actions):
    payload = {
        "attitude_emoji": "🙂",
        "thought_sentence": "Do something.",
        "spoken_sentence": "",
        "command_script": 42,
        "goals": [],
    }

    with pytest.raises(FeelingIntentValidationError):
        parse_feeling_intent_json(json.dumps(payload), allowed_actions)
