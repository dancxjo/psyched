from __future__ import annotations

import json

from pilot.prompt_builder import PilotPromptContext, PromptImage, build_prompt
from pilot.models import SensationSummary


def test_build_prompt_structures_context_and_actions() -> None:
    context = PilotPromptContext(
        topics={
            "/voice": "Queue hello",
            "/voice/spoken": "Hello there",
            "/instant": "Saw a familiar person waving.",
            "/status": {"speech": "paused"},
        },
        topic_templates={},
        status={"speech": "paused", "queue_length": 0},
        sensations=[
            SensationSummary(
                topic="/sensation/face",
                kind="face",
                collection_hint="faces",
                json_payload=json.dumps({"id": "f_93", "meta": {"name_hint": "Ava"}}),
                vector_length=4,
            )
        ],
        cockpit_actions=["voice.say", "voice.pause_speech"],
        window_seconds=3.0,
    )

    prompt = build_prompt(context)

    assert prompt.startswith("You are PILOT")
    assert "Context\n-------" in prompt
    assert "- window_seconds: 3.00" in prompt
    assert "- status_summary:" in prompt
    assert "recent_sensations" in prompt
    assert "Topics (prioritised)" in prompt
    assert "Available actions\n-----------------" in prompt
    assert '\"pause_speech\", \"say\"' in prompt
    assert "/voice/spoken" in prompt
    assert "faces" in prompt
    assert '\"queue_length\": 0' in prompt


def test_build_prompt_mentions_vision_images_when_available() -> None:
    """Ensure vision images add explicit prompt guidance without leaking binary data."""

    context = PilotPromptContext(
        topics={"/instant": "Investigating the lab."},
        topic_templates={},
        status={"speech": "active"},
        sensations=[],
        cockpit_actions=["nav.move_to"],
        window_seconds=2.0,
        vision_images=[
            PromptImage(
                topic="/camera/color/image_raw/compressed",
                description="current frame jpeg (9216 bytes)",
                base64_data="AAECAw==",
            )
        ],
    )

    prompt = build_prompt(context)

    assert "Vision" in prompt
    assert "robot is seeing currently" in prompt
    assert "/camera/color/image_raw/compressed" in prompt
    assert "AAECAw==" not in prompt


def test_build_prompt_uses_topic_templates_for_custom_formatting() -> None:
    context = PilotPromptContext(
        topics={
            "/custom/state": {"pose": {"x": 1.0, "y": -0.5}, "status": "ready"},
            "/instant": "Standing by",
        },
        topic_templates={
            "/custom/state": "state: x={{data.pose.x}} y={{data.pose.y}} status={{data.status}}",
        },
        status={},
        sensations=[],
        cockpit_actions=["voice.say"],
        window_seconds=1.0,
    )

    prompt = build_prompt(context)

    assert "state: x=1.0 y=-0.5 status=ready" in prompt
    assert "/custom/state" in prompt


def test_build_prompt_prioritises_voice_and_asr_topics() -> None:
    context = PilotPromptContext(
        topics={
            "/ear/hole": "hello world",  # ASR transcript topic
            "/voice": "Queue hello",
            "/voice/spoken": "Hello there",
            "/voice/pause": "",  # empty but should still appear
            "/status": {"speech": "paused"},
            "/misc": "",  # should be filtered because not meaningful
        },
        topic_templates={},
        status={"speech": "paused"},
        sensations=[],
        cockpit_actions=["voice.say"],
        window_seconds=1.0,
    )

    prompt = build_prompt(context)

    topics_section = prompt.split("Topics (prioritised)\n--------------------\n", 1)[1]
    assert "/voice/spoken" in topics_section
    assert "/voice" in topics_section
    assert "/ear/hole" in topics_section
    assert "/misc" not in topics_section
    assert topics_section.index("/voice/spoken") < topics_section.index("/ear/hole")
