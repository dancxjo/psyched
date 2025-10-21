from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone

from pilot.models import SensationSummary
from pilot.prompt_builder import PilotPromptContext, PromptImage, build_prompt


_SNAPSHOT_TS = datetime(2024, 1, 1, 0, 0, tzinfo=timezone.utc)


def _topic_entry(value, *, age: float = 0.5, summary: str | None = None) -> dict:
    captured_at = (_SNAPSHOT_TS - timedelta(seconds=age)).isoformat()
    entry = {
        "value": value,
        "age_seconds": age,
        "captured_at": captured_at,
    }
    if summary is not None:
        entry["prompt_summary"] = summary
    return entry


def test_build_prompt_structures_context_and_actions() -> None:
    context = PilotPromptContext(
        topics={
            "/voice": _topic_entry("Queue hello", summary="Queue hello"),
            "/voice/spoken": _topic_entry(
                "Hello there", age=0.2, summary="Just said \"Hello there\"."
            ),
            "/instant": _topic_entry("Saw a familiar person waving."),
            "/status": _topic_entry({"speech": "paused"}),
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
        snapshot_timestamp=_SNAPSHOT_TS,
        static_sections=[
            "Voice module exposes /voice (pending speech) and /voice/spoken (recent narration)."
        ],
    )

    prompt = build_prompt(context)

    assert prompt.startswith("You are an autonomous artificial embodied being")
    assert "Context\n-------" in prompt
    assert "- rolling_window_seconds: 3.00" in prompt
    assert _SNAPSHOT_TS.isoformat() in prompt
    assert "- status_summary:" in prompt
    assert "recent_sensations" in prompt
    assert "Topics (prioritised)" in prompt
    assert "Available actions\n-----------------" in prompt
    assert '\"pause_speech\", \"say\"' in prompt
    assert "Module briefs" in prompt
    assert "Voice module exposes" in prompt
    assert "/voice/spoken" in prompt
    assert "faces" in prompt
    assert '\"queue_length\": 0' in prompt


def test_build_prompt_mentions_vision_images_when_available() -> None:
    """Ensure vision images add explicit prompt guidance without leaking binary data."""

    context = PilotPromptContext(
        topics={"/instant": _topic_entry("Investigating the lab.")},
        topic_templates={},
        status={"speech": "active"},
        sensations=[],
        cockpit_actions=["nav.move_to"],
        window_seconds=2.0,
        snapshot_timestamp=_SNAPSHOT_TS,
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
            "/custom/state": _topic_entry(
                {"pose": {"x": 1.0, "y": -0.5}, "status": "ready"}
            ),
            "/instant": _topic_entry("Standing by"),
        },
        topic_templates={
            "/custom/state": "state: x={{data.pose.x}} y={{data.pose.y}} status={{data.status}}",
        },
        status={},
        sensations=[],
        cockpit_actions=["voice.say"],
        window_seconds=1.0,
        snapshot_timestamp=_SNAPSHOT_TS,
    )

    prompt = build_prompt(context)

    assert "state: x=1.0 y=-0.5 status=ready" in prompt
    assert "/custom/state" in prompt


def test_build_prompt_prioritises_voice_and_asr_topics() -> None:
    context = PilotPromptContext(
        topics={
            "/ear/hole": _topic_entry("hello world"),  # ASR transcript topic
            "/voice": _topic_entry("Queue hello", summary="Queue hello"),
            "/voice/spoken": _topic_entry("Hello there", summary="Hello there"),
            "/voice/pause": _topic_entry(""),  # empty but should still appear
            "/status": _topic_entry({"speech": "paused"}),
            "/misc": _topic_entry(""),  # should be filtered because not meaningful
        },
        topic_templates={},
        status={"speech": "paused"},
        sensations=[],
        cockpit_actions=["voice.say"],
        window_seconds=1.0,
        snapshot_timestamp=_SNAPSHOT_TS,
    )

    prompt = build_prompt(context)

    topics_section = prompt.split("Topics (prioritised)\n--------------------\n", 1)[1]
    assert "/voice/spoken" in topics_section
    assert "/voice" in topics_section
    assert "/ear/hole" in topics_section
    assert "/misc" not in topics_section
    assert topics_section.index("/voice/spoken") < topics_section.index("/ear/hole")


def test_build_prompt_labels_topic_age_and_captured_time() -> None:
    context = PilotPromptContext(
        topics={
            "/instant": _topic_entry("Monitoring", age=0.1),
            "/voice": _topic_entry("Queue hello", age=1.7, summary="Queue hello"),
        },
        topic_templates={},
        status={},
        sensations=[],
        cockpit_actions=[],
        window_seconds=2.0,
        snapshot_timestamp=_SNAPSHOT_TS,
    )

    prompt = build_prompt(context)

    assert "[0.1s ago" in prompt
    assert "[1.7s ago" in prompt
    assert (_SNAPSHOT_TS - timedelta(seconds=1.7)).isoformat() in prompt
