from __future__ import annotations

import json
from felt.prompt_builder import FeltPromptContext, SensationSummary, build_prompt


def test_build_prompt_includes_topics_status_and_actions():
    context = FeltPromptContext(
        topics={"/instant": "Saw a familiar person waving.", "/status": {"speech": "paused"}},
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
        cockpit_actions=["say(text)", "pause_speech"],
        window_seconds=3.0,
    )

    prompt = build_prompt(context)

    assert prompt.startswith("You are FELT")
    assert "pause_speech" in prompt
    assert "/instant" in prompt
    assert "faces" in prompt
    assert "recent_sensations" in prompt
    assert "vector_length" in prompt
