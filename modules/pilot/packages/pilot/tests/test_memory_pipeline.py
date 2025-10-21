from __future__ import annotations

from datetime import datetime, timezone

from pilot.memory_pipeline import MemoryBatch, prepare_memory_batch
from pilot.models import FeelingIntentData, SensationRecord


def test_prepare_memory_batch_builds_events_and_associations():
    feeling = FeelingIntentData(
        situation_overview="Pete is cataloguing faces in the atrium.",
        attitude_emoji="🙂",
        thought_sentence="I should remember this moment.",
        spoken_sentence="",
        command_script="voice.pause_speech()",
        goals=["reflect"],
        mood_delta="calming",
        memory_collection_raw="faces",
        memory_collection_text="thoughts",
        memory_collection_emoji="emotions",
        episode_id="ep1",
        situation_id="sit1",
    )

    sensations = [
        SensationRecord(
            topic="/sensation/face",
            kind="face",
            collection_hint="faces",
            json_payload="{\"id\": \"face_1\"}",
            vector=[0.1, 0.2, 0.3],
        )
    ]

    timestamp = datetime(2024, 1, 1, tzinfo=timezone.utc)
    batch = prepare_memory_batch(
        feeling=feeling,
        sensations=sensations,
        source_topics=["/instant", "/sensation/face"],
        timestamp=timestamp,
        feeling_id="pilot-123",
    )

    assert isinstance(batch, MemoryBatch)
    assert batch.feeling_id == "pilot-123"
    assert batch.events
    feeling_event = next(event for event in batch.events if event.tag == "feeling")
    assert feeling_event.kind == "thoughts"
    assert feeling_event.metadata["situation_overview"] == "Pete is cataloguing faces in the atrium."
    assert feeling_event.metadata["episode_id"] == "ep1"
    sensation_event = next(event for event in batch.events if event.metadata["topic"] == "/sensation/face")
    assert sensation_event.kind == "faces"
    assert sensation_event.embedding == [0.1, 0.2, 0.3]
    assert batch.associations
    link = batch.associations[0]
    assert link.source_tag == sensation_event.tag
    assert link.target_tag == "feeling"
    assert link.properties["topic"] == "/sensation/face"
