from __future__ import annotations

from datetime import datetime, timezone

from felt.memory_pipeline import MemoryBatch, prepare_memory_batch
from felt.models import FeelingIntentData, SensationRecord


def test_prepare_memory_batch_includes_vectors_and_graph_links():
    feeling = FeelingIntentData(
        attitude_emoji="🙂",
        thought_sentence="I should remember this moment.",
        spoken_sentence="",
        commands=["pause_speech"],
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
        feeling_id="feel-123",
    )

    assert isinstance(batch, MemoryBatch)
    assert batch.feeling_id == "feel-123"
    collections = {entry.get("collection") for entry in batch.vectors}
    assert {"faces", "thoughts", "emotions"}.issubset(collections)
    assert batch.graph_mutations
    params = batch.graph_mutations[0]["params"]
    assert params["episode_id"] == "ep1"
    assert params["sensation_ids"] == ["face_1"]
    assert "source_topics" in params and "/instant" in params["source_topics"]
