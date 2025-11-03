"""Tests for the Faces module topic translator."""

from modules.faces.pilot.topic_translator import summarise_face_trigger


def test_face_trigger_summary_includes_name_and_memory_ids() -> None:
    """Ensure recognised faces surface key metadata to the prompt."""

    payload = {
        "data": '{"name": "Stranger", "memory_id": "abc", "vector_id": "v1"}'
    }
    assert (
        summarise_face_trigger(payload)
        == 'Face trigger: Stranger (memory abc, vector v1).'
    )


def test_face_trigger_summary_handles_missing_payload() -> None:
    """Unknown payloads should fall back to a gentle status message."""

    assert (
        summarise_face_trigger({})
        == "Face trigger: awaiting identification."
    )


def test_face_trigger_summary_uses_signature_fallback_when_ids_missing() -> None:
    """Signatures should surface when memory/vector identifiers are absent."""

    payload = {
        "data": '{"name": "Stranger", "signature": "deadbeef1234"}'
    }
    assert (
        summarise_face_trigger(payload)
        == "Face trigger: Stranger (memory mem-deadbeef1234, vector deadbeef1234)."
    )
