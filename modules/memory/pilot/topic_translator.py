"""Prompt helpers surfacing the memory module to the pilot loop."""

from __future__ import annotations

from typing import Any, Dict, Optional

# No dynamic topic translations yet, but keep the mapping ready so additional
# telemetry topics can plug in without touching pilot internals.
TOPIC_TRANSLATORS: Dict[str, Any] = {}

# Static prompt sections teach the pilot that the memory module is available
# over ROS services for storing and recalling contextual traces.
STATIC_PROMPT_SECTIONS = [
    (
        "Memory module persists Pete's sensations and pilot outputs via "
        "ROS services (/memory/memorize, /memory/associate, /memory/recall). "
        "Store events with vector embeddings so future recalls can ground the "
        "pilot's reasoning in past observations linked through Neo4j."
    ),
    (
        "When recalling, specify the collection name (faces, thoughts, emotions, "
        "etc.) so Qdrant searches the right neighbourhood. Returned memories "
        "include metadata describing origin topics, timestamps, and sources."
    ),
]


def __all__() -> list[str]:  # pragma: no cover - module metadata helper
    return ["TOPIC_TRANSLATORS", "STATIC_PROMPT_SECTIONS"]
