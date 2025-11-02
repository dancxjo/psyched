"""Prompt translators for navigation perception topics."""

from __future__ import annotations

import math
import statistics
from typing import Any, Iterable, Mapping


def _extract_text(payload: Any) -> str | None:
    if isinstance(payload, Mapping):
        data = payload.get("data")
        if isinstance(data, str):
            text = data.strip()
            return text or None
    elif isinstance(payload, str):
        text = payload.strip()
        return text or None
    return None


def summarise_vision_annotation(payload: Any) -> str:
    """Summarize the free-form annotation emitted by the vision prompt node."""

    text = _extract_text(payload)
    if text is None:
        return "Vision annotation stream is idle."
    return f'Vision annotation: "{text}".'


def _finite_ranges(values: Iterable[Any]) -> list[float]:
    readings: list[float] = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            readings.append(number)
    return readings


def summarise_laserscan(payload: Any) -> str:
    """Summarize the ``sensor_msgs/msg/LaserScan`` payload."""

    if not isinstance(payload, Mapping):
        return "Depth scan awaiting data."
    ranges = payload.get("ranges")
    if not isinstance(ranges, Iterable):
        return "Depth scan awaiting data."
    finite = _finite_ranges(ranges)
    if not finite:
        return "Depth scan awaiting data."
    count = len(finite)
    min_range = min(finite)
    max_range = max(finite)
    median = statistics.median(finite)
    return (
        f"Depth scan: {count} beams min={min_range:.2f}m "
        f"median={median:.2f}m max={max_range:.2f}m."
    )


TOPIC_TRANSLATORS = {
    "/vision_prompt/vision_annotation": summarise_vision_annotation,
    "vision_annotation": summarise_vision_annotation,
    "/depth_projection/scan": summarise_laserscan,
    "/scan": summarise_laserscan,
    "scan": summarise_laserscan,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Nav contributes perception context via /vision_prompt/vision_annotation "
        "(LLM-generated overlays) and /depth_projection/scan (synthetic laser). "
        "Summaries condense annotations and range statistics so the pilot can "
        "reason about nearby obstacles."
    )
]


__all__ = [
    "TOPIC_TRANSLATORS",
    "STATIC_PROMPT_SECTIONS",
    "summarise_vision_annotation",
    "summarise_laserscan",
]
