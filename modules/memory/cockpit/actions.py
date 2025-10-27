"""Custom cockpit actions for the memory module."""

from __future__ import annotations

import json
from typing import Any, Mapping

from cockpit.actions import ActionError, ActionResult, ActionContext
from cockpit.ros import RosClient

DEFAULT_TIMEOUT_SECONDS = 8.0


def _service_config(definition: Any) -> tuple[str, str | None, float]:
    defaults = getattr(definition, "defaults", {}) or {}
    service = defaults.get("service")
    if not isinstance(service, str) or not service.strip():
        raise ActionError("Memory action is missing a target service configuration")
    service_type = defaults.get("service_type")
    if service_type is not None and (not isinstance(service_type, str) or not service_type.strip()):
        service_type = None
    timeout_ms = defaults.get("timeout_ms", int(DEFAULT_TIMEOUT_SECONDS * 1000))
    try:
        timeout = float(timeout_ms) / 1000.0
    except (TypeError, ValueError):  # pragma: no cover - defensive guard
        timeout = DEFAULT_TIMEOUT_SECONDS
    if timeout <= 0:
        timeout = DEFAULT_TIMEOUT_SECONDS
    return service, service_type, timeout


def _coerce_mapping(value: Any) -> Mapping[str, Any]:
    if isinstance(value, Mapping):
        return value
    raise ActionError("Expected a JSON object for memory payload data")


async def memorize_event_action(*, context: ActionContext, ros: RosClient, definition: Any) -> ActionResult:
    """Persist a structured memory event via the memory service."""

    kind_raw = context.get_argument("kind")
    if not isinstance(kind_raw, str) or not kind_raw.strip():
        raise ActionError("kind must be a non-empty string")
    kind = kind_raw.strip()

    data_raw = context.get_argument("data")
    data = dict(_coerce_mapping(data_raw))

    flush = context.get_argument("flush", True)
    flush_flag = bool(flush)

    service, service_type, timeout = _service_config(definition)

    payload = {
        "event": {
            "header": {
                "frame_id": "memory",
                "stamp": {"sec": 0, "nanosec": 0},
            },
            "kind": kind,
            "json_data": json.dumps(data, ensure_ascii=False),
            "embedding": [],
        },
        "flush": flush_flag,
    }

    try:
        result = await ros.call_service(
            service_name=service,
            service_type=service_type,
            arguments=payload,
            timeout=timeout,
        )
    except Exception as exc:  # pragma: no cover - ROS errors exercised in integration
        raise ActionError(f"Failed to memorise event: {exc}") from exc

    return ActionResult(payload={"result": result})


async def recall_text_action(*, context: ActionContext, ros: RosClient, definition: Any) -> ActionResult:
    """Retrieve memories relevant to a free-text query."""

    query_raw = context.get_argument("query")
    if not isinstance(query_raw, str) or not query_raw.strip():
        raise ActionError("query must be a non-empty string")
    query = query_raw.strip()

    kind_raw = context.get_argument("kind")
    collection = kind_raw.strip() if isinstance(kind_raw, str) and kind_raw.strip() else "episodic"

    k_value = context.get_argument("k", 5)
    try:
        limit = int(k_value)
    except (TypeError, ValueError):
        raise ActionError("k must be an integer")
    if limit <= 0 or limit > 50:
        raise ActionError("k must be between 1 and 50")

    service, service_type, timeout = _service_config(definition)

    payload = {
        "kind": collection,
        "text": query,
        "limit": limit,
        "embedding": [],
    }

    try:
        result = await ros.call_service(
            service_name=service,
            service_type=service_type,
            arguments=payload,
            timeout=timeout,
        )
    except Exception as exc:  # pragma: no cover - ROS errors exercised in integration
        raise ActionError(f"Failed to recall memories: {exc}") from exc

    return ActionResult(payload={"result": result})


__all__ = ["memorize_event_action", "recall_text_action"]
