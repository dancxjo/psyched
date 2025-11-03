"""Custom cockpit actions for the faces module."""

from __future__ import annotations

from typing import Any, Iterable, List

from cockpit.actions import ActionContext, ActionError, ActionResult
from cockpit.ros import RosClient

DEFAULT_TIMEOUT_SECONDS = 8.0


def _service_config(definition: Any) -> tuple[str, str | None, float]:
    defaults = getattr(definition, "defaults", {}) or {}
    service = defaults.get("service")
    if not isinstance(service, str) or not service.strip():
        raise ActionError("Faces tagging action missing service configuration")
    service_type = defaults.get("service_type")
    if service_type is not None and (not isinstance(service_type, str) or not service_type.strip()):
        service_type = None
    timeout_ms = defaults.get("timeout_ms", int(DEFAULT_TIMEOUT_SECONDS * 1000))
    try:
        timeout = float(timeout_ms) / 1000.0
    except (TypeError, ValueError):
        timeout = DEFAULT_TIMEOUT_SECONDS
    if timeout <= 0:
        timeout = DEFAULT_TIMEOUT_SECONDS
    return service, service_type, timeout


def _normalise_aliases(raw: Any, label: str) -> List[str]:
    aliases: List[str] = []
    if raw is None:
        pass
    elif isinstance(raw, str):
        text = raw.strip()
        if text:
            aliases.append(text)
    elif isinstance(raw, Iterable):
        for entry in raw:
            if isinstance(entry, str):
                text = entry.strip()
                if text and text not in aliases:
                    aliases.append(text)
    else:
        raise ActionError("aliases must be a list of strings when provided")
    label_text = label.strip()
    if label_text and label_text not in aliases:
        aliases.append(label_text)
    return aliases


async def tag_face_action(
    *,
    context: ActionContext,
    ros: RosClient,
    definition: Any,
) -> ActionResult:
    """Tag a stored face memory with a manual identity label."""

    face_id_raw = context.get_argument("face_id")
    if not isinstance(face_id_raw, str) or not face_id_raw.strip():
        raise ActionError("face_id must be a non-empty string")
    face_id = face_id_raw.strip()

    label_raw = context.get_argument("label")
    if not isinstance(label_raw, str) or not label_raw.strip():
        raise ActionError("label must be a non-empty string")
    label = label_raw.strip()

    identity_id_raw = context.get_argument("identity_id")
    identity_id = identity_id_raw.strip() if isinstance(identity_id_raw, str) else ""

    aliases = _normalise_aliases(context.get_argument("aliases", None), label)

    service, service_type, timeout = _service_config(definition)

    request_payload = {
        "memory_id": face_id,
        "label": label,
        "identity_id": identity_id,
        "aliases": aliases,
    }

    try:
        result = await ros.call_service(
            service_name=service,
            service_type=service_type,
            arguments=request_payload,
            timeout=timeout,
        )
    except Exception as exc:  # pragma: no cover - ROS bridge runtime errors
        raise ActionError(f"Failed to tag face identity: {exc}") from exc

    success = bool(result.get("success"))
    if not success:
        message = str(result.get("message") or "Tagging failed without additional details.")
        raise ActionError(message)

    return ActionResult(payload={"result": result})


__all__ = ["tag_face_action"]
