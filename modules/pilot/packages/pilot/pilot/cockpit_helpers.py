"""Utility helpers for sharing cockpit contracts between pilot and cockpit."""

from __future__ import annotations

import json
from collections import OrderedDict
from pathlib import Path
from typing import Any, Dict, Mapping, MutableMapping, Tuple

__all__ = [
    "load_local_cockpit_actions",
    "normalise_cockpit_modules_payload",
    "render_action_signature",
]

_SIGNATURE_TYPE_MAP = {
    "string": "str",
    "number": "float",
    "integer": "int",
    "boolean": "bool",
    "array": "list",
    "object": "dict",
}


def _coerce_parameters(value: Any) -> Dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    return {}


def render_action_signature(name: str, parameters: Mapping[str, Any] | None) -> str:
    if not name:
        return "()"
    if not parameters:
        return f"{name}()"
    props = parameters.get("properties") if isinstance(parameters, Mapping) else None
    if not isinstance(props, Mapping) or not props:
        return f"{name}()"

    ordered = OrderedDict()
    for key in props.keys():
        ordered[str(key)] = props[key]

    parts: list[str] = []
    for key, spec in ordered.items():
        if not isinstance(spec, Mapping):
            hint = "object"
        else:
            raw_type = spec.get("type")
            if isinstance(raw_type, str):
                hint = _SIGNATURE_TYPE_MAP.get(raw_type.lower(), raw_type)
            else:
                hint = "object"
            if hint == "list" and isinstance(spec.get("items"), Mapping):
                item_spec = spec.get("items", {})
                item_type = item_spec.get("type") if isinstance(item_spec, Mapping) else None
                if isinstance(item_type, str):
                    hint = f"list[{_SIGNATURE_TYPE_MAP.get(item_type.lower(), item_type)}]"
        parts.append(f"{key}: {hint}")
    joined = ", ".join(parts)
    return f"{name}({joined})"


def load_local_cockpit_actions(
    modules_root: Path,
    *,
    logger: Any | None = None,
) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, Dict[str, Any]]]:
    """Return cockpit actions discovered under *modules_root*.

    The helper yields two parallel mappings: ``metadata`` stores rich action
    descriptors keyed by fully-qualified name (``module.action``) while
    ``schemas`` exposes the raw JSON Schema fragments used for argument
    validation. Each descriptor includes a generated ``signature`` so downstream
    consumers can present callable summaries without re-deriving them.
    """

    metadata: Dict[str, Dict[str, Any]] = {}
    schemas: Dict[str, Dict[str, Any]] = {}

    try:
        module_dirs = sorted(p for p in modules_root.iterdir() if p.is_dir())
    except FileNotFoundError:
            return metadata, schemas
    except OSError as exc:  # pragma: no cover - filesystem race conditions
        if logger is not None:
            logger.warning("Failed to enumerate modules at %s: %s", modules_root, exc)
            return metadata, schemas

    for module_dir in module_dirs:
        actions_file = module_dir / "cockpit" / "api" / "actions.json"
        try:
            text = actions_file.read_text(encoding="utf-8")
        except FileNotFoundError:
            continue
        except OSError as exc:  # pragma: no cover - filesystem errors vary by host
            if logger is not None:
                logger.warning("Failed to read %s: %s", actions_file, exc)
            continue

        try:
            payload = json.loads(text)
        except json.JSONDecodeError as exc:
            if logger is not None:
                logger.warning("Invalid JSON in %s: %s", actions_file, exc)
            continue

        raw_actions = payload.get("actions") if isinstance(payload, Mapping) else None
        if not isinstance(raw_actions, list):
            continue

        for entry in raw_actions:
            if not isinstance(entry, Mapping):
                continue
            name = str(entry.get("name") or "").strip()
            if not name:
                continue
            fq_name = f"{module_dir.name}.{name}"
            params = _coerce_parameters(entry.get("parameters"))
            schemas[fq_name] = params
            description = str(entry.get("description") or "").strip()
            signature = str(entry.get("signature") or "").strip()
            if not signature:
                signature = render_action_signature(name, params)
            metadata[fq_name] = {
                "module": module_dir.name,
                "name": name,
                "signature": signature,
                "description": description,
                "parameters": params,
                "streaming": bool(entry.get("streaming")),
                "kind": str(entry.get("kind") or "").strip(),
            }

    return metadata, schemas


def normalise_cockpit_modules_payload(payload: Mapping[str, Any]) -> Dict[str, Any]:
    """Return a condensed status snapshot derived from ``/api/modules``.

    The cockpit's ``/api/modules`` endpoint emits a list of module dictionaries,
    bridge metadata and host information. The pilot prefers a dictionary keyed
    by module name so we collapse the payload into a structure that is stable
    for LLM consumption.
    """

    modules_section = payload.get("modules")
    modules: Dict[str, Dict[str, Any]] = {}

    if isinstance(modules_section, list):
        for entry in modules_section:
            if not isinstance(entry, Mapping):
                continue
            name = str(entry.get("name") or "").strip()
            if not name:
                continue
            systemd = entry.get("systemd")
            modules[name] = {
                "display_name": entry.get("display_name"),
                "slug": entry.get("slug"),
                "dashboard_url": entry.get("dashboard_url"),
                "has_cockpit": bool(entry.get("has_cockpit")),
                "systemd": dict(systemd) if isinstance(systemd, Mapping) else {},
            }

    status: Dict[str, Any] = {"modules": modules}

    host = payload.get("host")
    if isinstance(host, Mapping):
        status["host"] = {
            "name": host.get("name"),
            "shortname": host.get("shortname"),
        }

    bridge = payload.get("bridge")
    if isinstance(bridge, Mapping):
        status["bridge"] = dict(bridge)

    return status

