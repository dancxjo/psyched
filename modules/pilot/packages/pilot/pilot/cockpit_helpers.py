"""Utility helpers for sharing cockpit contracts between pilot and cockpit."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Mapping, Tuple


def load_local_cockpit_actions(
    modules_root: Path,
    *,
    logger: Any | None = None,
) -> Tuple[list[str], Dict[str, Dict[str, Any]]]:
    """Return cockpit actions discovered under *modules_root*.

    Parameters
    ----------
    modules_root:
        Base directory containing module folders (e.g. ``.../modules``).
    logger:
        Optional logger receiving debug information when manifests cannot be
        parsed.

    Returns
    -------
    tuple[list[str], dict[str, dict[str, Any]]]
        ``(actions, schemas)`` where *actions* is a list of fully-qualified
        ``"module.action"`` identifiers and *schemas* maps those identifiers to
        their parameter schemas.

    Examples
    --------
    >>> from pathlib import Path
    >>> root = Path("/tmp/modules")
    >>> (root / "alpha" / "cockpit" / "api").mkdir(parents=True, exist_ok=True)
    >>> (root / "alpha" / "cockpit" / "api" / "actions.json").write_text(
    ...     '{"actions": [{"name": "ping", "parameters": {"text": {"type": "string"}}}]}'
    ... )
    >>> load_local_cockpit_actions(root)[0]
    ['alpha.ping']

    The helper mirrors the logic used by the ``psh`` CLI so the pilot can rely
    on the same manifests without shelling out to ``psh``.
    """

    actions: list[str] = []
    schemas: Dict[str, Dict[str, Any]] = {}

    try:
        module_dirs = sorted(p for p in modules_root.iterdir() if p.is_dir())
    except FileNotFoundError:
        return actions, schemas
    except OSError as exc:  # pragma: no cover - filesystem race conditions
        if logger is not None:
            logger.warning("Failed to enumerate modules at %s: %s", modules_root, exc)
        return actions, schemas

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
            actions.append(fq_name)
            params = entry.get("parameters")
            if isinstance(params, Mapping):
                schemas[fq_name] = dict(params)

    return actions, schemas


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

