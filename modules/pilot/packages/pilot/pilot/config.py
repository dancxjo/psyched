"""Utilities for loading host configuration data used by the pilot."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # pragma: no cover - fallback for older interpreters
    import tomli as tomllib  # type: ignore

import yaml

__all__ = [
    "HostConfigError",
    "ModuleDescriptor",
    "discover_active_modules",
    "load_host_config",
]


class HostConfigError(RuntimeError):
    """Raised when host configuration data cannot be loaded."""


@dataclass(frozen=True)
class ModuleDescriptor:
    """Describe an active module surfaced in the cockpit."""

    name: str
    display_name: str
    raw_config: Mapping[str, Any]

    @property
    def slug(self) -> str:
        """Return a URL-friendly slug for the module."""
        return self.name.replace("_", "-")


def load_host_config(path: Path | str) -> MutableMapping[str, Any]:
    """Load a host configuration file.

    Parameters
    ----------
    path:
        Filesystem path to a JSON, JSONC, YAML, or TOML configuration file.

    Returns
    -------
    MutableMapping[str, Any]
        Parsed host configuration data.
    """

    resolved = Path(path)
    if not resolved.exists():
        raise HostConfigError(f"Host configuration not found: {resolved}")

    text = resolved.read_text(encoding="utf-8")
    suffix = resolved.suffix.lower()
    if suffix == ".jsonc":
        text = _strip_json_comments(text)
        suffix = ".json"

    if suffix == ".json":
        return json.loads(text)
    if suffix in {".yaml", ".yml"}:
        data = yaml.safe_load(text)
        if not isinstance(data, MutableMapping):
            raise HostConfigError(
                f"Expected mapping at root of YAML config but received {type(data)!r}",
            )
        return data
    if suffix == ".toml":
        data = tomllib.loads(text)
        if not isinstance(data, MutableMapping):
            raise HostConfigError(
                f"Expected mapping at root of TOML config but received {type(data)!r}",
            )
        return data

    raise HostConfigError(
        f"Unsupported host configuration format for '{resolved.name}'.",
    )


def discover_active_modules(config: Mapping[str, Any]) -> List[ModuleDescriptor]:
    """Return module descriptors for modules active on the host.

    When ``host.modules`` is declared, it is treated as the canonical list of
    cockpit modules. Config entries under ``config.mod`` (or the legacy
    ``modules`` table) supply metadata for those modules without enabling them
    implicitly. For legacy manifests that omit ``host.modules``, module
    directives with a truthy ``launch`` flag are still respected.
    """

    if not config:
        return []

    modules: Dict[str, Mapping[str, Any]] = {}
    modules.update(_normalize_module_entries(config.get("modules")))

    config_section = config.get("config")
    if isinstance(config_section, Mapping):
        modules.update(_normalize_module_entries(config_section.get("mod")))

    host_section = config.get("host")
    use_host_list = False
    host_module_names: List[str] = []
    if isinstance(host_section, Mapping) and "modules" in host_section:
        use_host_list = True
        host_module_names = _coerce_string_sequence(host_section.get("modules"))

    if use_host_list:
        return _modules_from_host_list(host_module_names, modules)

    active: Dict[str, ModuleDescriptor] = {}

    for name, raw in modules.items():
        if _launch_enabled(raw):
            materialized = dict(raw)
            active[name] = ModuleDescriptor(
                name=name,
                display_name=_display_name_for(name, materialized),
                raw_config=materialized,
            )

    return sorted(active.values(), key=lambda module: module.display_name.lower())


def _launch_enabled(raw: Mapping[str, Any]) -> bool:
    launch = raw.get("launch")
    if isinstance(launch, Mapping):
        enabled = launch.get("enabled")
        if enabled is None:
            return True
        coerced = _coerce_bool(enabled)
        if coerced is not None:
            return coerced
        return bool(enabled)
    if launch is None:
        return False
    coerced = _coerce_bool(launch)
    if coerced is not None:
        return coerced
    return bool(launch)


def _display_name_for(name: str, raw: Optional[Mapping[str, Any]]) -> str:
    if raw and isinstance(raw.get("display_name"), str):
        return raw["display_name"]  # type: ignore[index]
    if not name:
        return "Module"
    return name.replace("_", " ").title()


def _coerce_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if not normalized:
            return None
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
        return None
    if isinstance(value, (int, float)):
        if isinstance(value, float) and math.isnan(value):
            return None
        return value != 0
    return None


def _normalize_module_entries(raw: Any) -> Dict[str, Mapping[str, Any]]:
    modules: Dict[str, Mapping[str, Any]] = {}
    if isinstance(raw, Mapping):
        for name, value in raw.items():
            key = str(name)
            modules[key] = _coerce_module_mapping(value)
    elif isinstance(raw, Iterable) and not isinstance(raw, (str, bytes)):
        for value in raw:
            key = str(value)
            if not key:
                continue
            modules[key] = {"launch": True}
    elif isinstance(raw, (str, bytes)):
        key = raw.strip()
        if key:
            modules[key] = {"launch": True}
    return modules


def _coerce_module_mapping(value: Any) -> Mapping[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    if isinstance(value, bool):
        return {"launch": value}
    if value is None:
        return {}
    return {}


def _coerce_string_sequence(value: Any) -> List[str]:
    if value is None:
        return []
    if isinstance(value, (str, bytes)):
        normalized = value.strip()
        return [normalized] if normalized else []
    if isinstance(value, Iterable) and not isinstance(value, (str, bytes)):
        result: List[str] = []
        for item in value:
            text = str(item).strip()
            if text:
                result.append(text)
        return result
    text = str(value).strip()
    return [text] if text else []


def _modules_from_host_list(
    names: Iterable[str],
    modules: Mapping[str, Mapping[str, Any]],
) -> List[ModuleDescriptor]:
    result: List[ModuleDescriptor] = []
    seen: set[str] = set()
    for raw_name in names:
        name = str(raw_name).strip()
        if not name or name in seen:
            continue
        seen.add(name)
        raw_config = modules.get(name, {})
        materialized = dict(raw_config)
        result.append(
            ModuleDescriptor(
                name=name,
                display_name=_display_name_for(name, materialized),
                raw_config=materialized,
            )
        )
    return result


def _strip_json_comments(text: str) -> str:
    """Remove C and C++-style comments from a JSONC document."""

    result: list[str] = []
    in_string = False
    string_delimiter = ""
    i = 0
    length = len(text)

    while i < length:
        char = text[i]
        if in_string:
            result.append(char)
            if char == "\\":
                if i + 1 < length:
                    result.append(text[i + 1])
                    i += 1
            elif char == string_delimiter:
                in_string = False
            i += 1
            continue

        if char in {'"', "'"}:
            in_string = True
            string_delimiter = char
            result.append(char)
            i += 1
            continue

        if char == "/" and i + 1 < length:
            next_char = text[i + 1]
            if next_char == "/":
                i += 2
                while i < length and text[i] not in "\r\n":
                    i += 1
                continue
            if next_char == "*":
                i += 2
                while i + 1 < length and (text[i] != "*" or text[i + 1] != "/"):
                    i += 1
                i += 2
                continue

        result.append(char)
        i += 1

    return "".join(result)
