"""Utilities for loading host configuration data used by the pilot."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional

import tomli_w
import tomllib

__all__ = [
    "HostConfigError",
    "ModuleDescriptor",
    "discover_active_modules",
    "load_host_config",
    "save_host_config",
    "set_module_config",
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
        Filesystem path to a TOML configuration file.

    Returns
    -------
    MutableMapping[str, Any]
        Parsed host configuration data.
    """

    resolved = Path(path)
    if not resolved.exists():
        raise HostConfigError(f"Host configuration not found: {resolved}")

    if resolved.suffix.lower() != ".toml":
        raise HostConfigError(
            f"Unsupported host configuration format for '{resolved.name}'. "
            "Host manifests must be expressed in TOML.",
        )

    text = resolved.read_text(encoding="utf-8")
    try:
        data = tomllib.loads(text)
    except tomllib.TOMLDecodeError as exc:
        raise HostConfigError(f"Failed to parse host configuration: {exc}") from exc
    if not isinstance(data, MutableMapping):
        raise HostConfigError(
            f"Expected mapping at root of TOML config but received {type(data)!r}",
        )
    return data


def save_host_config(path: Path | str, data: Mapping[str, Any]) -> None:
    """Persist *data* to *path* in TOML format.

    Parameters
    ----------
    path:
        Filesystem path for the TOML document.
    data:
        Parsed host configuration that should be serialized back to disk.
    """

    resolved = Path(path)
    text = tomli_w.dumps(_normalize_for_toml(data))
    if not text.endswith("\n"):
        text += "\n"
    resolved.write_text(text, encoding="utf-8")


def set_module_config(
    config: MutableMapping[str, Any],
    module_name: str,
    module_config: Mapping[str, Any],
) -> None:
    """Replace the configuration block for ``module_name`` in ``config``.

    Parameters
    ----------
    config:
        Parsed host configuration data that will be mutated in place.
    module_name:
        Canonical module identifier.
    module_config:
        Mapping describing the module settings to persist.
    """

    if not module_name or not isinstance(module_name, str):
        raise ValueError("module_name must be a non-empty string")

    module_block = _ensure_module_mapping(config)
    module_block[module_name] = _clone_config_value(module_config)


def discover_active_modules(config: Mapping[str, Any]) -> List[ModuleDescriptor]:
    """Return module descriptors for modules active on the host.

    When ``host.modules`` is declared, it is treated as the canonical list of
    cockpit modules. Config entries under ``config.mod`` supply metadata for
    those modules without enabling them implicitly. When ``host.modules`` is
    absent, module directives with a truthy ``launch`` flag are still
    respected for compatibility with ad-hoc configurations.
    """

    if not config:
        return []

    modules: Dict[str, Mapping[str, Any]] = {}
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


def _ensure_module_mapping(config: MutableMapping[str, Any]) -> MutableMapping[str, Any]:
    config_section = _ensure_mapping(config, "config")
    mod_section = _ensure_mapping(config_section, "mod")
    return mod_section


def _ensure_mapping(
    container: MutableMapping[str, Any], key: str
) -> MutableMapping[str, Any]:
    current = container.get(key)
    if current is None:
        mapping: MutableMapping[str, Any] = {}
        container[key] = mapping
        return mapping
    if isinstance(current, MutableMapping):
        return current
    raise HostConfigError(
        f"Expected mapping for '{key}' but received {type(current).__name__}"
    )


def _clone_config_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _clone_config_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clone_config_value(item) for item in value]
    return value


def _normalize_for_toml(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _normalize_for_toml(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_normalize_for_toml(item) for item in value]
    return value


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
