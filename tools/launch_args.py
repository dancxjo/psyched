"""Utilities for translating TOML configuration into ``ros2 launch`` arguments."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, MutableMapping

try:  # Python 3.11+
    import tomllib  # type: ignore[attr-defined]
except ModuleNotFoundError:  # pragma: no cover - fallback for Python <3.11
    import tomli as tomllib  # type: ignore[no-redef]


TomlMapping = Mapping[str, object]


@dataclass(slots=True)
class _FormatContext:
    """Shared state for formatting TOML values into launch arguments."""

    key: str
    value: object

    def as_launch_argument(self) -> str:
        """Render the ``key:=value`` pair expected by ``ros2 launch``."""

        return f"{self.key}:={self._format_value(self.value)}"

    @staticmethod
    def _format_value(value: object) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, (int, float)):
            return str(value)
        if isinstance(value, str):
            escaped = value.replace("\"", r"\"")
            return f'"{escaped}"'
        if value is None:
            return '""'
        raise TypeError(f"Unsupported launch argument type: {type(value)!r}")


def _is_flat_mapping(candidate: TomlMapping) -> bool:
    """Return ``True`` when ``candidate`` only contains scalar-compatible values."""

    return all(not isinstance(value, Mapping) for value in candidate.values())


def _extract_arguments(candidate: object) -> TomlMapping | None:
    """Return the launch argument mapping from ``candidate`` when available."""

    if not isinstance(candidate, Mapping):
        return None
    arguments = candidate.get("arguments")
    if isinstance(arguments, Mapping):
        return arguments
    if _is_flat_mapping(candidate):
        return candidate
    return None


def _resolve_argument_table(raw: TomlMapping, module: str | None) -> TomlMapping:
    """Return the mapping of launch arguments from a parsed TOML object."""

    if module:
        modules = raw.get("modules")
        if isinstance(modules, Mapping):
            module_entry = modules.get(module)
            if isinstance(module_entry, Mapping):
                launch_entry = module_entry.get("launch")
                launch_args = _extract_arguments(launch_entry)
                if launch_args is not None:
                    return launch_args
                explicit_args = module_entry.get("arguments")
                if isinstance(explicit_args, Mapping):
                    return explicit_args
                if _is_flat_mapping(module_entry):
                    return module_entry
        # Fallback: allow top-level table named after the module.
        module_table = raw.get(module)
        if isinstance(module_table, Mapping) and _is_flat_mapping(module_table):
            return module_table

    # Legacy format: module-specific TOML already scoped to the arguments table.
    launch = raw.get("launch")
    if isinstance(launch, Mapping):
        arguments = launch.get("arguments")
        if isinstance(arguments, Mapping):
            return arguments
        if _is_flat_mapping(launch):
            return launch
    arguments = raw.get("arguments")
    if isinstance(arguments, Mapping):
        return arguments
    if _is_flat_mapping(raw):
        return raw
    return {}


def toml_to_launch_arguments(path: Path, module: str | None = None) -> list[str]:
    """Convert ``path`` to a list of ``name:=value`` launch argument strings."""

    path = Path(path)
    if not path.exists():
        return []
    text = path.read_text(encoding="utf-8")
    if not text.strip():
        return []
    data = tomllib.loads(text)
    if not isinstance(data, MutableMapping):  # pragma: no cover - tomllib guarantees dict
        return []
    arguments = _resolve_argument_table(data, module)
    result: list[str] = []
    for key, value in arguments.items():
        ctx = _FormatContext(key=str(key), value=value)
        result.append(ctx.as_launch_argument())
    return result


def main(argv: Iterable[str] | None = None) -> int:
    """CLI entry point used by shell scripts."""

    import argparse

    parser = argparse.ArgumentParser(
        description="Render ros2 launch arguments from a TOML configuration file.",
    )
    parser.add_argument("toml_path", help="Path to the configuration TOML file")
    parser.add_argument(
        "--module",
        help="Optional module scope; when provided the tool inspects modules.<module>",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)
    for line in toml_to_launch_arguments(Path(args.toml_path), module=args.module):
        print(line)
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())
