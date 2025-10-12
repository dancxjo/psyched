"""Utilities for translating configuration files into ``ros2 launch`` arguments."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, MutableMapping

import tomllib


ConfigMapping = Mapping[str, object]


@dataclass(slots=True)
class _FormatContext:
    """Shared state for formatting configuration values into launch arguments."""

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


def _is_flat_mapping(candidate: ConfigMapping) -> bool:
    """Return ``True`` when ``candidate`` only contains scalar-compatible values."""

    return all(not isinstance(value, Mapping) for value in candidate.values())


def _extract_arguments(candidate: object) -> ConfigMapping | None:
    """Return the launch argument mapping from ``candidate`` when available."""

    if not isinstance(candidate, Mapping):
        return None
    arguments = candidate.get("arguments")
    if isinstance(arguments, Mapping):
        return arguments
    if _is_flat_mapping(candidate):
        return candidate
    return None


def _module_entry(raw: ConfigMapping, module: str) -> ConfigMapping | None:
    """Return the configuration table for ``module`` when available."""

    config_section = raw.get("config")
    if not isinstance(config_section, Mapping):
        return None
    modules_section = config_section.get("mod")
    if not isinstance(modules_section, Mapping):
        return None
    module_entry = modules_section.get(module)
    if isinstance(module_entry, Mapping):
        return module_entry
    return None


def _resolve_argument_table(raw: ConfigMapping, module: str) -> ConfigMapping:
    """Return the mapping of launch arguments from a parsed host manifest."""

    module_entry = _module_entry(raw, module)
    if module_entry is None:
        return {}

    launch_entry = module_entry.get("launch")
    launch_args = _extract_arguments(launch_entry)
    if launch_args is not None:
        return launch_args

    explicit_args = module_entry.get("arguments")
    if isinstance(explicit_args, Mapping):
        return explicit_args

    if _is_flat_mapping(module_entry):
        return module_entry

    return {}


def toml_to_launch_arguments(path: Path, module: str | None = None) -> list[str]:
    """Convert ``path`` to ``name:=value`` launch arguments.

    Host manifests are authored in TOML. When ``module`` is provided, the
    helper extracts ``config.mod.<module>.launch.arguments`` (or compatible
    flat tables) and formats each value for ``ros2 launch`` consumption.
    """

    path = Path(path)
    if not path.exists():
        return []
    text = path.read_text(encoding="utf-8")
    if not text.strip():
        return []
    try:
        data = tomllib.loads(text)
    except tomllib.TOMLDecodeError as error:
        raise ValueError(
            f"Invalid TOML configuration in {path}: {error}",
        ) from error
    if not isinstance(data, MutableMapping):
        raise TypeError(
            f"Expected TOML configuration {path} to decode to a mapping, got {type(data)!r}",
        )
    if module is None:
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
        description="Render ros2 launch arguments from a configuration file.",
    )
    parser.add_argument("toml_path", help="Path to the configuration file")
    parser.add_argument(
        "--module",
        help="Optional module scope; when provided the tool inspects config.mod.<module>",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)
    for line in toml_to_launch_arguments(Path(args.toml_path), module=args.module):
        print(line)
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())
