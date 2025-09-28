"""Utilities for rendering ROS 2 launch arguments from TOML configuration."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping

try:  # Python 3.11+
    import tomllib  # type: ignore[attr-defined]
except ModuleNotFoundError:  # pragma: no cover - fallback for Python <3.11
    import tomli as tomllib  # type: ignore[no-redef]


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


def _resolve_argument_table(raw: Mapping[str, object]) -> Mapping[str, object]:
    """Return the mapping of launch arguments from a parsed TOML object."""

    if not raw:
        return {}
    launch = raw.get("launch")
    if isinstance(launch, Mapping):
        arguments = launch.get("arguments")
        if isinstance(arguments, Mapping):
            return arguments
        return launch
    arguments = raw.get("arguments")
    if isinstance(arguments, Mapping):
        return arguments
    # Fallback: if the root mapping only contains simple values, treat them as args.
    if all(not isinstance(value, Mapping) for value in raw.values()):
        return raw
    return {}


def toml_to_launch_arguments(path: Path) -> list[str]:
    """Convert ``path`` to a list of ``ros2 launch`` style ``name:=value`` strings."""

    path = Path(path)
    if not path.exists():
        return []
    text = path.read_text(encoding="utf-8")
    if not text.strip():
        return []
    data = tomllib.loads(text)
    if not isinstance(data, Mapping):  # pragma: no cover - tomllib guarantees dict
        return []
    arguments = _resolve_argument_table(data)
    result: list[str] = []
    for key, value in arguments.items():
        ctx = _FormatContext(key=str(key), value=value)
        result.append(ctx.as_launch_argument())
    return result


def main(argv: Iterable[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="Render ros2 launch arguments from a TOML configuration file."
    )
    parser.add_argument("toml_path", help="Path to the module configuration TOML file")
    args = parser.parse_args(list(argv) if argv is not None else None)
    for line in toml_to_launch_arguments(Path(args.toml_path)):
        print(line)
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())
