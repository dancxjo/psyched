"""TOML-backed store for the voice module configuration."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

try:  # Python 3.11+
    import tomllib  # type: ignore[attr-defined]
except ModuleNotFoundError:  # pragma: no cover - fallback for Python <3.11
    import tomli as tomllib  # type: ignore[no-redef]


@dataclass(slots=True)
class VoiceConfigStore:
    """Simple TOML-backed store for the voice module configuration."""

    path: Path

    def __post_init__(self) -> None:  # pragma: no cover - trivial validation
        self.path = Path(self.path)

    def load(self) -> dict:
        """Return the current configuration dictionary."""

        if not self.path.exists():
            return {}
        try:
            text = self.path.read_text(encoding="utf-8")
        except FileNotFoundError:  # pragma: no cover - racy deletion handling
            return {}
        if not text.strip():
            return {}
        try:
            data = tomllib.loads(text)
        except tomllib.TOMLDecodeError as exc:  # type: ignore[attr-defined]
            raise ValueError(f"Failed to parse voice config: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError("Voice config TOML must represent a mapping")
        return dict(data)

    def update(self, updates: Mapping[str, object]) -> dict:
        """Merge ``updates`` into the on-disk configuration and persist it."""

        existing = self.load()
        changed = dict(existing)
        for key, value in updates.items():
            if value is None:
                continue
            changed[key] = value
        self._write(changed)
        return changed

    def _write(self, data: Mapping[str, object]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.path.with_suffix(self.path.suffix + ".tmp")
        lines = [self._format_entry(key, value) for key, value in data.items()]
        tmp_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        tmp_path.replace(self.path)

    @staticmethod
    def _format_entry(key: str, value: object) -> str:
        if isinstance(value, bool):
            encoded = "true" if value else "false"
        elif isinstance(value, (int, float)):
            encoded = str(value)
        elif isinstance(value, str):
            escaped = value.replace("\"", r"\"")
            encoded = f'"{escaped}"'
        else:  # pragma: no cover - defensive guard
            raise TypeError(f"Unsupported value type for voice config: {type(value)!r}")
        return f"{key} = {encoded}"
