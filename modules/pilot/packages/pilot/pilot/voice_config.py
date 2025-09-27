"""Utilities for loading and persisting voice module configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml


@dataclass(slots=True)
class VoiceConfigStore:
    """Simple YAML-backed store for the voice module configuration."""

    path: Path

    def __post_init__(self) -> None:  # pragma: no cover - trivial validation
        self.path = Path(self.path)

    def load(self) -> dict:
        """Return the current configuration dictionary.

        Missing files yield an empty dictionary so callers can fall back to
        sensible defaults without raising an exception.
        """

        if not self.path.exists():
            return {}
        try:
            with self.path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except FileNotFoundError:  # pragma: no cover - racy deletion handling
            return {}
        except yaml.YAMLError as exc:  # pragma: no cover - invalid YAML during runtime
            raise ValueError(f"Failed to parse voice config: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError("Voice config YAML must represent a mapping")
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
        with tmp_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(
                data,
                handle,
                default_flow_style=False,
                sort_keys=False,
            )
        tmp_path.replace(self.path)
