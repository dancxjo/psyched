"""Module metadata loader for the simplified pilot backend."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, MutableMapping, Optional

import tomllib

__all__ = ["ModuleCatalog", "ModuleInfo"]


@dataclass(slots=True)
class ModuleInfo:
    """Metadata describing a module's pilot-facing assets."""

    name: str
    display_name: str
    description: str
    pilot_assets: bool

    @property
    def slug(self) -> str:
        return self.name.replace("_", "-")

    @property
    def dashboard_url(self) -> Optional[str]:
        if not self.pilot_assets:
            return None
        return f"/modules/{self.name}/"

    def to_dict(self) -> Dict[str, object]:
        payload: Dict[str, object] = {
            "name": self.name,
            "display_name": self.display_name,
            "description": self.description,
            "slug": self.slug,
            "has_pilot": self.pilot_assets,
        }
        if self.dashboard_url:
            payload["dashboard_url"] = self.dashboard_url
        return payload


class ModuleCatalog:
    """Discover module metadata from ``module.toml`` manifests."""

    def __init__(self, modules_root: Path):
        self.modules_root = Path(modules_root)
        self._cache: Dict[str, ModuleInfo] | None = None

    def refresh(self) -> None:
        """Invalidate cached module metadata."""

        self._cache = None

    def list_modules(self) -> List[ModuleInfo]:
        """Return a sorted list of known modules."""

        snapshot = self._ensure_cache()
        return sorted(snapshot.values(), key=lambda info: info.display_name.lower())

    def get_module(self, name: str) -> ModuleInfo:
        """Return metadata for a specific module."""

        snapshot = self._ensure_cache()
        try:
            return snapshot[name]
        except KeyError as exc:
            raise KeyError(f"Module not found: {name}") from exc

    # Internal helpers -------------------------------------------------
    def _ensure_cache(self) -> Dict[str, ModuleInfo]:
        if self._cache is None:
            self._cache = self._load()
        return self._cache

    def _load(self) -> Dict[str, ModuleInfo]:
        modules: Dict[str, ModuleInfo] = {}
        if not self.modules_root.exists():
            return modules

        for entry in sorted(self.modules_root.iterdir()):
            if not entry.is_dir() or entry.name.startswith("."):
                continue
            manifest = entry / "module.toml"
            pilot_dir = entry / "pilot"

            if manifest.exists():
                info = self._parse_manifest(entry.name, manifest, pilot_dir)
            else:
                info = ModuleInfo(
                    name=entry.name,
                    display_name=entry.name.replace("_", " ").title(),
                    description="",
                    pilot_assets=pilot_dir.exists(),
                )

            modules[info.name] = info

        return modules

    def _parse_manifest(self, module_name: str, manifest: Path, pilot_dir: Path) -> ModuleInfo:
        text = manifest.read_text(encoding="utf-8")
        try:
            manifest_data = tomllib.loads(text)
        except tomllib.TOMLDecodeError as exc:  # pragma: no cover - invalid manifest
            raise RuntimeError(f"Failed to parse {manifest}: {exc}") from exc

        name = str(manifest_data.get("name") or module_name)
        pilot_cfg = self._coerce_mapping(manifest_data.get("pilot"))

        display_name = str(
            pilot_cfg.get("display_name")
            or manifest_data.get("display_name")
            or name.replace("_", " ").title(),
        )
        description = str(
            pilot_cfg.get("description")
            or manifest_data.get("description")
            or "",
        )

        return ModuleInfo(
            name=name,
            display_name=display_name,
            description=description,
            pilot_assets=pilot_dir.exists(),
        )

    @staticmethod
    def _coerce_mapping(value: object | None) -> MutableMapping[str, object]:
        if isinstance(value, Mapping):
            return dict(value)
        return {}
