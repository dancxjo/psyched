"""Module metadata loader for the pilot backend."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, MutableMapping
import socket

try:  # Python 3.11+
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - fallback for old interpreters
    import tomli as tomllib  # type: ignore


def _as_list(value: object | None) -> List[str]:
    if value is None:
        return []
    if isinstance(value, list):
        return [str(item) for item in value]
    return [str(value)]


from .qos import QosConfig


def _host_shortname() -> str:
    """Return the short hostname for the current machine."""

    try:
        hostname = socket.gethostname()
    except Exception:  # pragma: no cover - extremely unlikely
        return "host"
    if not hostname:
        return "host"
    return hostname.split(".")[0]


_HOST_HEALTH_TOPIC = f"/hosts/{_host_shortname()}/health"


def _extract_pilot_block(contents: str) -> str | None:
    lines = contents.splitlines()
    captured: List[str] = []
    capturing = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('[pilot]') or stripped.startswith('[[pilot.'):
            capturing = True
            captured.append(line)
            continue
        if capturing and stripped.startswith('['):
            break
        if capturing:
            captured.append(line)
    return '\n'.join(captured) if captured else None


@dataclass(slots=True)
class ModuleTopic:
    """Topic subscription definition for a module."""

    topic: str
    type: str
    access: str = "ro"
    qos: QosConfig = field(default_factory=QosConfig)
    presentation: Optional[str] = None


@dataclass(slots=True)
class ModuleCommands:
    """Command metadata for a module."""

    mod: List[str] = field(default_factory=lambda: ["setup", "restart", "status", "logs"])
    system: List[str] = field(default_factory=lambda: ["start", "stop", "restart", "status"])


@dataclass(slots=True)
class ModuleInfo:
    """Full module description used by the frontend."""

    name: str
    display_name: str
    description: str
    topics: List[ModuleTopic] = field(default_factory=list)
    commands: ModuleCommands = field(default_factory=ModuleCommands)
    regimes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, object]:
        return {
            "name": self.name,
            "display_name": self.display_name,
            "description": self.description,
            "regimes": list(self.regimes),
            "topics": [
                {
                    "topic": topic.topic,
                    "type": topic.type,
                    "access": topic.access,
                    "presentation": topic.presentation,
                    "qos": topic.qos.asdict(),
                }
                for topic in self.topics
            ],
            "commands": {
                "mod": list(self.commands.mod),
                "system": list(self.commands.system),
            },
        }


class ModuleCatalog:
    """Loads module information from `module.toml` manifests."""

    def __init__(self, modules_root: Path):
        self.modules_root = Path(modules_root)
        self._cache: Dict[str, ModuleInfo] | None = None

    def _load(self) -> Dict[str, ModuleInfo]:
        modules: Dict[str, ModuleInfo] = {}
        if not self.modules_root.exists():
            return modules

        for entry in sorted(self.modules_root.iterdir()):
            if not entry.is_dir():
                continue
            if entry.name.startswith("."):
                continue
            manifest = entry / "module.toml"
            if not manifest.exists():
                continue

            contents = manifest.read_text(encoding="utf-8")
            block = _extract_pilot_block(contents)
            if not block:
                continue
            data = tomllib.loads(block)
            pilot_cfg: MutableMapping[str, object] = data.get("pilot", {})  # type: ignore[assignment]
            name = data.get("name", entry.name)
            display_name = str(pilot_cfg.get("display_name", name.title()))
            description = str(pilot_cfg.get("description", data.get("description", "")))

            default_commands = ModuleCommands()
            regimes = _as_list(pilot_cfg.get("regimes")) or ["general"]

            commands = ModuleCommands(
                mod=_as_list(pilot_cfg.get("mod_commands")) or list(default_commands.mod),
                system=_as_list(pilot_cfg.get("system_commands")) or list(default_commands.system),
            )

            topics_cfg = pilot_cfg.get("topics", [])
            topics: List[ModuleTopic] = []
            if isinstance(topics_cfg, list):
                for topic_entry in topics_cfg:
                    if not isinstance(topic_entry, MutableMapping):
                        continue
                    topic_name = str(topic_entry.get("name") or topic_entry.get("topic") or "")
                    if not topic_name:
                        continue
                    normalized_name = topic_name.lstrip("/")
                    if normalized_name == "host/health":
                        topic_name = _HOST_HEALTH_TOPIC
                    type_name = str(topic_entry.get("type", "std_msgs/msg/String"))
                    access = str(topic_entry.get("access", "ro"))
                    presentation = (
                        str(topic_entry.get("presentation")) if topic_entry.get("presentation") else None
                    )
                    qos_data = topic_entry.get("qos")
                    qos = QosConfig.from_mapping(qos_data if isinstance(qos_data, Mapping) else None)
                    topics.append(
                        ModuleTopic(
                            topic=topic_name,
                            type=type_name,
                            access=access,
                            qos=qos,
                            presentation=presentation,
                        )
                    )

            module_info = ModuleInfo(
                name=str(name),
                display_name=display_name,
                description=description,
                topics=topics,
                commands=commands,
                regimes=regimes,
            )
            modules[module_info.name] = module_info

        return modules

    def refresh(self) -> None:
        self._cache = None

    def list_modules(self) -> List[ModuleInfo]:
        if self._cache is None:
            self._cache = self._load()
        return list(self._cache.values())

    def get_module(self, name: str) -> ModuleInfo:
        if self._cache is None:
            self._cache = self._load()
        try:
            return self._cache[name]
        except KeyError as exc:  # pragma: no cover - guard for callers
            raise KeyError(f"Unknown module: {name}") from exc

    def iter_topics(self) -> Iterable[ModuleTopic]:
        for module in self.list_modules():
            yield from module.topics
