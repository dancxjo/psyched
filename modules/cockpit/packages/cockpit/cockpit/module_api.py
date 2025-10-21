"""Module API loader for cockpit action definitions."""

from __future__ import annotations

import asyncio
import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional

import tomllib

from .actions import ActionError, ActionResult, ModuleAction

try:  # pragma: no cover - exercised implicitly when ROS dependencies exist
    from .ros import RosClient, ServiceCallError, TopicPublishError, TopicStreamError
except ImportError:  # pragma: no cover - fallback for environments without rclpy
    RosClient = Any  # type: ignore[assignment]

    class ServiceCallError(Exception):
        """Placeholder error raised when ROS dependencies are unavailable."""

    class TopicStreamError(Exception):
        """Placeholder error raised when ROS dependencies are unavailable."""

    class TopicPublishError(Exception):
        """Placeholder error raised when ROS dependencies are unavailable."""

_LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class ActionDefinition:
    """Describe a module-level cockpit action defined in JSON."""

    name: str
    description: str
    kind: str
    defaults: Mapping[str, Any]
    parameters: Optional[Mapping[str, Any]] = None
    returns: Optional[Mapping[str, Any]] = None


@dataclass(slots=True)
class ModuleApiDefinition:
    """Container for the parsed cockpit API definition for a module."""

    version: int
    actions: List[ActionDefinition]


def load_module_api_definition(module_dir: Path) -> Optional[ModuleApiDefinition]:
    """Return the parsed cockpit API definition for *module_dir* if present."""

    api_path = Path(module_dir) / "cockpit" / "api" / "actions.json"
    if not api_path.exists():
        return None
    try:
        payload = json.loads(api_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        _LOGGER.warning("Failed to parse %s: %s", api_path, exc)
        return None

    version_raw = payload.get("version", 1)
    try:
        version = int(version_raw)
    except (TypeError, ValueError):
        version = 1

    raw_actions = payload.get("actions", [])
    if not isinstance(raw_actions, Iterable):
        return ModuleApiDefinition(version=version, actions=[])

    definitions: List[ActionDefinition] = []
    for raw in raw_actions:
        if not isinstance(raw, Mapping):
            continue
        name = str(raw.get("name", "")).strip()
        kind = str(raw.get("kind", "")).strip()
        if not name or not kind:
            _LOGGER.debug("Skipping action with missing name or kind in %s", api_path)
            continue
        description = str(raw.get("description", "")).strip()
        defaults = _coerce_mapping(raw.get("defaults"))
        parameters = _coerce_mapping(raw.get("parameters")) or None
        returns = _coerce_mapping(raw.get("returns")) or None
        definitions.append(
            ActionDefinition(
                name=name,
                description=description,
                kind=kind,
                defaults=defaults,
                parameters=parameters,
                returns=returns,
            )
        )
    return ModuleApiDefinition(version=version, actions=definitions)


def register_module_api_actions(
    registry,
    *,
    modules_root: Path,
    ros: RosClient,
    extra_roots: Iterable[Path] = (),
) -> None:
    """Register module-defined cockpit actions from JSON definitions.

    Parameters
    ----------
    registry:
        Target :class:`~cockpit.actions.ActionRegistry` that stores the
        resulting actions.
    modules_root:
        Primary directory containing module manifests and cockpit API
        definitions. This is typically the active workspace managed by
        ``psh``.
    ros:
        ROS client used when constructing action handlers.
    extra_roots:
        Optional additional directories that should be scanned when the
        primary workspace does not expose cockpit API files. This allows the
        cockpit to fall back to repository checkouts where the action
        definitions live even when the active workspace omits them (e.g. a
        stripped deployment tree). Later directories only contribute actions
        that were not already registered by earlier roots, ensuring
        workspace-specific overrides remain authoritative.
    """

    roots: List[Path] = [Path(modules_root)]
    roots.extend(Path(root) for root in extra_roots)

    visited: set[Path] = set()
    registered: set[tuple[str, str]] = set()

    for candidate in roots:
        try:
            resolved = candidate.resolve()
        except OSError:
            resolved = candidate
        if resolved in visited:
            continue
        visited.add(resolved)

        if not candidate.exists():
            continue

        for module_dir in sorted(p for p in candidate.iterdir() if p.is_dir()):
            api = load_module_api_definition(module_dir)
            if api is None:
                continue
            module_name = _canonical_module_name(module_dir)
            for definition in api.actions:
                key = (module_name, definition.name)
                if key in registered:
                    continue
                try:
                    action = _build_action(module_name, definition, ros)
                except ValueError as exc:
                    _LOGGER.warning(
                        "Skipping action %s.%s due to configuration error: %s",
                        module_name,
                        definition.name,
                        exc,
                    )
                    continue
                registry.register(module_name, action)
                registered.add(key)


def _canonical_module_name(module_dir: Path) -> str:
    """Return the canonical module name for ``module_dir``.

    The cockpit surfaces actions using the module identifier that other
    subsystems reference (e.g. the cockpit UI, :mod:`psh`, ROS bridge). On
    packaged hosts the on-disk directory can diverge from this identifier when
    version suffixes are appended (``pilot-unit``), so we prefer the manifest's
    ``name`` field when available and fall back to the directory name.
    """

    manifest = module_dir / "module.toml"
    if manifest.exists():
        try:
            payload = tomllib.loads(manifest.read_text(encoding="utf-8"))
        except tomllib.TOMLDecodeError as exc:  # pragma: no cover - invalid manifest
            _LOGGER.warning("Failed to parse module manifest %s: %s", manifest, exc)
        else:
            name = payload.get("name")
            if isinstance(name, str) and name.strip():
                return name.strip()
    return module_dir.name


def _build_action(
    module: str,
    definition: ActionDefinition,
    ros: RosClient,
) -> ModuleAction:
    kind = definition.kind.lower()
    if kind == "stream-topic":
        return _build_stream_topic_action(module, definition, ros)
    if kind == "call-service":
        return _build_call_service_action(module, definition, ros)
    if kind == "publish-topic":
        return _build_publish_topic_action(module, definition, ros)
    raise ValueError(f"Unsupported action kind: {definition.kind}")


def _allowed_argument_keys(parameters: Optional[Mapping[str, Any]]) -> set[str]:
    if not parameters:
        return set()
    props = parameters.get("properties")
    if isinstance(props, Mapping):
        return {str(key) for key in props.keys()}
    return set()


def _merge_arguments(
    defaults: Mapping[str, Any],
    provided: Mapping[str, Any],
    allowed: set[str],
) -> Dict[str, Any]:
    merged = dict(defaults)
    for key, value in provided.items():
        if allowed and key not in allowed:
            continue
        merged[key] = value
    return merged


def _build_stream_topic_action(
    module: str,
    definition: ActionDefinition,
    ros: RosClient,
) -> ModuleAction:
    defaults = dict(definition.defaults)
    allowed_keys = _allowed_argument_keys(definition.parameters)

    async def handler(context) -> ActionResult:
        arguments = context.arguments if isinstance(context.arguments, Mapping) else {}
        merged = _merge_arguments(defaults, arguments, allowed_keys)

        topic = merged.get("topic")
        message_type = merged.get("message_type")
        role = merged.get("role", "subscribe")
        queue_length = merged.get("queue_length", 10)
        qos_override = merged.get("qos")

        if not isinstance(topic, str) or not topic.strip():
            raise ActionError("Configured topic must be a non-empty string")
        if not isinstance(message_type, str) or not message_type.strip():
            raise ActionError("Configured message_type must be a non-empty string")
        if not isinstance(role, str):
            raise ActionError("Configured role must be a string")
        try:
            queue_len = int(queue_length)
        except (TypeError, ValueError):
            raise ActionError("queue_length must be an integer")
        if queue_len < 1:
            raise ActionError("queue_length must be greater than zero")
        if qos_override is not None and not isinstance(qos_override, Mapping):
            raise ActionError("qos overrides must be expressed as a JSON object")

        try:
            stream = await ros.create_topic_stream(
                module=module,
                topic=topic,
                message_type=message_type,
                role=role,
                queue_length=queue_len,
                qos=qos_override if isinstance(qos_override, Mapping) else None,
                loop=asyncio.get_running_loop(),
            )
        except (TopicStreamError, ValueError) as exc:
            raise ActionError(str(exc)) from exc

        payload = {"stream": stream.metadata.to_dict()}
        return ActionResult(payload=payload, stream=stream)

    return ModuleAction(
        name=definition.name,
        description=definition.description or f"Stream ROS topic traffic for the {module} module",
        parameters=definition.parameters or {},
        handler=handler,
        returns=definition.returns,
        streaming=True,
    )


def _build_call_service_action(
    module: str,
    definition: ActionDefinition,
    ros: RosClient,
) -> ModuleAction:
    defaults = dict(definition.defaults)
    allowed_keys = _allowed_argument_keys(definition.parameters)

    async def handler(context) -> ActionResult:
        arguments = context.arguments if isinstance(context.arguments, Mapping) else {}
        merged = _merge_arguments(defaults, arguments, allowed_keys)

        service = merged.get("service")
        service_type = merged.get("service_type")
        payload = merged.get("arguments", {})
        timeout_ms = merged.get("timeout_ms", 8000)

        if not isinstance(service, str) or not service.strip():
            raise ActionError("Configured service must be a non-empty string")
        if service_type is not None and not isinstance(service_type, str):
            raise ActionError("service_type must be a string when provided")
        if payload is None:
            payload = {}
        if not isinstance(payload, Mapping):
            raise ActionError("Service arguments must be expressed as a JSON object")
        try:
            timeout_seconds = float(timeout_ms) / 1000.0
        except (TypeError, ValueError):
            raise ActionError("timeout_ms must be numeric")
        if timeout_seconds <= 0:
            raise ActionError("timeout_ms must be positive")

        try:
            result = await ros.call_service(
                service_name=service,
                service_type=service_type,
                arguments=payload,
                timeout=timeout_seconds,
            )
        except (ServiceCallError, ValueError) as exc:
            raise ActionError(str(exc)) from exc

        return ActionResult(payload={"result": result})

    return ModuleAction(
        name=definition.name,
        description=definition.description or f"Invoke ROS services on behalf of the {module} module",
        parameters=definition.parameters or {},
        handler=handler,
        returns=definition.returns,
        streaming=False,
    )


def _build_publish_topic_action(
    module: str,
    definition: ActionDefinition,
    ros: RosClient,
) -> ModuleAction:
    defaults = dict(definition.defaults)
    topic_default = defaults.get("topic")
    message_type_default = defaults.get("message_type")
    if not isinstance(topic_default, str) or not topic_default.strip():
        raise ValueError("publish-topic actions must define a non-empty topic default")
    if not isinstance(message_type_default, str) or not message_type_default.strip():
        raise ValueError("publish-topic actions must define a non-empty message_type default")

    payload_default = defaults.get("payload")
    base_payload: Dict[str, Any] = {}
    if isinstance(payload_default, Mapping):
        base_payload.update(dict(payload_default))

    raw_map = defaults.get("payload_map", {})
    payload_map: Dict[str, str] = {}
    if isinstance(raw_map, Mapping):
        for key, value in raw_map.items():
            if isinstance(key, str) and key.strip() and isinstance(value, str) and value.strip():
                payload_map[key.strip()] = value.strip()

    allowed_keys = _allowed_argument_keys(definition.parameters)
    reserved_keys = {"topic", "message_type", "payload", "qos"}

    async def handler(context) -> ActionResult:
        arguments = context.arguments if isinstance(context.arguments, Mapping) else {}
        merged = _merge_arguments(defaults, arguments, allowed_keys | {"payload"})

        topic_value = merged.get("topic", topic_default)
        message_type_value = merged.get("message_type", message_type_default)
        qos_override = merged.get("qos")

        if not isinstance(topic_value, str) or not topic_value.strip():
            raise ActionError("Configured topic must be a non-empty string")
        if not isinstance(message_type_value, str) or not message_type_value.strip():
            raise ActionError("Configured message_type must be a non-empty string")
        topic_value = topic_value.strip()
        message_type_value = message_type_value.strip()

        payload: Dict[str, Any] = dict(base_payload)

        payload_override = merged.get("payload")
        if isinstance(payload_override, Mapping):
            payload.update(dict(payload_override))
        elif payload_override is not None and "payload" in arguments:
            raise ActionError("Payload overrides must be expressed as a JSON object")

        for key in allowed_keys:
            if key in reserved_keys:
                continue
            if key not in arguments:
                continue
            target_field = payload_map.get(key, key)
            if not target_field:
                continue
            payload[target_field] = arguments[key]

        try:
            await ros.publish_topic(
                module=module,
                topic=topic_value,
                message_type=message_type_value,
                payload=payload,
                qos=qos_override if isinstance(qos_override, Mapping) else None,
            )
        except (TopicPublishError, ValueError) as exc:
            raise ActionError(str(exc)) from exc

        return ActionResult(
            payload={
                "status": "published",
                "topic": topic_value,
                "message_type": message_type_value,
            }
        )

    return ModuleAction(
        name=definition.name,
        description=definition.description or f"Publish ROS topic traffic for the {module} module",
        parameters=definition.parameters or {},
        handler=handler,
        returns=definition.returns,
        streaming=False,
    )


def _coerce_mapping(value: object | None) -> MutableMapping[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    return {}


__all__ = [
    "ActionDefinition",
    "ModuleApiDefinition",
    "load_module_api_definition",
    "register_module_api_actions",
]
