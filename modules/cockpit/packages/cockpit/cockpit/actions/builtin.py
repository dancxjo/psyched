"""Built-in module actions backed by the shared cockpit ROS client."""

from __future__ import annotations

import asyncio

from typing import Any, Dict, Iterable, Mapping, Optional

from cockpit.actions import ActionError, ActionResult, ModuleAction
from cockpit.ros import RosClient, ServiceCallError, TopicStreamError


def register_builtin_actions(
    registry,
    *,
    ros: RosClient,
    modules: Iterable[str],
) -> None:
    """Register default ROS-backed actions for the provided modules."""

    for module_name in modules:
        registry.register_many(
            module_name,
            [
                _topic_stream_action(module_name, ros),
                _service_call_action(module_name, ros),
            ],
        )


def _topic_stream_action(module: str, ros: RosClient) -> ModuleAction:
    parameters: Dict[str, Any] = {
        "type": "object",
        "properties": {
            "topic": {
                "type": "string",
                "description": "ROS topic name to bridge",
            },
            "message_type": {
                "type": "string",
                "description": "Fully-qualified ROS message type for the topic",
            },
            "role": {
                "type": "string",
                "enum": ["subscribe", "publish", "both"],
                "default": "subscribe",
            },
            "queue_length": {
                "type": "integer",
                "minimum": 1,
                "default": 10,
            },
            "qos": {
                "type": "object",
                "description": "Optional QoS overrides (reliability, durability)",
                "additionalProperties": True,
            },
        },
        "required": ["topic", "message_type"],
        "additionalProperties": False,
    }

    async def handler(context):
        topic = context.get_argument("topic")
        message_type = context.get_argument("message_type")
        role = context.get_argument("role", "subscribe")
        queue_length = context.get_argument("queue_length", 10)
        qos_override = context.get_argument("qos", None)

        if not isinstance(topic, str) or not topic.strip():
            raise ActionError("topic must be a non-empty string")
        if not isinstance(message_type, str) or not message_type.strip():
            raise ActionError("message_type must be a non-empty string")
        if not isinstance(role, str):
            raise ActionError("role must be a string")
        try:
            queue_length_int = int(queue_length)
        except (TypeError, ValueError):
            raise ActionError("queue_length must be an integer greater than zero")
        if queue_length_int < 1:
            raise ActionError("queue_length must be an integer greater than zero")
        if qos_override is not None and not isinstance(qos_override, Mapping):
            raise ActionError("qos must be expressed as a JSON object")

        try:
            stream = await ros.create_topic_stream(
                module=module,
                topic=topic,
                message_type=message_type,
                role=role,
                queue_length=queue_length_int,
                qos=qos_override if isinstance(qos_override, Mapping) else None,
                loop=asyncio.get_running_loop(),
            )
        except (TopicStreamError, ValueError) as exc:
            raise ActionError(str(exc)) from exc

        payload = {
            "stream": stream.metadata.to_dict(),
        }
        return ActionResult(payload=payload, stream=stream)

    return ModuleAction(
        name="stream_topic",
        description=f"Stream ROS topic traffic for the {module} module",
        parameters=parameters,
        streaming=True,
        handler=handler,
        returns={
            "type": "object",
            "properties": {
                "stream": {
                    "type": "object",
                    "properties": {
                        "id": {"type": "string"},
                        "module": {"type": "string"},
                        "topic": {"type": "string"},
                        "message_type": {"type": "string"},
                        "role": {"type": "string"},
                    },
                    "required": ["id", "module", "topic", "message_type", "role"],
                    "additionalProperties": False,
                },
            },
            "required": ["stream"],
            "additionalProperties": False,
        },
    )


def _service_call_action(module: str, ros: RosClient) -> ModuleAction:
    parameters: Dict[str, Any] = {
        "type": "object",
        "properties": {
            "service": {
                "type": "string",
                "description": "ROS service name to invoke",
            },
            "service_type": {
                "type": "string",
                "description": "Optional service type hint",
            },
            "arguments": {
                "type": "object",
                "description": "Service request arguments",
                "additionalProperties": True,
            },
            "timeout_ms": {
                "type": "integer",
                "description": "Optional timeout for the service call in milliseconds",
                "minimum": 1,
                "default": 8000,
            },
        },
        "required": ["service"],
        "additionalProperties": False,
    }

    async def handler(context):
        service = context.get_argument("service")
        service_type = context.get_argument("service_type")
        arguments = context.get_argument("arguments", {})
        timeout_ms = context.get_argument("timeout_ms", 8000)

        if not isinstance(service, str) or not service.strip():
            raise ActionError("service must be a non-empty string")
        if service_type is not None and not isinstance(service_type, str):
            raise ActionError("service_type must be a string when provided")
        if arguments is None:
            arguments = {}
        if not isinstance(arguments, Mapping):
            raise ActionError("arguments must be a JSON object")
        try:
            timeout_seconds = float(timeout_ms) / 1000.0
        except (TypeError, ValueError):
            raise ActionError("timeout_ms must be a numeric value")
        if timeout_seconds <= 0:
            raise ActionError("timeout_ms must be positive")

        try:
            result = await ros.call_service(
                service_name=service,
                service_type=service_type,
                arguments=arguments,
                timeout=timeout_seconds,
            )
        except (ServiceCallError, ValueError) as exc:
            raise ActionError(str(exc)) from exc

        return ActionResult(payload={"result": result})

    return ModuleAction(
        name="call_service",
        description=f"Invoke ROS services on behalf of the {module} module",
        parameters=parameters,
        streaming=False,
        handler=handler,
        returns={
            "type": "object",
            "properties": {
                "result": {"type": "object"},
            },
            "required": ["result"],
            "additionalProperties": False,
        },
    )
