from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any, Dict

import pytest

from cockpit.actions import ActionRegistry
from cockpit.module_api import load_module_api_definition, register_module_api_actions


def write_actions_file(root: Path, module: str, payload: str) -> Path:
    module_dir = root / module
    api_dir = module_dir / "cockpit" / "api"
    api_dir.mkdir(parents=True, exist_ok=True)
    actions_path = api_dir / "actions.json"
    actions_path.write_text(payload, encoding="utf-8")
    return module_dir


def test_load_module_api_definition_parses_actions(tmp_path: Path) -> None:
    payload = """
    {
      "version": 1,
      "actions": [
        {
          "name": "debug_stream",
          "description": "Stream pilot debug",
          "kind": "stream-topic",
          "defaults": {"topic": "/pilot/debug", "message_type": "std_msgs/msg/String"},
          "parameters": {"type": "object", "properties": {"queue_length": {"type": "integer"}}}
        }
      ]
    }
    """
    module_dir = write_actions_file(tmp_path, "pilot", payload)

    definition = load_module_api_definition(module_dir)
    assert definition is not None
    assert definition.version == 1
    assert len(definition.actions) == 1
    action = definition.actions[0]
    assert action.name == "debug_stream"
    assert action.kind == "stream-topic"
    assert action.defaults["topic"] == "/pilot/debug"
    assert action.parameters is not None


class _FakeStream:
    def __init__(self, metadata: dict[str, str]):
        self.metadata = type("Metadata", (), {"to_dict": lambda self: metadata.copy()})()


class _FakeRos:
    def __init__(self) -> None:
        self._streams: list[_FakeStream] = []
        self._service_calls: list[Dict[str, Any]] = []
        self._publish_calls: list[Dict[str, Any]] = []

    async def create_topic_stream(self, **_kwargs):  # type: ignore[no-untyped-def]
        stream = _FakeStream({
            "id": "stream-1",
            "module": "pilot",
            "topic": "/pilot/debug",
            "message_type": "std_msgs/msg/String",
            "role": "subscribe",
        })
        self._streams.append(stream)
        return stream

    async def call_service(self, **kwargs):  # pragma: no cover - exercised in tests
        self._service_calls.append(dict(kwargs))
        return {"status": "ok"}

    async def publish_topic(self, **kwargs):  # pragma: no cover - exercised in tests
        self._publish_calls.append(dict(kwargs))

    @property
    def service_calls(self) -> list[Dict[str, Any]]:
        return list(self._service_calls)

    @property
    def publish_calls(self) -> list[Dict[str, Any]]:
        return list(self._publish_calls)


def test_register_module_api_actions_registers_stream(tmp_path: Path) -> None:
    payload = """
    {
      "actions": [
        {
          "name": "debug_stream",
          "description": "Stream pilot debug",
          "kind": "stream-topic",
          "defaults": {
            "topic": "/pilot/debug",
            "message_type": "std_msgs/msg/String",
            "role": "subscribe",
            "queue_length": 5
          }
        }
      ]
    }
    """
    write_actions_file(tmp_path, "pilot", payload)
    registry = ActionRegistry()
    ros = _FakeRos()

    register_module_api_actions(registry, modules_root=tmp_path, ros=ros)  # type: ignore[arg-type]

    result = asyncio.run(
        registry.execute(
            "pilot",
            "debug_stream",
            app=None,
            arguments={},
            request=None,
        )
    )
    assert result.streaming is True
    assert result.stream is not None
    metadata = result.stream.metadata.to_dict()
    assert metadata["topic"] == "/pilot/debug"


def test_register_module_api_actions_registers_service(tmp_path: Path) -> None:
    payload = """
    {
      "actions": [
        {
          "name": "debug_snapshot",
          "description": "Request a snapshot via ROS service",
          "kind": "call-service",
          "defaults": {
            "service": "/pilot/debug",
            "service_type": "std_srvs/srv/Trigger",
            "arguments": {"force": true},
            "timeout_ms": 1500
          },
          "parameters": {
            "type": "object",
            "properties": {
              "timeout_ms": {"type": "integer"},
              "arguments": {"type": "object"}
            },
            "additionalProperties": false
          }
        }
      ]
    }
    """
    write_actions_file(tmp_path, "pilot", payload)
    registry = ActionRegistry()
    ros = _FakeRos()

    register_module_api_actions(registry, modules_root=tmp_path, ros=ros)  # type: ignore[arg-type]

    result = asyncio.run(
        registry.execute(
            "pilot",
            "debug_snapshot",
            app=None,
            arguments={"timeout_ms": 1200, "arguments": {"force": False}},
            request=None,
        )
    )

    assert result.streaming is False
    assert result.payload == {"result": {"status": "ok"}}

    assert len(ros.service_calls) == 1
    call = ros.service_calls[0]
    assert call["service_name"] == "/pilot/debug"
    assert call["service_type"] == "std_srvs/srv/Trigger"
    assert call["arguments"] == {"force": False}
    assert call["timeout"] == pytest.approx(1.2)


def test_register_module_api_actions_registers_publish(tmp_path: Path) -> None:
    payload = """
    {
      "actions": [
        {
          "name": "say",
          "description": "Publish speech text to the voice module",
          "kind": "publish-topic",
          "defaults": {
            "topic": "/voice",
            "message_type": "std_msgs/msg/String",
            "payload_map": {"text": "data"}
          },
          "parameters": {
            "type": "object",
            "properties": {
              "text": {
                "type": "string",
                "minLength": 1
              }
            },
            "required": ["text"],
            "additionalProperties": false
          }
        }
      ]
    }
    """
    write_actions_file(tmp_path, "voice", payload)
    registry = ActionRegistry()
    ros = _FakeRos()

    register_module_api_actions(registry, modules_root=tmp_path, ros=ros)  # type: ignore[arg-type]

    result = asyncio.run(
        registry.execute(
            "voice",
            "say",
            app=None,
            arguments={"text": "Hello there"},
            request=None,
        )
    )

    assert result.streaming is False
    assert result.payload == {
        "status": "published",
        "topic": "/voice",
        "message_type": "std_msgs/msg/String",
    }

    assert ros.publish_calls == [
        {
            "module": "voice",
            "topic": "/voice",
            "message_type": "std_msgs/msg/String",
            "payload": {"data": "Hello there"},
            "qos": None,
        }
    ]


def test_register_module_api_actions_uses_extra_roots(tmp_path: Path) -> None:
    payload = """
    {
      "actions": [
        {
          "name": "debug_stream",
          "description": "Stream pilot debug",
          "kind": "stream-topic",
          "defaults": {
            "topic": "/pilot/debug",
            "message_type": "std_msgs/msg/String"
          }
        }
      ]
    }
    """

    work_root = tmp_path / "work"
    work_root.mkdir()
    fallback_root = tmp_path / "repo"
    write_actions_file(fallback_root, "pilot", payload)

    registry = ActionRegistry()
    ros = _FakeRos()

    register_module_api_actions(
        registry,
        modules_root=work_root,
        ros=ros,
        extra_roots=[fallback_root],
    )  # type: ignore[arg-type]

    action = registry.get("pilot", "debug_stream")
    assert action.description == "Stream pilot debug"


def test_register_module_api_actions_uses_manifest_name(tmp_path: Path) -> None:
    payload = """
    {
      "actions": [
        {
          "name": "debug_stream",
          "description": "Stream pilot debug",
          "kind": "stream-topic",
          "defaults": {
            "topic": "/pilot/debug",
            "message_type": "std_msgs/msg/String"
          }
        }
      ]
    }
    """
    module_dir = write_actions_file(tmp_path, "pilot-unit", payload)
    module_dir.joinpath("module.toml").write_text(
        "name = \"pilot\"\n", encoding="utf-8"
    )
    registry = ActionRegistry()
    ros = _FakeRos()

    register_module_api_actions(registry, modules_root=tmp_path, ros=ros)  # type: ignore[arg-type]

    action = registry.get("pilot", "debug_stream")
    assert action.description == "Stream pilot debug"


def test_publish_topic_action_supports_text_parser(tmp_path: Path) -> None:
    payload = """
    {
      "actions": [
        {
          "name": "cmd_vel_text",
          "description": "Parse a textual command into cmd_vel",
          "kind": "publish-topic",
          "defaults": {
            "topic": "cmd_vel",
            "message_type": "geometry_msgs/msg/Twist",
            "payload": {
              "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
              "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "text_parser": "twist/v1",
            "text_argument": "command",
            "payload_map": {
              "command": null,
              "linear_x": "linear.x",
              "angular_z": "angular.z"
            }
          },
          "parameters": {
            "type": "object",
            "properties": {
              "command": {"type": "string"},
              "linear_x": {"type": "number"},
              "angular_z": {"type": "number"}
            },
            "additionalProperties": false
          }
        }
      ]
    }
    """

    write_actions_file(tmp_path, "foot", payload)
    registry = ActionRegistry()
    ros = _FakeRos()

    register_module_api_actions(registry, modules_root=tmp_path, ros=ros)  # type: ignore[arg-type]

    result = asyncio.run(
        registry.execute(
            "foot",
            "cmd_vel_text",
            app=None,
            arguments={"command": "forward 0.4 turn_right 1.2"},
            request=None,
        )
    )

    assert result.streaming is False
    assert ros.publish_calls and ros.publish_calls[0]["topic"] == "cmd_vel"
    payload_sent = ros.publish_calls[0]["payload"]
    assert payload_sent["linear"]["x"] == pytest.approx(0.4)
    assert payload_sent["angular"]["z"] == pytest.approx(-1.2)


def test_publish_topic_action_supports_indexed_payload_map(tmp_path: Path) -> None:
    payload = """
    {
      "actions": [
        {
          "name": "set_power_led",
          "description": "Update the Create power LED",
          "kind": "publish-topic",
          "defaults": {
            "topic": "power_led",
            "message_type": "std_msgs/msg/UInt8MultiArray",
            "payload": {"data": [0, 128]},
            "payload_map": {
              "color": "data[0]",
              "intensity": "data[1]"
            }
          },
          "parameters": {
            "type": "object",
            "properties": {
              "color": {"type": "integer"},
              "intensity": {"type": "integer"}
            },
            "required": ["color", "intensity"],
            "additionalProperties": false
          }
        }
      ]
    }
    """

    write_actions_file(tmp_path, "foot", payload)
    registry = ActionRegistry()
    ros = _FakeRos()

    register_module_api_actions(registry, modules_root=tmp_path, ros=ros)  # type: ignore[arg-type]

    asyncio.run(
        registry.execute(
            "foot",
            "set_power_led",
            app=None,
            arguments={"color": 64, "intensity": 200},
            request=None,
        )
    )

    assert ros.publish_calls
    published = ros.publish_calls[0]["payload"]
    assert published["data"] == [64, 200]
