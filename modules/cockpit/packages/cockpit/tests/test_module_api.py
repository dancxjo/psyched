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

    @property
    def service_calls(self) -> list[Dict[str, Any]]:
        return list(self._service_calls)


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
