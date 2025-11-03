"""Tests for the faces cockpit module actions."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, Dict, List

import pytest

from cockpit.actions import ActionContext, ActionError
from modules.faces.cockpit.actions import tag_face_action


class DummyRosClient:
    """Minimal ROS client stub capturing service invocations."""

    def __init__(self, response: Dict[str, Any]) -> None:
        self._response = response
        self.calls: List[Dict[str, Any]] = []

    async def call_service(
        self,
        *,
        service_name: str,
        service_type: str | None,
        arguments: Dict[str, Any],
        timeout: float,
    ) -> Dict[str, Any]:
        self.calls.append(
            {
                "service": service_name,
                "service_type": service_type,
                "arguments": arguments,
                "timeout": timeout,
            }
        )
        return dict(self._response)


def _context(arguments: Dict[str, Any]) -> ActionContext:
    return ActionContext(module="faces", arguments=arguments, app=SimpleNamespace())


def _definition(overrides: Dict[str, Any] | None = None) -> SimpleNamespace:
    defaults = {
        "service": "/memory/tag_identity",
        "service_type": "memory_interfaces/srv/TagIdentity",
        "timeout_ms": 4000,
    }
    if overrides:
        defaults.update(overrides)
    return SimpleNamespace(defaults=defaults)


@pytest.mark.asyncio
async def test_tag_face_action_invokes_ros_service() -> None:
    """Tagging a face forwards the request to the memory tagging service."""

    ros = DummyRosClient({"success": True, "identity_id": "person:alice", "message": ""})
    context = _context({"face_id": "mem-123", "label": "Alice Example"})

    result = await tag_face_action(context=context, ros=ros, definition=_definition())

    assert ros.calls, "ROS client should receive a service call"
    call = ros.calls[0]
    assert call["service"] == "/memory/tag_identity"
    assert call["arguments"]["memory_id"] == "mem-123"
    assert call["arguments"]["label"] == "Alice Example"
    assert "Alice Example" in call["arguments"]["aliases"]
    assert result.payload and result.payload["result"]["identity_id"] == "person:alice"


@pytest.mark.asyncio
async def test_tag_face_action_rejects_missing_face_id() -> None:
    """A missing face identifier should produce a validation error."""

    ros = DummyRosClient({"success": True})
    context = _context({"face_id": " ", "label": "Bob"})

    with pytest.raises(ActionError):
        await tag_face_action(context=context, ros=ros, definition=_definition())
