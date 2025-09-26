"""API contract tests for the pilot backend."""

from __future__ import annotations

from typing import Any, Dict, Iterable, List

import pytest
from fastapi.testclient import TestClient

from pilot.app import create_app
from pilot.module_catalog import ModuleCatalog
from pilot.topic_manager import TopicSession


class FakeTopicManager:
    """Minimal topic manager used to verify request routing."""

    def __init__(self) -> None:
        self.created: List[Dict[str, Any]] = []
        self.sessions: Dict[str, TopicSession] = {}

    def create_session(
        self,
        *,
        topic: str,
        access: str,
        qos: Dict[str, Any] | None,
        module: str | None,
        message_type: str,
    ) -> TopicSession:
        payload = {"topic": topic, "access": access, "qos": qos, "module": module, "message_type": message_type}
        self.created.append(payload)
        session = TopicSession(
            session_id=f"sess-{len(self.created)}",
            topic=topic,
            access=access,
            message_type=message_type,
            qos=None,
        )
        self.sessions[session.session_id] = session
        return session

    def drop_session(self, session_id: str) -> None:
        self.sessions.pop(session_id, None)

    def list_sessions(self) -> Iterable[TopicSession]:
        return list(self.sessions.values())

    def get_session(self, session_id: str) -> TopicSession | None:
        return self.sessions.get(session_id)

    async def pump_messages(self, session_id: str):  # pragma: no cover - unused in tests
        yield {}

    async def publish(self, session_id: str, data):  # pragma: no cover - unused in tests
        self.created.append({"publish": data})

    def set_paused(self, session_id: str, paused: bool) -> TopicSession:
        session = self.sessions[session_id]
        session.paused = paused
        return session


class FakeCommandExecutor:
    """Stub command executor to observe invocations."""

    def __init__(self) -> None:
        self.calls: List[Dict[str, Any]] = []

    async def run(self, scope: str, module: str, command: str, args: List[str] | None = None) -> Dict[str, Any]:
        record = {
            "scope": scope,
            "module": module,
            "command": command,
            "args": args or [],
        }
        self.calls.append(record)
        return {"status": "ok", **record}


@pytest.fixture()
def test_client(tmp_path):
    """Create a FastAPI test client with in-memory dependencies."""

    repo_root = tmp_path / "repo"
    modules_dir = repo_root / "modules"
    modules_dir.mkdir(parents=True)

    pilot_module = modules_dir / "pilot"
    pilot_module.mkdir()
    (pilot_module / "module.toml").write_text(
        """
name = "pilot"

[pilot]
display_name = "Pilot"
description = "Test module"

[[pilot.topics]]
name = "/cmd_vel"
type = "geometry_msgs/msg/Twist"
access = "rw"

[pilot.topics.qos]
history = "keep_last"
depth = 10

"""
    )

    catalog = ModuleCatalog(modules_dir)
    topic_manager = FakeTopicManager()
    executor = FakeCommandExecutor()

    app = create_app(catalog=catalog, topic_manager=topic_manager, command_executor=executor)
    app.state._test_executor = executor
    app.state._test_topics = topic_manager
    client = TestClient(app)
    return client


def test_get_modules_returns_metadata(test_client):
    response = test_client.get("/api/modules")
    assert response.status_code == 200
    data = response.json()
    modules = data["modules"]
    pilot_entry = next((entry for entry in modules if entry["name"] == "pilot"), None)
    assert pilot_entry is not None
    assert pilot_entry["regimes"], "Pilot module should declare regimes for grouping"


def test_post_command_invokes_executor(test_client):
    executor: FakeCommandExecutor = test_client.app.state._test_executor
    payload = {"scope": "mod", "command": "restart"}
    response = test_client.post("/api/modules/pilot/commands", json=payload)
    assert response.status_code == 202
    assert executor.calls
    call = executor.calls[-1]
    assert call["command"] == "restart"


def test_create_topic_session(test_client):
    topic_manager: FakeTopicManager = test_client.app.state._test_topics
    payload = {"topic": "/cmd_vel", "access": "rw"}
    response = test_client.post("/api/topics", json=payload)
    assert response.status_code == 201
    body = response.json()
    assert body["session"]["access"] == "rw"
    assert topic_manager.created[-1]["topic"] == "/cmd_vel"
