"""API contract tests for the pilot backend."""

from __future__ import annotations

import socket
from typing import Any, Dict, Iterable, List

import pytest
from fastapi.testclient import TestClient
import yaml

from pilot.app import create_app
from pilot.module_catalog import ModuleCatalog
from pilot.topic_manager import TopicSession
from pilot.voice_config import VoiceConfigStore


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
qos = { history = "keep_last", depth = 10 }

[[pilot.topics]]
name = "/host/health"
type = "psyched_msgs/msg/HostHealth"
access = "ro"
presentation = "status"

"""
    )

    catalog = ModuleCatalog(modules_dir)
    topic_manager = FakeTopicManager()
    executor = FakeCommandExecutor()
    voice_config_dir = repo_root / "hosts" / "cerebellum" / "config"
    voice_config_dir.mkdir(parents=True)
    voice_config_path = voice_config_dir / "voice.yaml"
    voice_config_path.write_text(
        """
enable_tts: false
engine: espeak
voice: en-US-TestVoice
topic: /voice
interrupt: /voice/interrupt
resume: /voice/resume
clear: /voice/clear
"""
    )
    voice_store = VoiceConfigStore(voice_config_path)

    app = create_app(
        catalog=catalog,
        topic_manager=topic_manager,
        command_executor=executor,
        voice_config_store=voice_store,
    )
    app.state._test_executor = executor
    app.state._test_topics = topic_manager
    app.state._test_catalog = catalog
    app.state._voice_config_path = voice_config_path
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


def test_create_host_health_topic_uses_structured_message(test_client):
    """Host health subscriptions should resolve to the HostHealth payload."""

    topic_manager: FakeTopicManager = test_client.app.state._test_topics
    catalog: ModuleCatalog = test_client.app.state._test_catalog
    topics = {topic.topic: topic for topic in catalog.get_module("pilot").topics}
    assert topics, "Pilot module topics should be discovered"
    short_host = socket.gethostname().split(".")[0]
    expected_topic = f"/hosts/health/{short_host}"
    assert expected_topic in topics, f"Host health topic missing from catalog: {topics.keys()}"
    payload = {"topic": f"/hosts/health/{short_host}", "access": "ro", "module": "pilot"}

    response = test_client.post("/api/topics", json=payload)

    assert response.status_code == 201, response.text
    body = response.json()
    assert body["session"]["message_type"] == "psyched_msgs/msg/HostHealth"
    assert topic_manager.created[-1]["message_type"] == "psyched_msgs/msg/HostHealth"


def test_get_voice_config_returns_yaml_payload(test_client):
    response = test_client.get("/api/voice/config")
    assert response.status_code == 200
    data = response.json()
    assert data["config"]["engine"] == "espeak"
    assert data["config"]["voice"] == "en-US-TestVoice"


def test_put_voice_config_persists_updates(test_client):
    payload = {"engine": "ms-edge", "enable_tts": True, "voice": "en-US-AnaNeural"}
    response = test_client.put("/api/voice/config", json=payload)
    assert response.status_code == 200
    body = response.json()
    assert body["config"]["engine"] == "ms-edge"
    voice_path = test_client.app.state._voice_config_path
    file_data = yaml.safe_load(voice_path.read_text())
    assert file_data["engine"] == "ms-edge"
    assert file_data["voice"] == "en-US-AnaNeural"
    assert file_data["enable_tts"] is True


def test_put_voice_config_rejects_empty_updates(test_client):
    response = test_client.put("/api/voice/config", json={})
    assert response.status_code == 400
    assert response.json()["detail"].lower().startswith("no updates")
