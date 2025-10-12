"""Integration tests for the HTTP API surface exposed by the cockpit server."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
import importlib.util
import sys
from typing import Any, Dict, Iterable, List

import pytest

pytest.importorskip("aiohttp")
from aiohttp import ClientSession, web

REPO_ROOT = Path(__file__).resolve().parents[5]
PACKAGE_DIR = REPO_ROOT / "modules" / "pilot" / "packages" / "pilot" / "pilot"
MODULES_ROOT = REPO_ROOT / "modules"

spec = importlib.util.spec_from_file_location(
    "pilot",
    PACKAGE_DIR / "__init__.py",
    submodule_search_locations=[str(PACKAGE_DIR)],
)
module = importlib.util.module_from_spec(spec)
sys.modules["pilot"] = module
spec.loader.exec_module(module)

from pilot.server import PilotSettings, create_app


class StubRosBridge:
    """ROS bridge stub that captures subscription requests for inspection."""

    def __init__(self) -> None:
        self.subscriptions: List[Dict[str, Any]] = []

    async def handle_websocket(self, request: web.Request) -> web.StreamResponse:
        self.subscriptions.append(dict(request.query))
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        await ws.close()
        return ws


@pytest.fixture()
def config_file(tmp_path: Path) -> Path:
    payload = {
        "host": {"name": "api-test"},
        "modules": {
            "imu": {"launch": True},
            "pilot": {"launch": True},
        },
    }
    path = tmp_path / "api-test.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_modules_endpoint_returns_active_modules(config_file: Path, tmp_path: Path) -> None:
    asyncio.run(_exercise_modules_endpoint(config_file, tmp_path))


def test_websocket_route_delegates_to_bridge(config_file: Path, tmp_path: Path) -> None:
    asyncio.run(_exercise_websocket_endpoint(config_file, tmp_path))


def test_legacy_ws_route_delegates_to_bridge(config_file: Path, tmp_path: Path) -> None:
    asyncio.run(_exercise_legacy_websocket_endpoint(config_file, tmp_path))


def test_command_endpoint_executes_allowed_command(config_file: Path, tmp_path: Path) -> None:
    asyncio.run(_exercise_command_endpoint_success(config_file, tmp_path))


def test_command_endpoint_rejects_unknown_command(config_file: Path, tmp_path: Path) -> None:
    asyncio.run(_exercise_command_endpoint_rejects(config_file, tmp_path))


def test_static_handler_serves_module_pilot_assets(config_file: Path) -> None:
    asyncio.run(_exercise_static_overlay(config_file))


async def _exercise_modules_endpoint(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    bridge = StubRosBridge()
    app = create_app(settings=settings, ros_bridge=bridge)

    async with _run_app(app) as client:
        response = await client.get("/api/modules")
        assert response.status == 200
        payload = await response.json()

    modules = payload["modules"]
    module_names = {module["name"] for module in modules}
    assert module_names == {"imu", "pilot"}

    for module in modules:
        assert module.get("slug"), "modules should include a slug field"
        assert isinstance(module.get("commands", {}), dict)
        if module.get("has_pilot"):
            assert module.get("dashboard_url"), "Modules with pilot assets should expose a dashboard URL"


async def _exercise_websocket_endpoint(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    bridge = StubRosBridge()
    app = create_app(settings=settings, ros_bridge=bridge)

    async with _run_app(app) as client:
        await client.ws_connect("/api/topics/bridge?topic=/imu/data&type=sensor_msgs/msg/Imu&role=subscribe")

    assert bridge.subscriptions == [
        {
            "topic": "/imu/data",
            "type": "sensor_msgs/msg/Imu",
            "role": "subscribe",
        }
    ]


async def _exercise_legacy_websocket_endpoint(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    bridge = StubRosBridge()
    app = create_app(settings=settings, ros_bridge=bridge)

    async with _run_app(app) as client:
        await client.ws_connect("/ws?topic=/imu/data&type=sensor_msgs/msg/Imu&role=subscribe")

    assert bridge.subscriptions == [
        {
            "topic": "/imu/data",
            "type": "sensor_msgs/msg/Imu",
            "role": "subscribe",
        }
    ]


async def _exercise_command_endpoint_success(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    bridge = StubRosBridge()

    class StubExecutor:
        def __init__(self) -> None:
            self.calls: list[tuple[str, str, str, tuple[str, ...]]] = []

        async def run(self, scope: str, module: str, command: str, args: Iterable[str]) -> Dict[str, Any]:
            call = (scope, module, command, tuple(args))
            self.calls.append(call)
            return {
                "code": 0,
                "stdout": "ok\n",
                "stderr": "",
                "stdout_plain": "ok\n",
                "stderr_plain": "",
            }

    executor = StubExecutor()
    app = create_app(settings=settings, ros_bridge=bridge)
    app["command_executor"] = executor

    async with _run_app(app) as client:
        response = await client.post("/api/modules/imu/commands", json={"scope": "mod", "command": "setup"})
        assert response.status == 200
        payload = await response.json()

    assert executor.calls == [("mod", "imu", "setup", ())]
    assert payload["result"]["code"] == 0


async def _exercise_command_endpoint_rejects(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    bridge = StubRosBridge()

    class StubExecutor:
        def __init__(self) -> None:
            self.calls: list[Any] = []

        async def run(self, scope: str, module: str, command: str, args: Iterable[str]) -> Dict[str, Any]:
            self.calls.append((scope, module, command, tuple(args)))
            return {}

    executor = StubExecutor()
    app = create_app(settings=settings, ros_bridge=bridge)
    app["command_executor"] = executor

    async with _run_app(app) as client:
        response = await client.post("/api/modules/imu/commands", json={"scope": "mod", "command": "reboot"})
        assert response.status == 400
        error_text = await response.text()

    assert executor.calls == []
    assert "Unsupported command" in error_text


async def _exercise_static_overlay(config_file: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=REPO_ROOT / "modules" / "pilot" / "frontend",
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    bridge = StubRosBridge()
    app = create_app(settings=settings, ros_bridge=bridge)

    async with _run_app(app) as client:
        response = await client.get("/modules/foot/js/foot.js")
        assert response.status == 200
        script_body = await response.text()
        assert "FOOT_TOPICS" in script_body

        index_response = await client.get("/modules/foot/")
        assert index_response.status == 200
        index_html = await index_response.text()
        assert "Create Base Dashboard" in index_html

        missing_response = await client.get("/modules/unknown/page.html")
        assert missing_response.status == 404


class _TestClientContext:
    def __init__(self, app: web.Application):
        self._app = app
        self._runner: web.AppRunner | None = None
        self._site: web.TCPSite | None = None
        self._session: ClientSession | None = None
        self._address: str | None = None

    async def __aenter__(self) -> "_TestClientContext":
        self._runner = web.AppRunner(self._app)
        await self._runner.setup()
        self._site = web.TCPSite(self._runner, "127.0.0.1", 0)
        await self._site.start()
        port = self._site._server.sockets[0].getsockname()[1]
        self._address = f"http://127.0.0.1:{port}"
        self._session = ClientSession()
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        if self._session:
            await self._session.close()
        if self._site:
            await self._site.stop()
        if self._runner:
            await self._runner.cleanup()

    async def get(self, path: str) -> web.ClientResponse:
        assert self._session and self._address
        return await self._session.get(self._address + path)

    async def post(self, path: str, json: Any) -> web.ClientResponse:
        assert self._session and self._address
        return await self._session.post(self._address + path, json=json)

    async def ws_connect(self, path: str) -> web.ClientWebSocketResponse:
        assert self._session and self._address
        return await self._session.ws_connect(self._address.replace("http", "ws") + path)


def _run_app(app: web.Application) -> _TestClientContext:
    return _TestClientContext(app)
