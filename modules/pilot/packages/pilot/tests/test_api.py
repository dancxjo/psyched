"""Integration tests for the HTTP API surface exposed by the cockpit server."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any, Dict, List

import pytest
from aiohttp import ClientSession, web

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
        "host": {"name": "api-test", "modules": ["pilot"]},
        "modules": {
            "imu": {"launch": True},
        },
    }
    path = tmp_path / "api-test.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_modules_endpoint_returns_active_modules(config_file: Path, tmp_path: Path) -> None:
    asyncio.run(_exercise_modules_endpoint(config_file, tmp_path))


def test_websocket_route_delegates_to_bridge(config_file: Path, tmp_path: Path) -> None:
    asyncio.run(_exercise_websocket_endpoint(config_file, tmp_path))


async def _exercise_modules_endpoint(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
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


async def _exercise_websocket_endpoint(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
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

    async def ws_connect(self, path: str) -> web.ClientWebSocketResponse:
        assert self._session and self._address
        return await self._session.ws_connect(self._address.replace("http", "ws") + path)


def _run_app(app: web.Application) -> _TestClientContext:
    return _TestClientContext(app)
