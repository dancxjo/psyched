"""Regression tests for the streamlined pilot HTTP API."""

from __future__ import annotations

import asyncio
import asyncio
import importlib.util
from pathlib import Path
import sys
from typing import Iterator

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


@pytest.fixture()
def config_file(tmp_path: Path) -> Iterator[Path]:
    text = """
[host]
name = "api-test"
modules = ["imu", "pilot", "foot"]

[config.mod.imu.launch]
enabled = true

[config.mod.pilot.launch]
enabled = true

[config.mod.foot.launch]
enabled = true
"""
    path = tmp_path / "api-test.toml"
    path.write_text(text.strip() + "\n", encoding="utf-8")
    yield path


@pytest.fixture()
def config_with_settings(tmp_path: Path) -> Iterator[Path]:
    text = """
[host]
name = "config-test"
modules = ["imu", "pilot", "foot"]

[config.mod.imu.launch]
enabled = true

[config.mod.imu.env]
RANGE = 12

[config.mod.foot.launch]
enabled = false

[config.mod.foot.env]
BALANCE_MODE = "conservative"

[config.mod.pilot.launch]
enabled = true
"""
    path = tmp_path / "config-test.toml"
    path.write_text(text.strip() + "\n", encoding="utf-8")
    yield path


def test_modules_endpoint_reports_active_dashboards(config_file: Path, tmp_path: Path) -> None:
    asyncio.run(_exercise_modules_endpoint(config_file, tmp_path))


def test_static_overlay_serves_module_assets(config_file: Path) -> None:
    asyncio.run(_exercise_static_overlay(config_file))


def test_module_config_endpoint_reports_and_updates_settings(
    config_with_settings: Path, tmp_path: Path
) -> None:
    asyncio.run(_exercise_module_config_endpoint(config_with_settings, tmp_path))


def test_module_config_endpoint_rejects_null_values(
    config_with_settings: Path, tmp_path: Path
) -> None:
    asyncio.run(_exercise_module_config_validation(config_with_settings, tmp_path))


async def _exercise_modules_endpoint(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
        video_base="http://127.0.0.1:8089",
        video_port=8089,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.get("/api/modules")
        assert response.status == 200
        payload = await response.json()

    modules = payload["modules"]
    module_names = {module["name"] for module in modules}
    assert module_names == {"imu", "pilot", "foot"}

    foot_entry = next(module for module in modules if module["name"] == "foot")
    assert foot_entry["has_pilot"] is True
    assert foot_entry["dashboard_url"].endswith("/modules/foot/")

    imu_entry = next(module for module in modules if module["name"] == "imu")
    assert imu_entry["has_pilot"] is True

    for module in modules:
        assert module.get("slug"), "modules should expose a slug"
        assert "topics" not in module, "topics metadata should be omitted"
        if module.get("has_pilot"):
            assert module.get("dashboard_url")

    bridge = payload.get("bridge")
    assert bridge == {
        "mode": settings.bridge_mode,
        "rosbridge_uri": settings.rosbridge_uri,
        "video_base": settings.video_base,
        "video_port": settings.video_port,
    }


async def _exercise_static_overlay(config_file: Path) -> None:
    frontend_root = REPO_ROOT / "modules" / "pilot" / "packages" / "pilot" / "pilot" / "frontend"
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=frontend_root,
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.get("/modules/foot/js/foot.js")
        assert response.status == 200
        body = await response.text()
        assert "Foot" in body

        index_response = await client.get("/modules/foot/")
        assert index_response.status == 200
        html = await index_response.text()
        assert "Create Base Control" in html

        missing = await client.get("/modules/unknown/widget.html")
        assert missing.status == 404


async def _exercise_module_config_endpoint(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.get("/api/module-config")
        assert response.status == 200
        payload = await response.json()

        modules = {module["name"]: module for module in payload["modules"]}
        assert set(modules) == {"imu", "pilot", "foot"}

        imu_entry = modules["imu"]
        assert imu_entry["listed"] is True
        assert imu_entry["active"] is True
        assert imu_entry["config"]["env"]["RANGE"] == 12

        update_payload = {
            "env": {"RANGE": 42, "OFFSET": 3},
            "launch": {"enabled": False},
        }
        update_response = await client.put(
            "/api/module-config/imu",
            json=update_payload,
        )
        assert update_response.status == 200
        updated = await update_response.json()
        assert updated["config"]["env"] == {"RANGE": 42, "OFFSET": 3}
        assert updated["config"]["launch"]["enabled"] is False

        follow_up = await client.get("/api/module-config")
        assert follow_up.status == 200
        follow_payload = await follow_up.json()
        refreshed = {
            module["name"]: module for module in follow_payload["modules"]
        }
        assert refreshed["imu"]["config"]["env"]["OFFSET"] == 3

    text = config_file.read_text(encoding="utf-8")
    assert "OFFSET" in text
    assert "RANGE = 42" in text


async def _exercise_module_config_validation(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=REPO_ROOT,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.put(
            "/api/module-config/foot",
            json={"env": {"BALANCE_MODE": None}},
        )
        assert response.status == 400

        bad_json = await client.request(
            "PUT",
            "/api/module-config/foot",
            data="not-json",
            headers={"Content-Type": "application/json"},
        )
        assert bad_json.status == 400


class _TestClient:
    def __init__(self, app: web.Application):
        self._app = app
        self._runner: web.AppRunner | None = None
        self._site: web.TCPSite | None = None
        self._session: ClientSession | None = None
        self._address: str | None = None

    async def __aenter__(self) -> "_TestClient":
        self._runner = web.AppRunner(self._app)
        await self._runner.setup()
        self._site = web.TCPSite(self._runner, "127.0.0.1", 0)
        await self._site.start()
        assert self._site._server is not None  # type: ignore[union-attr]
        port = self._site._server.sockets[0].getsockname()[1]  # type: ignore[attr-defined]
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

    async def request(self, method: str, path: str, **kwargs) -> web.ClientResponse:
        assert self._session and self._address
        return await self._session.request(method, self._address + path, **kwargs)

    async def get(self, path: str) -> web.ClientResponse:
        return await self.request("GET", path)

    async def put(self, path: str, **kwargs) -> web.ClientResponse:
        return await self.request("PUT", path, **kwargs)


def _run_app(app: web.Application) -> _TestClient:
    return _TestClient(app)
