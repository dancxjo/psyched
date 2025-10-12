"""Regression tests for the streamlined pilot HTTP API."""

from __future__ import annotations

import asyncio
import importlib.util
import json
import os
from pathlib import Path
import sys
import textwrap
from typing import Any, Iterator

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


def test_modules_endpoint_reports_active_dashboards(config_file: Path, tmp_path: Path) -> None:
    asyncio.run(_exercise_modules_endpoint(config_file, tmp_path))


def test_static_overlay_serves_module_assets(config_file: Path) -> None:
    asyncio.run(_exercise_static_overlay(config_file))


def test_operations_endpoint_lists_available_commands(
    config_file: Path, tmp_path: Path
) -> None:
    asyncio.run(_exercise_operations_catalog(config_file, tmp_path))


def test_operations_endpoint_executes_known_commands(
    config_file: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    asyncio.run(
        _exercise_operation_execution(config_file, tmp_path, monkeypatch)
    )


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

    async def get(self, path: str) -> web.ClientResponse:
        assert self._session and self._address
        return await self._session.get(self._address + path)

    async def post(self, path: str, *, json: Any) -> web.ClientResponse:
        assert self._session and self._address
        return await self._session.post(self._address + path, json=json)


def _run_app(app: web.Application) -> _TestClient:
    return _TestClient(app)


async def _exercise_operations_catalog(
    config_file: Path, tmp_path: Path
) -> None:
    repo_root = tmp_path / "repo"
    frontend_root = tmp_path / "frontend"
    modules_root = repo_root / "modules"
    frontend_root.mkdir(parents=True, exist_ok=True)
    modules_root.mkdir(parents=True, exist_ok=True)

    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=frontend_root,
        modules_root=modules_root,
        repo_root=repo_root,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.get("/api/operations")
        assert response.status == 200
        payload = await response.json()

    operations = {entry["id"]: entry for entry in payload["operations"]}
    assert "gut-pull" in operations
    assert "psh-build" in operations
    gut = operations["gut-pull"]
    assert gut["command"] == ["gut", "pull"]
    assert operations["psh-build"]["command"] == ["psh", "build"]


async def _exercise_operation_execution(
    config_file: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    frontend_root = tmp_path / "frontend"
    modules_root = repo_root / "modules"
    frontend_root.mkdir(parents=True, exist_ok=True)
    modules_root.mkdir(parents=True, exist_ok=True)

    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log_file = repo_root / "operation-log.jsonl"

    _write_stub_command(bin_dir / "gut", log_file)
    _write_stub_command(bin_dir / "psh", log_file)

    original_path = os.environ.get("PATH", "")
    monkeypatch.setenv("PATH", f"{bin_dir}:{original_path}")

    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=frontend_root,
        modules_root=modules_root,
        repo_root=repo_root,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        gut_response = await client.post(
            "/api/operations", json={"operation": "gut-pull"}
        )
        assert gut_response.status == 200
        gut_payload = await gut_response.json()
        assert gut_payload["operation"] == "gut-pull"
        assert gut_payload["status"] == "ok", gut_payload
        assert gut_payload["exit_code"] == 0

        build_response = await client.post(
            "/api/operations", json={"operation": "psh-build"}
        )
        assert build_response.status == 200
        build_payload = await build_response.json()
        assert build_payload["operation"] == "psh-build"
        assert build_payload["status"] == "ok", build_payload
        assert build_payload["exit_code"] == 0

    entries = [
        json.loads(line)
        for line in log_file.read_text(encoding="utf-8").splitlines()
    ]
    assert any(entry["argv"][0].endswith("gut") for entry in entries)
    assert any(entry["argv"][0].endswith("psh") for entry in entries)
    for entry in entries:
        assert entry["cwd"] == str(repo_root)


def _write_stub_command(path: Path, log_file: Path) -> None:
    log_literal = json.dumps(str(log_file))
    script = textwrap.dedent(
        f"""
        #!/usr/bin/env python3
        import json
        import os
        import sys

        entry = {{"argv": sys.argv, "cwd": os.getcwd()}}
        with open({log_literal}, "a", encoding="utf-8") as fh:
            json.dump(entry, fh)
            fh.write("\\n")
        """
    ).strip()
    path.write_text(script + "\n", encoding="utf-8")
    path.chmod(0o755)
