"""Regression tests for the streamlined pilot HTTP API."""

from __future__ import annotations

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

from pilot.server import CommandResult, PilotSettings, create_app, MODULE_LOG_LINE_LIMIT


@pytest.fixture()
def config_file(tmp_path: Path) -> Iterator[Path]:
    text = """
[host]
name = "api-test"
modules = ["imu", "pilot", "foot", "viscera"]

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
modules = ["imu", "pilot", "foot", "viscera"]

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


def test_modules_endpoint_reports_active_dashboards(
    monkeypatch: pytest.MonkeyPatch,
    config_file: Path,
    tmp_path: Path,
) -> None:
    async def fake_status(module_name: str) -> dict[str, object]:
        return {
            "supported": True,
            "unit": f"psh-module-{module_name}.service",
            "load_state": "loaded",
            "active_state": "inactive",
            "sub_state": "dead",
            "unit_file_state": "disabled",
            "exists": True,
            "enabled": False,
            "active": False,
        }

    monkeypatch.setattr("pilot.server._module_systemd_status", fake_status)
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


def test_module_logs_endpoint_returns_recent_lines(tmp_path: Path) -> None:
    asyncio.run(_exercise_module_logs_endpoint(tmp_path))


def test_module_logs_endpoint_handles_missing_file(tmp_path: Path) -> None:
    asyncio.run(_exercise_module_logs_missing_file(tmp_path))


def test_module_logs_endpoint_rejects_invalid_name(tmp_path: Path) -> None:
    asyncio.run(_exercise_module_logs_invalid_name(tmp_path))


def test_module_logs_endpoint_clears_file(tmp_path: Path) -> None:
    asyncio.run(_exercise_module_logs_clear(tmp_path))


def test_git_pull_endpoint_invokes_repo_update(
    monkeypatch: pytest.MonkeyPatch, config_file: Path, tmp_path: Path
) -> None:
    calls: list[tuple[tuple[str, ...], Path]] = []

    async def fake_run_command(*, command: list[str], cwd: Path) -> CommandResult:
        calls.append((tuple(command), cwd))
        return CommandResult(command=list(command), returncode=0, stdout="Already up to date.\n", stderr="")

    monkeypatch.setattr("pilot.server._run_command", fake_run_command)
    asyncio.run(_exercise_git_pull_endpoint(config_file, tmp_path, calls))


def test_psh_endpoint_runs_permitted_operation(
    monkeypatch: pytest.MonkeyPatch, config_file: Path, tmp_path: Path
) -> None:
    calls: list[tuple[tuple[str, ...], Path]] = []

    async def fake_run_command(*, command: list[str], cwd: Path) -> CommandResult:
        calls.append((tuple(command), cwd))
        return CommandResult(command=list(command), returncode=0, stdout="modules launched", stderr="")

    monkeypatch.setattr("pilot.server._run_command", fake_run_command)
    asyncio.run(_exercise_psh_operation_endpoint(config_file, tmp_path, calls))


def test_psh_endpoint_rejects_unknown_operation(
    config_file: Path, tmp_path: Path
) -> None:
    asyncio.run(_exercise_psh_rejection(config_file, tmp_path))


def test_systemd_endpoint_runs_permitted_action(
    monkeypatch: pytest.MonkeyPatch,
    config_file: Path,
    tmp_path: Path,
) -> None:
    calls: list[tuple[tuple[str, ...], Path]] = []

    async def fake_run_command(*, command: list[str], cwd: Path) -> CommandResult:
        calls.append((tuple(command), cwd))
        return CommandResult(command=list(command), returncode=0, stdout="started", stderr="")

    async def fake_status(module_name: str) -> dict[str, object]:
        return {
            "supported": True,
            "unit": f"psh-module-{module_name}.service",
            "load_state": "loaded",
            "active_state": "active",
            "sub_state": "running",
            "unit_file_state": "enabled",
            "exists": True,
            "enabled": True,
            "active": True,
        }

    monkeypatch.setattr("pilot.server._run_command", fake_run_command)
    monkeypatch.setattr("pilot.server._module_systemd_status", fake_status)
    asyncio.run(_exercise_systemd_operation_endpoint(config_file, tmp_path, calls))


def test_systemd_endpoint_rejects_unknown_action(
    config_file: Path,
    tmp_path: Path,
) -> None:
    asyncio.run(_exercise_systemd_rejection(config_file, tmp_path))


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
    assert module_names == {"imu", "pilot", "foot", "viscera"}

    foot_entry = next(module for module in modules if module["name"] == "foot")
    assert foot_entry["has_pilot"] is True
    assert foot_entry["dashboard_url"].endswith("/modules/foot/")

    imu_entry = next(module for module in modules if module["name"] == "imu")
    assert imu_entry["has_pilot"] is True

    pilot_entry = next(module for module in modules if module["name"] == "pilot")
    assert pilot_entry["has_pilot"] is True
    assert pilot_entry["dashboard_url"].endswith("/modules/pilot/")

    viscera_entry = next(module for module in modules if module["name"] == "viscera")
    assert viscera_entry["has_pilot"] is True
    assert viscera_entry["dashboard_url"].endswith("/modules/viscera/")

    for module in modules:
        assert module.get("slug"), "modules should expose a slug"
        assert "topics" not in module, "topics metadata should be omitted"
        if module.get("has_pilot"):
            assert module.get("dashboard_url")
        systemd = module.get("systemd")
        assert isinstance(systemd, dict)
        assert set(systemd).issuperset({"unit", "supported", "active", "enabled", "exists"})

    bridge = payload.get("bridge")
    assert bridge == {
        "mode": settings.bridge_mode,
        "rosbridge_uri": settings.rosbridge_uri,
        "video_base": settings.video_base,
        "video_port": settings.video_port,
    }

    host_meta = payload.get("host")
    assert host_meta == {
        "name": "api-test",
        "shortname": "api-test",
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

        viscera_dashboard = await client.get("/modules/viscera/components/viscera-dashboard.js")
        assert viscera_dashboard.status == 200

        viscera_index = await client.get("/modules/viscera/")
        assert viscera_index.status == 200

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
        assert set(modules) == {"imu", "pilot", "foot", "viscera"}

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


async def _exercise_module_logs_endpoint(tmp_path: Path) -> None:
    module_name = "nav"
    total_lines = MODULE_LOG_LINE_LIMIT + 4
    lines = [f"log line {index}" for index in range(1, total_lines + 1)]
    settings = _build_minimal_settings(tmp_path, module_name=module_name, log_lines=lines)
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.get(f"/api/modules/{module_name}/logs")
        assert response.status == 200
        payload = await response.json()

    assert payload["module"] == module_name
    assert payload["display_name"] == "Nav"
    assert payload["truncated"] is True
    assert len(payload["lines"]) == MODULE_LOG_LINE_LIMIT
    # Lines are now returned newest first
    assert payload["lines"][0] == lines[-1]
    assert payload["lines"][-1] == lines[-MODULE_LOG_LINE_LIMIT]
    assert payload["updated_at"] is not None


async def _exercise_module_logs_missing_file(tmp_path: Path) -> None:
    module_name = "imu"
    settings = _build_minimal_settings(tmp_path, module_name=module_name, log_lines=None)
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.get(f"/api/modules/{module_name}/logs")
        assert response.status == 200
        payload = await response.json()

    assert payload["module"] == module_name
    assert payload["lines"] == []
    assert payload["truncated"] is False
    assert payload["updated_at"] is None


async def _exercise_module_logs_invalid_name(tmp_path: Path) -> None:
    module_name = "nav"
    settings = _build_minimal_settings(tmp_path, module_name=module_name, log_lines=None)
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.get("/api/modules/nav!/logs")
        assert response.status == 400


async def _exercise_module_logs_clear(tmp_path: Path) -> None:
    module_name = "nav"
    settings = _build_minimal_settings(
        tmp_path,
        module_name=module_name,
        log_lines=["first line", "second line"],
    )
    log_path = tmp_path / "log" / "modules" / f"{module_name}.log"
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.delete(f"/api/modules/{module_name}/logs")
        assert response.status == 200
        payload = await response.json()
        assert payload == {"module": module_name, "cleared": True}
        assert log_path.read_text(encoding="utf-8") == ""

        follow_up = await client.get(f"/api/modules/{module_name}/logs")
        assert follow_up.status == 200
        log_payload = await follow_up.json()
        assert log_payload["lines"] == []
        assert log_payload["truncated"] is False

        # Clearing again should succeed even when the file is missing on disk.
        log_path.unlink()
        second = await client.delete(f"/api/modules/{module_name}/logs")
        assert second.status == 200
        assert log_path.exists()


async def _exercise_git_pull_endpoint(
    config_file: Path, tmp_path: Path, calls: list[tuple[tuple[str, ...], Path]]
) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=tmp_path,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.post("/api/ops/git-pull")
        assert response.status == 200
        payload = await response.json()

    assert payload["success"] is True
    assert "Already up to date" in payload["stdout"]
    assert calls == [(("git", "pull", "--ff-only"), tmp_path)]


async def _exercise_psh_operation_endpoint(
    config_file: Path, tmp_path: Path, calls: list[tuple[tuple[str, ...], Path]]
) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=tmp_path,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.post(
            "/api/ops/psh",
            json={"operation": "module-up"},
        )
        assert response.status == 200
        payload = await response.json()

    assert payload["success"] is True
    assert payload["stdout"] == "modules launched"
    assert calls == [(("psh", "mod", "up"), tmp_path)]


async def _exercise_psh_rejection(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=tmp_path,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.post(
            "/api/ops/psh",
            json={"operation": "unknown"},
        )
        assert response.status == 400


async def _exercise_systemd_operation_endpoint(
    config_file: Path,
    tmp_path: Path,
    calls: list[tuple[tuple[str, ...], Path]],
) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=tmp_path,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.post("/api/modules/imu/systemd/up")
        assert response.status == 200
        payload = await response.json()

    assert payload["success"] is True
    assert payload["module"] == "imu"
    assert payload["action"] == "up"
    assert payload["status"]["active"] is True
    assert calls == [( ("psh", "sys", "up", "--module", "imu"), tmp_path)]


async def _exercise_systemd_rejection(config_file: Path, tmp_path: Path) -> None:
    settings = PilotSettings(
        host_config_path=config_file,
        frontend_root=tmp_path,
        modules_root=MODULES_ROOT,
        repo_root=tmp_path,
        listen_host="127.0.0.1",
        listen_port=0,
    )
    app = create_app(settings=settings)

    async with _run_app(app) as client:
        response = await client.post("/api/modules/imu/systemd/unknown")
        assert response.status == 400


def _build_minimal_settings(
    tmp_path: Path,
    *,
    module_name: str,
    log_lines: list[str] | None,
) -> PilotSettings:
    modules_root = tmp_path / "modules"
    modules_root.mkdir(parents=True, exist_ok=True)
    module_dir = modules_root / module_name
    module_dir.mkdir(parents=True, exist_ok=True)

    manifest = module_dir / "module.toml"
    manifest.write_text(
        (
            f"name = \"{module_name}\"\n"
            "\n"
            "[pilot]\n"
            f"display_name = \"{module_name.title()}\"\n"
            "description = \"Test module\"\n"
        ),
        encoding="utf-8",
    )

    config_text = (
        "[host]\n"
        "name = \"logs-test\"\n"
        f"modules = [\"{module_name}\"]\n"
        "\n"
        f"[config.mod.{module_name}.launch]\n"
        "enabled = true\n"
    )
    config_path = tmp_path / "logs-test.toml"
    config_path.write_text(config_text, encoding="utf-8")

    if log_lines is not None:
        log_dir = tmp_path / "log" / "modules"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"{module_name}.log"
        log_path.write_text("\n".join(log_lines) + "\n", encoding="utf-8")

    return PilotSettings(
        host_config_path=config_path,
        frontend_root=tmp_path,
        modules_root=modules_root,
        repo_root=tmp_path,
        listen_host="127.0.0.1",
        listen_port=0,
    )


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

    async def post(self, path: str, **kwargs) -> web.ClientResponse:
        return await self.request("POST", path, **kwargs)

    async def delete(self, path: str, **kwargs) -> web.ClientResponse:
        return await self.request("DELETE", path, **kwargs)


def _run_app(app: web.Application) -> _TestClient:
    return _TestClient(app)
