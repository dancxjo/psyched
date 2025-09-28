"""FastAPI application for the pilot control surface."""

from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from fastapi import APIRouter, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, ConfigDict

from .module_catalog import ModuleCatalog, ModuleInfo, ModuleTopic
from .topic_manager import TopicSessionManager, TopicSession
from .voice_config import VoiceConfigStore
from pathlib import Path
import tomllib


class CommandRequest(BaseModel):
    """Request payload for executing module/system commands."""

    scope: str = Field(pattern="^(mod|sys)$", description="Command scope: mod or sys")
    command: str
    args: List[str] | None = Field(default=None, description="Additional command arguments")


class TopicRequest(BaseModel):
    """Request payload for creating topic sessions."""

    topic: str
    access: str = Field(default="ro", pattern="^(ro|wo|rw)$")
    module: str | None = None
    qos: Dict[str, Any] | None = None
    message_type: str | None = None


class PauseRequest(BaseModel):
    """Request payload for toggling topic flow."""

    paused: bool = True


class VoiceConfigUpdate(BaseModel):
    """Payload for updating the voice module configuration."""

    model_config = ConfigDict(extra="allow")

    enable_tts: bool | None = None
    engine: str | None = None
    voice: str | None = None
    voices_dir: str | None = None
    model: str | None = None
    model_url: str | None = None
    config_url: str | None = None
    topic: str | None = None
    interrupt: str | None = None
    resume: str | None = None
    clear: str | None = None


class CommandExecutor:
    """Executes `psh` commands asynchronously."""

    def __init__(self, repo_root: Path) -> None:
        self._repo_root = repo_root

    async def run(self, scope: str, module: str, command: str, args: List[str] | None = None) -> Dict[str, Any]:
        args = args or []
        psh_path = self._repo_root / "psh" / "main.ts"
        if not psh_path.exists():  # pragma: no cover - should exist in deployed repo
            raise RuntimeError("psh toolchain not available")
        deno = "deno"
        scope_normalized = scope.lower()
        # Align invocation order with the CLI semantics implemented in psh/cli.ts.
        # - `psh mod` expects `psh mod <action> <modules...>`
        # - `psh sys` expects `psh sys <action> <units...>`
        if scope_normalized == "mod":
            command_args = [
                "run",
                "-A",
                str(psh_path),
                "mod",
                command,
                *( [module] if module else [] ),
                *args,
            ]
        elif scope_normalized == "sys":
            command_args = [
                "run",
                "-A",
                str(psh_path),
                "sys",
                command,
                *( [module] if module else [] ),
                *args,
            ]
        else:
            command_args = [
                "run",
                "-A",
                str(psh_path),
                scope,
                *( [module] if module else [] ),
                command,
                *args,
            ]
        process = await asyncio.create_subprocess_exec(
            deno,
            *command_args,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await process.communicate()
        return {
            "code": process.returncode,
            "stdout": stdout.decode(),
            "stderr": stderr.decode(),
        }


class PilotApplication:
    """Encapsulates the FastAPI router and backend dependencies."""

    def __init__(
        self,
        *,
        catalog: ModuleCatalog,
        topic_manager: TopicSessionManager,
        command_executor: CommandExecutor,
        static_root: Path | None = None,
        voice_config_store: VoiceConfigStore | None = None,
    ) -> None:
        self.catalog = catalog
        self.topic_manager = topic_manager
        self.command_executor = command_executor
        self.static_root = static_root
        self.voice_config_store = voice_config_store
        self.app = FastAPI(title="Psyched Pilot", version="2.0.0", lifespan=self._lifespan)
        self._configure_routes()

    @asynccontextmanager
    async def _lifespan(self, app: FastAPI):  # noqa: D401
        """FastAPI lifespan handler that tears down topic sessions on shutdown."""

        try:
            yield
        finally:  # pragma: no cover - exercised during application shutdown
            self.topic_manager.shutdown()

    # ------------------------------------------------------------------
    # Route configuration
    # ------------------------------------------------------------------

    def _configure_routes(self) -> None:
        router = APIRouter(prefix="/api")

        def _find_repo_root_with_hosts(start: Path | None = None) -> Path | None:
            """Walk upwards from `start` to find a parent directory containing a `hosts` directory.

            Returns the repository root Path or None if not found.
            """
            cur = Path(start or Path(__file__).resolve())
            for _ in range(12):
                candidate = cur
                hosts_dir = candidate / "hosts"
                if hosts_dir.exists() and hosts_dir.is_dir():
                    return candidate
                if cur.parent == cur:
                    break
                cur = cur.parent
            return None

        def _enabled_modules_for_host() -> list[str] | None:
            """Return a set of module names enabled for this host from hosts/<shortname>.toml.

            If the hosts file doesn't exist or does not declare `modules`, return None to
            indicate no filtering should be applied (i.e., show all modules).
            """
            try:
                # reuse shortname function from module_catalog if available
                from .module_catalog import _host_shortname

                short = _host_shortname()
            except Exception:
                short = None

            repo_root = _find_repo_root_with_hosts()
            if repo_root is None or short is None:
                return None
            host_file = repo_root / "hosts" / f"{short}.toml"
            if not host_file.exists():
                return None
            try:
                contents = host_file.read_text(encoding="utf-8")
                data = tomllib.loads(contents)
                modules = data.get("modules")
                if modules is None:
                    return None
                if not isinstance(modules, list):
                    return None
                # Preserve the order declared in hosts/*.toml
                return [str(m) for m in modules]
            except Exception:
                return None

        @router.get("/modules")
        async def list_modules() -> Dict[str, Any]:
            modules = [module.to_dict() for module in self.catalog.list_modules()]
            enabled = _enabled_modules_for_host()
            if enabled is None:
                return {"modules": modules}
            # Order modules according to the host's declared list and include only those entries.
            modules_by_name = {m.get("name"): m for m in modules}
            ordered: list[dict] = []
            for name in enabled:
                item = modules_by_name.get(name)
                if item is not None:
                    ordered.append(item)
            return {"modules": ordered}

        @router.get("/topics")
        async def list_topics() -> Dict[str, Any]:
            sessions = [session.to_dict() for session in self.topic_manager.list_sessions()]
            return {"sessions": sessions}

        @router.post("/modules/{module}/commands", status_code=202)
        async def run_command(module: str, payload: CommandRequest) -> Dict[str, Any]:
            if payload.scope == "mod":
                catalog_entry = self.catalog.get_module(module)
                if payload.command not in catalog_entry.commands.mod:
                    raise HTTPException(status_code=400, detail="Unsupported module command")
            else:
                catalog_entry = self.catalog.get_module(module)
                if payload.command not in catalog_entry.commands.system:
                    raise HTTPException(status_code=400, detail="Unsupported system command")
            result = await self.command_executor.run(payload.scope, module, payload.command, payload.args or [])
            return {"result": result}

        @router.post("/topics", status_code=201)
        async def create_topic(payload: TopicRequest) -> Dict[str, Any]:
            message_type = payload.message_type
            qos = payload.qos
            module_name = payload.module
            if message_type is None:
                module_topic = _find_topic(self.catalog, payload.topic, module_name)
                if module_topic is None:
                    raise HTTPException(status_code=404, detail="Unknown topic")
                message_type = module_topic.type
                qos = qos or module_topic.qos.asdict()
            try:
                session = self.topic_manager.create_session(
                    topic=payload.topic,
                    access=payload.access,
                    qos=qos,
                    module=module_name,
                    message_type=message_type,
                )
            except RuntimeError as exc:
                if "rosidl_runtime_py" in str(exc):
                    raise HTTPException(
                        status_code=503,
                        detail=(
                            "rosidl_runtime_py is required for message introspection. "
                            "Install ROS 2 Python message runtime or run with the workspace "
                            "environment sourced (install/setup.bash)"
                        ),
                    )
                raise

            return {"session": session.to_dict()}

        @router.delete("/topics/{session_id}", status_code=204)
        async def delete_topic(session_id: str) -> JSONResponse:
            self.topic_manager.drop_session(session_id)
            return JSONResponse(status_code=204, content=None)

        @router.post("/topics/{session_id}/pause")
        async def pause_topic(session_id: str, payload: PauseRequest) -> Dict[str, Any]:
            session = self.topic_manager.set_paused(session_id, payload.paused)
            return {"session": session.to_dict()}

        if self.voice_config_store is not None:

            @router.get("/voice/config")
            async def get_voice_config() -> Dict[str, Any]:
                config = self.voice_config_store.load()
                return {"config": config}

            @router.put("/voice/config")
            async def update_voice_config(payload: VoiceConfigUpdate) -> Dict[str, Any]:
                updates = payload.model_dump(exclude_none=True)
                if not updates:
                    raise HTTPException(status_code=400, detail="No updates provided")
                try:
                    config = self.voice_config_store.update(updates)
                except ValueError as exc:
                    raise HTTPException(status_code=400, detail=str(exc)) from exc
                return {"config": config}

        @self.app.websocket("/ws/topics/{session_id}")
        async def websocket_topic(session_id: str, websocket: WebSocket) -> None:
            session = self.topic_manager.get_session(session_id)
            if not session:
                await websocket.close(code=4404)
                return
            await websocket.accept()

            send_task = None
            receive_task = None

            async def sender_loop():
                async for payload in self.topic_manager.pump_messages(session_id):
                    if session.paused:
                        await asyncio.sleep(0.05)
                        continue
                    await websocket.send_json({"topic": session.topic, "data": payload})

            async def receiver_loop():
                while True:
                    message = await websocket.receive_text()
                    data = json.loads(message)
                    await self.topic_manager.publish(session_id, data)

            try:
                if session.access in {"ro", "rw"}:
                    send_task = asyncio.create_task(sender_loop())
                if session.access in {"wo", "rw"}:
                    receive_task = asyncio.create_task(receiver_loop())

                tasks = [task for task in [send_task, receive_task] if task is not None]
                if not tasks:
                    await websocket.close(code=4400)
                    return
                await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            except WebSocketDisconnect:  # pragma: no cover - handled during runtime
                pass
            finally:
                if send_task:
                    send_task.cancel()
                if receive_task:
                    receive_task.cancel()

        self.app.include_router(router)

        if self.static_root and self.static_root.exists():
            # Register static file mount after websocket routes so the WS handshake
            # is not intercepted by the StaticFiles ASGI app (which only handles HTTP).
            self.app.mount("/", StaticFiles(directory=str(self.static_root), html=True), name="static")

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------
    def build_app(self) -> FastAPI:
        return self.app


def _find_topic(catalog: ModuleCatalog, topic_name: str, module_name: str | None) -> ModuleTopic | None:
    if module_name:
        try:
            module = catalog.get_module(module_name)
        except KeyError:  # pragma: no cover - validated earlier
            return None
        for topic in module.topics:
            if topic.topic == topic_name:
                return topic
        return None
    for module in catalog.list_modules():
        for topic in module.topics:
            if topic.topic == topic_name:
                return topic
    return None


def create_app(
    *,
    catalog: ModuleCatalog,
    topic_manager: TopicSessionManager,
    command_executor: CommandExecutor,
    static_root: Path | None = None,
    voice_config_store: VoiceConfigStore | None = None,
) -> FastAPI:
    """Factory used by tests and runtime entrypoints."""

    application = PilotApplication(
        catalog=catalog,
        topic_manager=topic_manager,
        command_executor=command_executor,
        static_root=static_root,
        voice_config_store=voice_config_store,
    )
    return application.build_app()
