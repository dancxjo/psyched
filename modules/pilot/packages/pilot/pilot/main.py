"""Entrypoint for running the pilot backend server."""

from __future__ import annotations

import argparse
import asyncio
import signal
import threading
from pathlib import Path
from typing import Optional

import uvicorn

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise RuntimeError("rclpy is required to run the pilot backend") from exc

from .app import CommandExecutor, create_app
from .module_catalog import ModuleCatalog
from .topic_manager import TopicSessionManager


def find_repo_root(start: Path) -> Path:
    """Locate the repository root given a path inside the pilot package."""

    current = start.resolve()
    if current.is_file():
        current = current.parent

    for candidate in (current, *current.parents):
        modules_dir = candidate / "modules"
        psh_entry = candidate / "psh" / "main.ts"
        if modules_dir.exists() and psh_entry.exists():
            return candidate

    raise RuntimeError(
        "Unable to locate repository root – expected 'modules/' and 'psh/main.ts' sentinels."
    )


def _default_repo_root() -> Path:
    return find_repo_root(Path(__file__))


def _default_static_root() -> Path:
    return Path(__file__).resolve().parent / "static"


def _create_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pilot backend server")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP host to bind")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port to bind")
    parser.add_argument("--modules-root", type=Path, default=None, help="Root directory of module manifests")
    parser.add_argument("--static-root", type=Path, default=None, help="Directory of compiled frontend assets")
    parser.add_argument("--log-level", default="info", help="Uvicorn log level")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = _create_argument_parser()
    args = parser.parse_args(argv)

    repo_root = _default_repo_root()
    modules_root = args.modules_root or (repo_root / "modules")
    static_root = args.static_root or _default_static_root()

    rclpy.init()
    node = rclpy.create_node("pilot_backend")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    loop = asyncio.new_event_loop()

    topic_manager = TopicSessionManager(node=node, loop=loop)
    catalog = ModuleCatalog(modules_root)
    command_executor = CommandExecutor(repo_root)

    app = create_app(
        catalog=catalog,
        topic_manager=topic_manager,
        command_executor=command_executor,
        static_root=static_root,
    )

    def _spin_executor():
        with loop:
            loop.create_task(_executor_spin(executor))
            loop.run_forever()

    spin_thread = threading.Thread(target=_spin_executor, daemon=True)
    spin_thread.start()

    config = uvicorn.Config(app, host=str(args.host), port=args.port, log_level=args.log_level)
    server = uvicorn.Server(config)

    def _shutdown_handler(*_):  # noqa: D401
        loop.call_soon_threadsafe(loop.stop)

    signal.signal(signal.SIGINT, _shutdown_handler)
    signal.signal(signal.SIGTERM, _shutdown_handler)

    try:
        server.run()
    finally:
        loop.call_soon_threadsafe(loop.stop)
        spin_thread.join(timeout=2.0)
        topic_manager.shutdown()
        executor.shutdown()
        rclpy.shutdown()

    return 0


async def _executor_spin(executor: MultiThreadedExecutor) -> None:
    try:
        executor.spin()
    finally:  # pragma: no cover - cleanup path
        executor.shutdown()


if __name__ == "__main__":  # pragma: no cover - manual invocation
    raise SystemExit(main())
