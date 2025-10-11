"""Command line interface for the pilot cockpit server."""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
from pathlib import Path
import socket
from typing import Iterable, Optional

from .server import CockpitServer, PilotSettings
from .topics import RosTopicBridge

_LOGGER = logging.getLogger(__name__)


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pilot cockpit server")
    parser.add_argument("--host-config", type=Path, help="Path to host configuration file")
    parser.add_argument("--frontend-root", type=Path, help="Directory containing frontend assets")
    parser.add_argument("--listen-host", default="0.0.0.0", help="Host/IP to bind the HTTP server")
    parser.add_argument("--listen-port", type=int, default=8088, help="Port for the HTTP server")
    parser.add_argument("--log-level", default=os.environ.get("PILOT_LOG_LEVEL", "INFO"))
    return parser.parse_args(argv)


def resolve_host_config(explicit: Optional[Path]) -> Path:
    candidates = []
    if explicit:
        candidates.append(explicit)
    env_path = os.environ.get("PILOT_HOST_CONFIG")
    if env_path:
        candidates.append(Path(env_path))

    repo_dir = Path(os.environ.get("REPO_DIR", Path.cwd()))
    host_short = os.environ.get("HOST", socket.gethostname().split(".")[0])
    for suffix in (".json", ".jsonc", ".yaml", ".yml", ".toml"):
        candidates.append(repo_dir / "hosts" / f"{host_short}{suffix}")

    for candidate in candidates:
        if candidate and candidate.exists():
            return candidate.resolve()
    raise SystemExit("Unable to locate host configuration; set --host-config or PILOT_HOST_CONFIG")


def resolve_frontend_root(explicit: Optional[Path]) -> Path:
    if explicit and explicit.exists():
        return explicit.resolve()
    env_path = os.environ.get("PILOT_FRONTEND_ROOT")
    if env_path:
        path = Path(env_path)
        if path.exists():
            return path.resolve()
    repo_dir = Path(os.environ.get("REPO_DIR", Path.cwd()))
    fallback = repo_dir / "modules" / "pilot" / "frontend"
    if fallback.exists():
        return fallback.resolve()
    raise SystemExit("Unable to locate cockpit frontend assets; set --frontend-root or PILOT_FRONTEND_ROOT")


def main(argv: Optional[Iterable[str]] = None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))

    host_config = resolve_host_config(args.host_config)
    frontend_root = resolve_frontend_root(args.frontend_root)

    settings = PilotSettings(
        host_config_path=host_config,
        frontend_root=frontend_root,
        listen_host=args.listen_host,
        listen_port=args.listen_port,
    )

    bridge = RosTopicBridge()
    server = CockpitServer(settings=settings, ros_bridge=bridge)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def run_server() -> None:
        async with server:
            while True:
                await asyncio.sleep(3600)

    try:
        loop.run_until_complete(run_server())
    except KeyboardInterrupt:
        _LOGGER.info("Shutting down cockpit server")
    finally:
        loop.run_until_complete(server.stop())
        bridge.shutdown()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()


if __name__ == "__main__":  # pragma: no cover
    main()
