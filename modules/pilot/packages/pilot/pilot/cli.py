"""Command line interface for the pilot cockpit server."""

from __future__ import annotations

import argparse
import asyncio
import socket
import logging
import os
from pathlib import Path
from typing import Iterable, Optional

from .server import CockpitServer, PilotSettings

_LOGGER = logging.getLogger(__name__)


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pilot cockpit server")
    parser.add_argument("--host-config", type=Path, help="Path to host configuration file")
    parser.add_argument("--frontend-root", type=Path, help="Directory containing frontend assets")
    parser.add_argument("--listen-host", default="0.0.0.0", help="Host/IP to bind the HTTP server")
    parser.add_argument("--listen-port", type=int, default=8088, help="Port for the HTTP server")
    parser.add_argument("--modules-root", type=Path, help="Directory containing module manifests")
    parser.add_argument("--repo-root", type=Path, help="Repository root for psh execution")
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
    """Return a usable frontend directory or abort if none is available."""

    if explicit:
        if explicit.exists() and explicit.is_dir():
            return explicit.resolve()
        raise SystemExit("Frontend root must reference a directory")

    env_path = os.environ.get("PILOT_FRONTEND_ROOT")
    if env_path:
        path = Path(env_path)
        if path.exists() and path.is_dir():
            return path.resolve()

    repo_dir_env = os.environ.get("REPO_DIR")
    if repo_dir_env:
        repo_frontend = Path(repo_dir_env) / "modules" / "pilot" / "packages" / "pilot" / "pilot" / "frontend"
        if repo_frontend.exists() and repo_frontend.is_dir():
            return repo_frontend.resolve()

    package_frontend = Path(__file__).resolve().parent / "frontend"
    if package_frontend.exists() and package_frontend.is_dir():
        return package_frontend.resolve()

    repo_dir = Path(os.environ.get("REPO_DIR", Path.cwd()))
    repo_frontend = repo_dir / "modules" / "pilot" / "packages" / "pilot" / "pilot" / "frontend"
    if repo_frontend.exists() and repo_frontend.is_dir():
        return repo_frontend.resolve()

    raise SystemExit("Unable to locate cockpit frontend assets; set --frontend-root or PILOT_FRONTEND_ROOT")


def resolve_modules_root(explicit: Optional[Path], frontend_root: Path) -> Path:
    """Return the directory containing module manifests."""

    candidates: list[Path] = []
    if explicit:
        candidates.append(explicit)
    env_path = os.environ.get("PILOT_MODULES_ROOT")
    if env_path:
        candidates.append(Path(env_path))
    candidates.append(frontend_root.parent.parent)
    repo_dir = Path(os.environ.get("REPO_DIR", frontend_root.parent.parent.parent))
    candidates.append(repo_dir / "modules")
    for candidate in candidates:
        if candidate and candidate.exists() and candidate.is_dir():
            return candidate.resolve()
    raise SystemExit("Unable to locate modules directory; set --modules-root or PILOT_MODULES_ROOT")


def resolve_repo_root(explicit: Optional[Path], modules_root: Path) -> Path:
    """Return the repository root path."""

    candidates: list[Path] = []
    if explicit:
        candidates.append(explicit)
    env_path = os.environ.get("REPO_DIR")
    if env_path:
        candidates.append(Path(env_path))
    candidates.append(modules_root.parent)
    for candidate in candidates:
        if candidate and candidate.exists() and candidate.is_dir():
            return candidate.resolve()
    raise SystemExit("Unable to determine repository root; set --repo-root or REPO_DIR")


def main(argv: Optional[Iterable[str]] = None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))

    host_config = resolve_host_config(args.host_config)
    frontend_root = resolve_frontend_root(args.frontend_root)
    modules_root = resolve_modules_root(args.modules_root, frontend_root)
    repo_root = resolve_repo_root(args.repo_root, modules_root)

    bridge_mode = os.environ.get("PILOT_BRIDGE_MODE", "rosbridge").lower()
    rosbridge_uri = os.environ.get("PILOT_ROSBRIDGE_URI", "ws://127.0.0.1:9090")
    video_base = os.environ.get("PILOT_VIDEO_BASE")
    video_port_env = os.environ.get("PILOT_VIDEO_PORT")
    video_port = None
    if video_port_env:
        try:
            parsed_port = int(video_port_env)
        except ValueError:
            parsed_port = None
            _LOGGER.warning("Invalid PILOT_VIDEO_PORT value '%s'", video_port_env)
        if parsed_port is not None and parsed_port > 0:
            video_port = parsed_port

    settings = PilotSettings(
        host_config_path=host_config,
        frontend_root=frontend_root,
        modules_root=modules_root,
        repo_root=repo_root,
        listen_host=args.listen_host,
        listen_port=args.listen_port,
        bridge_mode=bridge_mode,
        rosbridge_uri=rosbridge_uri,
        video_base=video_base,
        video_port=video_port,
    )

    server = CockpitServer(settings=settings)

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
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()


if __name__ == "__main__":  # pragma: no cover
    main()
