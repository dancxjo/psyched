"""Command line interface for the cockpit cockpit server."""

from __future__ import annotations

import argparse
import asyncio
import functools
import logging
import os
import signal
import site
import socket
import sys
from pathlib import Path
from typing import Callable, Iterable, Optional

from .server import CockpitServer, CockpitSettings

_LOGGER = logging.getLogger(__name__)


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Cockpit cockpit server")
    parser.add_argument("--host-config", type=Path, help="Path to host configuration file")
    parser.add_argument("--frontend-root", type=Path, help="Directory containing frontend assets")
    parser.add_argument("--listen-host", default="0.0.0.0", help="Host/IP to bind the HTTP server")
    parser.add_argument("--listen-port", type=int, default=8088, help="Port for the HTTP server")
    parser.add_argument("--modules-root", type=Path, help="Directory containing module manifests")
    parser.add_argument("--repo-root", type=Path, help="Repository root for psh execution")
    parser.add_argument("--log-level", default=os.environ.get("COCKPIT_LOG_LEVEL", "INFO"))
    return parser.parse_args(argv)


def resolve_host_config(explicit: Optional[Path]) -> Path:
    candidates = []
    if explicit:
        candidates.append(explicit)
    env_path = os.environ.get("COCKPIT_HOST_CONFIG")
    if env_path:
        candidates.append(Path(env_path))

    repo_dir = Path(os.environ.get("REPO_DIR", Path.cwd()))
    host_short = os.environ.get("HOST", socket.gethostname().split(".")[0])
    candidates.append(repo_dir / "hosts" / f"{host_short}.toml")

    for candidate in candidates:
        if candidate and candidate.exists():
            return candidate.resolve()
    raise SystemExit("Unable to locate host configuration; set --host-config or COCKPIT_HOST_CONFIG")


def resolve_frontend_root(explicit: Optional[Path]) -> Path:
    """Return a usable frontend directory or abort if none is available."""

    if explicit:
        if explicit.exists() and explicit.is_dir():
            return explicit.resolve()
        raise SystemExit("Frontend root must reference a directory")

    env_path = os.environ.get("COCKPIT_FRONTEND_ROOT")
    if env_path:
        path = Path(env_path)
        if path.exists() and path.is_dir():
            return path.resolve()

    repo_dir_env = os.environ.get("REPO_DIR")
    if repo_dir_env:
        repo_frontend = Path(repo_dir_env) / "modules" / "cockpit" / "packages" / "cockpit" / "cockpit" / "frontend"
        if repo_frontend.exists() and repo_frontend.is_dir():
            return repo_frontend.resolve()

    package_frontend = Path(__file__).resolve().parent / "frontend"
    if package_frontend.exists() and package_frontend.is_dir():
        return package_frontend.resolve()

    repo_dir = Path(os.environ.get("REPO_DIR", Path.cwd()))
    repo_frontend = repo_dir / "modules" / "cockpit" / "packages" / "cockpit" / "cockpit" / "frontend"
    if repo_frontend.exists() and repo_frontend.is_dir():
        return repo_frontend.resolve()

    raise SystemExit("Unable to locate cockpit frontend assets; set --frontend-root or COCKPIT_FRONTEND_ROOT")


def _looks_like_modules_dir(path: Path) -> bool:
    """Return ``True`` when *path* appears to contain module manifests."""

    if not path.exists() or not path.is_dir():
        return False

    try:
        for entry in path.iterdir():
            if not entry.is_dir():
                continue
            if (entry / "module.toml").exists():
                return True
    except OSError:
        return False
    return False


def resolve_modules_root(explicit: Optional[Path], frontend_root: Path) -> Path:
    """Return the directory containing module manifests."""

    candidates: list[Path] = []
    if explicit:
        candidates.append(explicit)

    env_path = os.environ.get("COCKPIT_MODULES_ROOT")
    if env_path:
        candidates.append(Path(env_path))

    repo_dir = Path(os.environ.get("REPO_DIR", frontend_root.parent.parent.parent))
    candidates.append(repo_dir / "modules")

    # Packaged installations bundle the cockpit alongside a copy of the
    # module manifests under ``modules/``.
    candidates.append(frontend_root.parent.parent / "modules")

    # Historical fall-back: assume the modules directory is two levels above the
    # frontend bundle (modules/cockpit/packages). This is only correct when the
    # cockpit package lives inside the repository checkout.
    candidates.append(frontend_root.parent.parent)

    for candidate in candidates:
        if candidate and _looks_like_modules_dir(candidate):
            return candidate.resolve()

    raise SystemExit("Unable to locate modules directory; set --modules-root or COCKPIT_MODULES_ROOT")


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


def _register_signal_handlers(
    loop: asyncio.AbstractEventLoop, *, stop_callback: Callable[[], None]
) -> None:
    """Register signal handlers so the cockpit shuts down gracefully.

    POSIX platforms rely on :py:meth:`asyncio.AbstractEventLoop.add_signal_handler`
    for ``SIGINT`` and ``SIGTERM`` while ``SIGHUP`` is logged and ignored so the
    cockpit keeps running when the parent terminal disappears. Windows lacks
    :py:meth:`add_signal_handler`, so the legacy :mod:`signal` API is used
    instead.
    """

    supports_asyncio_signals = sys.platform != "win32" and hasattr(
        loop, "add_signal_handler"
    )

    stop_signals: list[signal.Signals] = []
    for name in ("SIGINT", "SIGTERM"):
        candidate = getattr(signal, name, None)
        if isinstance(candidate, signal.Signals):
            stop_signals.append(candidate)

    def request_shutdown(signal_name: str) -> None:
        _LOGGER.info("Received %s signal; shutting down cockpit server", signal_name)
        stop_callback()

    def log_ignored(signal_name: str) -> None:
        _LOGGER.info("Received %s signal; ignoring to keep cockpit server running", signal_name)

    sighup = getattr(signal, "SIGHUP", None)
    if supports_asyncio_signals:
        for sig in stop_signals:
            loop.add_signal_handler(sig, functools.partial(request_shutdown, sig.name))
        if isinstance(sighup, signal.Signals):
            loop.add_signal_handler(sighup, functools.partial(log_ignored, sighup.name))
    else:
        for sig in stop_signals:
            signal.signal(
                sig,
                lambda *_args, sig=sig: request_shutdown(sig.name),
            )
        if isinstance(sighup, signal.Signals):
            signal.signal(
                sighup,
                lambda *_args, sig=sighup: log_ignored(sig.name),
            )


def main(argv: Optional[Iterable[str]] = None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))

    host_config = resolve_host_config(args.host_config)
    frontend_root = resolve_frontend_root(args.frontend_root)
    modules_root = resolve_modules_root(args.modules_root, frontend_root)
    repo_root = resolve_repo_root(args.repo_root, modules_root)

    _bootstrap_ros_environment(repo_root)

    bridge_mode = os.environ.get("COCKPIT_BRIDGE_MODE", "actions").lower()
    video_base = os.environ.get("COCKPIT_VIDEO_BASE")
    video_port_env = os.environ.get("COCKPIT_VIDEO_PORT")
    video_port = None
    if video_port_env:
        try:
            parsed_port = int(video_port_env)
        except ValueError:
            parsed_port = None
            _LOGGER.warning("Invalid COCKPIT_VIDEO_PORT value '%s'", video_port_env)
        if parsed_port is not None and parsed_port > 0:
            video_port = parsed_port

    settings = CockpitSettings(
        host_config_path=host_config,
        frontend_root=frontend_root,
        modules_root=modules_root,
        repo_root=repo_root,
        listen_host=args.listen_host,
        listen_port=args.listen_port,
        bridge_mode=bridge_mode,
        video_base=video_base,
        video_port=video_port,
    )

    server = CockpitServer(settings=settings)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    stop_event = asyncio.Event()
    _register_signal_handlers(
        loop,
        stop_callback=lambda: loop.call_soon_threadsafe(stop_event.set),
    )

    async def run_server() -> None:
        async with server:
            await stop_event.wait()

    try:
        loop.run_until_complete(run_server())
    except KeyboardInterrupt:
        _LOGGER.info("Shutting down cockpit server")
        loop.call_soon_threadsafe(stop_event.set)
    finally:
        loop.run_until_complete(server.stop())
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()


def _bootstrap_ros_environment(repo_root: Path) -> None:
    """Ensure colcon overlays from *repo_root* are available to Python imports."""

    install_root = repo_root / "work" / "install"
    if not install_root.exists():
        return

    _augment_pythonpath(install_root)
    _ensure_ament_prefix_path(install_root)


def _augment_pythonpath(install_root: Path) -> None:
    """Add colcon-generated site-packages directories under *install_root* to sys.path."""

    python_tag = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [install_root / "lib" / python_tag / "site-packages"]
    candidates.extend(install_root.glob("*/lib/python*/site-packages"))

    normalized_sys_path = {Path(entry).resolve() for entry in sys.path if entry}
    for candidate in candidates:
        try:
            resolved = candidate.resolve()
        except FileNotFoundError:
            continue
        if not resolved.exists() or resolved in normalized_sys_path:
            continue
        site.addsitedir(str(resolved))
        normalized_sys_path.add(resolved)
        _LOGGER.debug("Added site-packages directory %s", resolved)


def _ensure_ament_prefix_path(install_root: Path) -> None:
    """Prepend *install_root* and package-specific prefixes to AMENT_PREFIX_PATH."""

    existing_raw = os.environ.get("AMENT_PREFIX_PATH", "")
    existing_paths = [Path(entry).resolve() for entry in existing_raw.split(os.pathsep) if entry]
    normalized = {path for path in existing_paths}

    prefixes = [install_root]
    prefixes.extend(path for path in install_root.iterdir() if path.is_dir())

    updated = list(existing_paths)
    for prefix in prefixes:
        try:
            resolved = prefix.resolve()
        except FileNotFoundError:
            continue
        if not resolved.exists() or resolved in normalized:
            continue
        updated.insert(0, resolved)
        normalized.add(resolved)

    os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join(str(path) for path in updated)


if __name__ == "__main__":  # pragma: no cover
    main()
