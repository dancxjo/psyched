"""Command line entry point for the cockpit websocket bridge."""
from __future__ import annotations

import argparse
import asyncio
import logging

from .bridge import run_bridge


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pilot cockpit websocket bridge")
    parser.add_argument("--host", default="0.0.0.0", help="Host interface to bind (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8088, help="Websocket port (default: 8088)")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP port for web UI (default: 8080)")
    parser.add_argument("--log-level", default="INFO", help="Python logging level (default: INFO)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    asyncio.run(run_bridge(host=args.host, port=args.port, http_port=args.http_port))


if __name__ == "__main__":
    main()
