"""Command line entry point for sampling and narrating viscera feelings."""
from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

from .feelers import DEFAULT_FEELERS
from .monitor import SystemMetricsProbe, Viscera


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Narrate Pete's visceral status.")
    parser.add_argument(
        "--interval",
        type=float,
        default=5.0,
        help="Seconds between samples (default: 5.0)",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Maximum number of sentiments to print per cycle.",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    """Entry point for the ``viscera_monitor`` console script."""

    parser = _build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    probe = SystemMetricsProbe()
    viscera = Viscera(feelers=DEFAULT_FEELERS, probe=probe, sentiment_limit=args.limit)

    try:
        while True:
            state = probe.sample()
            for sentence in viscera.narrate(state):
                print(sentence)
            time.sleep(max(0.1, args.interval))
    except KeyboardInterrupt:
        return 0
    except Exception as exc:  # pragma: no cover - defensive guard for CLI users.
        print(f"[viscera] error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover - script guard.
    sys.exit(main())
