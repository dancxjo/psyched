"""Command line entry point for sampling and narrating viscera feelings."""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Dict, Iterable

from .feelers import DEFAULT_FEELERS
from .monitor import SystemMetricsProbe, Viscera
from .state import BatteryState, FootState, SystemState


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


def _state_stats(state: SystemState) -> Dict[str, Any]:
    """Return a serialisable snapshot of :class:`SystemState` metrics."""

    battery: BatteryState = state.battery
    foot: FootState | None = state.foot
    return {
        "timestamp": state.timestamp.isoformat(),
        "battery": {
            "charge_fraction": battery.charge_fraction,
            "is_charging": battery.is_charging,
            "temperature_c": battery.temperature_c,
            "health_fraction": battery.health_fraction,
        },
        "cpu_load": state.cpu_load,
        "memory_load": state.memory_load,
        "disk_fill_level": state.disk_fill_level,
        "swap_fraction": state.swap_fraction,
        "uptime_sec": state.uptime_sec,
        "temperature_c": state.temperature_c,
        "process_count": state.process_count,
        "foot": None
        if foot is None
        else {
            "hazard_contacts": foot.hazard_contacts,
            "is_slipping": foot.is_slipping,
            "contact_confidence": foot.contact_confidence,
            "temperature_c": foot.temperature_c,
        },
    }


def main(argv: Iterable[str] | None = None) -> int:
    """Entry point for the ``viscera_monitor`` console script."""

    parser = _build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    probe = SystemMetricsProbe()
    viscera = Viscera(feelers=DEFAULT_FEELERS, probe=probe, sentiment_limit=args.limit)

    try:
        while True:
            state = probe.sample()
            stats_json = json.dumps(_state_stats(state), sort_keys=True)
            sentiments = viscera.feelings(state)
            if not sentiments:
                print(f"[viscera] stats={stats_json}")
            else:
                for sentiment in sentiments:
                    evidence_json = json.dumps(dict(sentiment.evidence), sort_keys=True)
                    tags_repr = f"[{', '.join(sentiment.tags)}]" if sentiment.tags else "[]"
                    print(
                        f"{sentiment.narrative} "
                        f"| intensity={sentiment.intensity:.3f} "
                        f"| tags={tags_repr} "
                        f"| evidence={evidence_json} "
                        f"| stats={stats_json}"
                    )
            time.sleep(max(0.1, args.interval))
    except KeyboardInterrupt:
        return 0
    except Exception as exc:  # pragma: no cover - defensive guard for CLI users.
        print(f"[viscera] error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover - script guard.
    sys.exit(main())
