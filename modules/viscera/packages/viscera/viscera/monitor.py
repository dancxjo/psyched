"""Coordinate feelers and capture the sensory snapshot backing them."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import os
import shutil
from typing import Callable, Dict, List, Optional, Sequence

from .feelers import DEFAULT_FEELERS, Feeler
from .sentiment import Sentiment
from .state import BatteryState, FootState, SystemState


def _clamp(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    return max(0.0, min(1.0, value))


@dataclass
class SystemMetricsProbe:
    """Sample host metrics with optional ``psutil`` acceleration."""

    psutil_module: object | None = None
    mount_point: str = "/"
    foot_state_provider: Optional[Callable[[], Optional[FootState]]] = None
    clock: Callable[[], datetime] = lambda: datetime.now(timezone.utc)

    def __post_init__(self) -> None:
        if self.psutil_module is None:
            try:  # pragma: no cover - import guarded for environments without psutil.
                import psutil  # type: ignore
            except Exception:  # pragma: no cover - import failure is acceptable.
                psutil = None  # type: ignore
            self.psutil_module = psutil

    def sample(self) -> SystemState:
        """Collect a :class:`SystemState` snapshot."""

        timestamp = self.clock()
        battery = self._sample_battery()
        cpu = self._sample_cpu()
        memory = self._sample_memory()
        disk = self._sample_disk()
        swap = self._sample_swap()
        uptime = self._sample_uptime(timestamp)
        temperature = self._sample_temperature()
        process_count = self._sample_process_count()
        foot = self.foot_state_provider() if self.foot_state_provider else None
        return SystemState(
            timestamp=timestamp,
            battery=battery,
            cpu_load=cpu,
            memory_load=memory,
            disk_fill_level=disk,
            foot=foot,
            swap_fraction=swap,
            uptime_sec=uptime,
            temperature_c=temperature,
            process_count=process_count,
        )

    # Sampling helpers -------------------------------------------------
    def _sample_battery(self) -> BatteryState:
        psutil = self.psutil_module
        if psutil is None or not hasattr(psutil, "sensors_battery"):
            return BatteryState()
        battery = psutil.sensors_battery()
        if battery is None:
            return BatteryState()
        percent = getattr(battery, "percent", None)
        fraction = None if percent is None else _clamp(percent / 100.0)
        is_plugged = getattr(battery, "power_plugged", None)
        return BatteryState(charge_fraction=fraction, is_charging=is_plugged)

    def _sample_cpu(self) -> Optional[float]:
        psutil = self.psutil_module
        if psutil is not None and hasattr(psutil, "cpu_percent"):
            try:
                return _clamp(psutil.cpu_percent(interval=None) / 100.0)
            except Exception:
                pass
        try:
            load_averages = os.getloadavg()
            cpu_count = os.cpu_count() or 1
            return _clamp(load_averages[0] / cpu_count)
        except (AttributeError, OSError):
            return None

    def _sample_memory(self) -> Optional[float]:
        psutil = self.psutil_module
        if psutil is not None and hasattr(psutil, "virtual_memory"):
            try:
                return _clamp(psutil.virtual_memory().percent / 100.0)
            except Exception:
                pass
        try:
            page_size = os.sysconf("SC_PAGE_SIZE")
            phys_pages = os.sysconf("SC_PHYS_PAGES")
            avail_pages = os.sysconf("SC_AVPHYS_PAGES")
        except (AttributeError, ValueError, OSError):
            return None
        total = page_size * phys_pages
        available = page_size * avail_pages
        if total <= 0:
            return None
        used_ratio = 1.0 - (available / total)
        return _clamp(used_ratio)

    def _sample_disk(self) -> Optional[float]:
        try:
            usage = shutil.disk_usage(self.mount_point)
        except (FileNotFoundError, PermissionError, OSError):
            return None
        if usage.total <= 0:
            return None
        return _clamp(usage.used / usage.total)

    def _sample_swap(self) -> Optional[float]:
        psutil = self.psutil_module
        if psutil is not None and hasattr(psutil, "swap_memory"):
            try:
                swap = psutil.swap_memory()
                total = getattr(swap, "total", 0) or 0
                used = getattr(swap, "used", 0) or 0
                if total <= 0:
                    return None
                return _clamp(used / total)
            except Exception:
                pass
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as meminfo:
                stats = {}
                for line in meminfo:
                    key, value = line.split(":", 1)
                    parts = value.strip().split()
                    if not parts:
                        continue
                    stats[key.strip()] = float(parts[0])  # values reported in kB
            total = stats.get("SwapTotal")
            free = stats.get("SwapFree")
            if not total or total <= 0.0:
                return None
            used = total - (free or 0.0)
            return _clamp(used / total)
        except (FileNotFoundError, PermissionError, OSError, ValueError):
            return None

    def _sample_uptime(self, now: datetime) -> Optional[float]:
        psutil = self.psutil_module
        if psutil is not None and hasattr(psutil, "boot_time"):
            try:
                boot_time = float(psutil.boot_time())
                if boot_time > 0:
                    return max(0.0, now.timestamp() - boot_time)
            except Exception:
                pass
        try:
            with open("/proc/uptime", "r", encoding="utf-8") as proc_uptime:
                first_field = proc_uptime.read().split()[0]
                return float(first_field)
        except (FileNotFoundError, PermissionError, OSError, IndexError, ValueError):
            return None

    def _sample_temperature(self) -> Optional[float]:
        psutil = self.psutil_module
        if psutil is None or not hasattr(psutil, "sensors_temperatures"):
            return None
        try:
            temps = psutil.sensors_temperatures() or {}
        except Exception:
            return None
        for readings in temps.values():
            for entry in readings:
                value = getattr(entry, "current", None)
                if value is None:
                    continue
                try:
                    return float(value)
                except (TypeError, ValueError):
                    continue
        return None

    def _sample_process_count(self) -> Optional[float]:
        psutil = self.psutil_module
        if psutil is not None:
            try:
                if hasattr(psutil, "pids"):
                    return float(len(psutil.pids()))
                if hasattr(psutil, "process_iter"):
                    return float(sum(1 for _ in psutil.process_iter()))
            except Exception:
                pass
        try:
            entries = os.listdir("/proc")
        except (FileNotFoundError, PermissionError, OSError):
            return None
        count = sum(1 for entry in entries if entry.isdigit())
        return float(count)


class Viscera:
    """Coordinate feelers and produce sorted sentiments."""

    def __init__(
        self,
        feelers: Sequence[Feeler] | None = None,
        *,
        probe: Optional[SystemMetricsProbe] = None,
        sentiment_limit: Optional[int] = None,
    ) -> None:
        self._feelers: Sequence[Feeler] = feelers or DEFAULT_FEELERS
        self._probe = probe
        self._sentiment_limit = sentiment_limit

    def sample(self) -> SystemState:
        """Request a state snapshot from the configured probe."""

        if self._probe is None:
            raise RuntimeError("No probe configured; supply a SystemMetricsProbe or a manual state")
        return self._probe.sample()

    def feelings(self, state: Optional[SystemState] = None) -> List[Sentiment]:
        """Return aggregated sentiments for *state* sorted by intensity."""

        snapshot = state or self.sample()
        aggregated: Dict[str, Sentiment] = {}
        for feeler in self._feelers:
            for sentiment in feeler(snapshot):
                existing = aggregated.get(sentiment.name)
                if existing is None or sentiment.intensity > existing.intensity:
                    aggregated[sentiment.name] = sentiment
        ordered = sorted(aggregated.values(), key=lambda s: s.intensity, reverse=True)
        if self._sentiment_limit is not None:
            return ordered[: self._sentiment_limit]
        return ordered

    def narrate(self, state: Optional[SystemState] = None) -> List[str]:
        """Return the narrative components of :meth:`feelings`."""

        return [sentiment.narrative for sentiment in self.feelings(state)]


__all__ = ["SystemMetricsProbe", "Viscera"]
