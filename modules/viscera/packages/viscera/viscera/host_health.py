"""Host health sampling utilities for publishing Pete's system vitals.

The sampling logic is intentionally defensive so it can operate without the
optional :mod:`psutil` dependency. When psutil is unavailable, the sampler
falls back to standard library facilities such as :func:`os.getloadavg` and
``/proc`` readers where possible.

Examples
--------
>>> sampler = HostHealthSampler(psutil_module=None)  # force standard library paths
>>> sample = sampler.sample()
>>> isinstance(sample.cpu_percent, float)
True
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
import shutil
import socket
import time
from typing import Any, Callable, Dict, Iterable, Tuple


def _safe_float(value: object, *, scale: float = 1.0) -> float:
    """Return *value* coerced to ``float`` or :data:`math.nan` if conversion fails."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    if not math.isfinite(number):
        return math.nan
    return number * scale


def _resolve_host_identity(
    host: str | None = None, host_short: str | None = None
) -> Tuple[str, str]:
    """Return ``(host, host_short)`` with deterministic fallbacks."""

    resolved_host = (host or "").strip()
    if not resolved_host:
        try:  # pragma: no cover - networking quirks.
            resolved_host = socket.gethostname() or ""
        except Exception:
            resolved_host = ""
    if not resolved_host:
        resolved_host = "host"

    resolved_short = (host_short or "").strip()
    if not resolved_short:
        resolved_short = resolved_host.split(".")[0] or resolved_host
    return resolved_host, resolved_short


def _default_clock() -> float:
    return time.time()


_AUTO = object()


@dataclass(frozen=True, slots=True)
class HostHealthSample:
    """Snapshot of host vitals prepared for the :class:`HostHealthPublisher`.

    All numeric fields default to :data:`math.nan` so downstream consumers can
    rely on IEEE NaN semantics to represent missing data. This mirrors the
    ``psyched_msgs/msg/HostHealth`` message semantics where omitted readings are
    encoded as NaN.
    """

    host: str = "host"
    host_short: str = "host"
    cpu_percent: float = math.nan
    load_avg_1: float = math.nan
    load_avg_5: float = math.nan
    load_avg_15: float = math.nan
    mem_used_percent: float = math.nan
    mem_total_mb: float = math.nan
    mem_used_mb: float = math.nan
    disk_used_percent_root: float = math.nan
    temp_c: float = math.nan
    uptime_sec: float = math.nan
    swap_used_percent: float = math.nan
    swap_total_mb: float = math.nan
    swap_used_mb: float = math.nan
    process_count: float = math.nan


class HostHealthSampler:
    """Collect host metrics backing the cockpit's HostHealth UI panel.

    Parameters
    ----------
    psutil_module:
        Explicit psutil-like module used for sampling. By default the sampler
        tries to import :mod:`psutil` lazily. Pass ``None`` to force standard
        library fallbacks during testing.
    mount_point:
        Filesystem path used for disk usage sampling (defaults to ``/``).
    clock:
        Callable returning the current UNIX timestamp; useful for deterministic
        tests.
    """

    def __init__(
        self,
        *,
        psutil_module: Any | None = _AUTO,
        mount_point: str = "/",
        clock: Callable[[], float] = _default_clock,
        host: str | None = None,
        host_short: str | None = None,
    ) -> None:
        if psutil_module is _AUTO:
            try:  # pragma: no cover - import guard survives environments without psutil.
                import psutil  # type: ignore
            except Exception:  # pragma: no cover - the project should run without psutil.
                psutil = None  # type: ignore
            self._psutil = psutil
        else:
            self._psutil = psutil_module
        self._mount_point = mount_point
        self._clock = clock
        self._host, self._host_short = _resolve_host_identity(host, host_short)

    # Public API -----------------------------------------------------
    def sample(self) -> HostHealthSample:
        """Return a :class:`HostHealthSample` populated with the latest metrics."""

        cpu_percent = self._sample_cpu_percent()
        load_1, load_5, load_15 = self._sample_load_average()
        mem_used_percent, mem_total_mb, mem_used_mb = self._sample_memory()
        disk_used_percent = self._sample_disk_usage()
        swap_used_percent, swap_total_mb, swap_used_mb = self._sample_swap_memory()
        temperature = self._sample_temperature()
        uptime = self._sample_uptime()
        process_count = self._sample_process_count()

        return HostHealthSample(
            host=self._host,
            host_short=self._host_short,
            cpu_percent=cpu_percent,
            load_avg_1=load_1,
            load_avg_5=load_5,
            load_avg_15=load_15,
            mem_used_percent=mem_used_percent,
            mem_total_mb=mem_total_mb,
            mem_used_mb=mem_used_mb,
            disk_used_percent_root=disk_used_percent,
            temp_c=temperature,
            uptime_sec=uptime,
            swap_used_percent=swap_used_percent,
            swap_total_mb=swap_total_mb,
            swap_used_mb=swap_used_mb,
            process_count=process_count,
        )

    # Sampling helpers -----------------------------------------------
    def _sample_cpu_percent(self) -> float:
        psutil = self._psutil
        if psutil is not None and hasattr(psutil, "cpu_percent"):
            try:
                value = psutil.cpu_percent(interval=None)
                if value == 0.0:
                    # ``cpu_percent`` reports 0.0 the first time it is invoked.
                    # Issue a second non-blocking call so we surface a meaningful value.
                    value = psutil.cpu_percent(interval=None)
                return _safe_float(value)
            except Exception:
                return math.nan
        try:
            load_1, _, _ = os.getloadavg()
            cpu_count = os.cpu_count() or 1
            return float(max(0.0, (load_1 / cpu_count) * 100.0))
        except (AttributeError, OSError):
            return math.nan

    def _sample_load_average(self) -> tuple[float, float, float]:
        try:
            load_1, load_5, load_15 = os.getloadavg()
            return float(load_1), float(load_5), float(load_15)
        except (AttributeError, OSError):
            return math.nan, math.nan, math.nan

    def _sample_memory(self) -> tuple[float, float, float]:
        psutil = self._psutil
        if psutil is not None and hasattr(psutil, "virtual_memory"):
            try:
                vm = psutil.virtual_memory()
                percent = _safe_float(vm.percent)
                total_mb = _safe_float(vm.total, scale=1.0 / (1024 * 1024))
                used_mb = _safe_float(getattr(vm, "used", vm.total - vm.available), scale=1.0 / (1024 * 1024))
                return percent, total_mb, used_mb
            except Exception:
                pass
        try:
            page_size = os.sysconf("SC_PAGE_SIZE")
            phys_pages = os.sysconf("SC_PHYS_PAGES")
            avail_pages = os.sysconf("SC_AVPHYS_PAGES")
        except (AttributeError, ValueError, OSError):
            return math.nan, math.nan, math.nan
        total = float(page_size) * float(phys_pages)
        available = float(page_size) * float(avail_pages)
        if total <= 0.0:
            return math.nan, math.nan, math.nan
        used = max(0.0, total - available)
        percent = (used / total) * 100.0
        total_mb = total / (1024 * 1024)
        used_mb = used / (1024 * 1024)
        return percent, total_mb, used_mb

    def _sample_disk_usage(self) -> float:
        psutil = self._psutil
        if psutil is not None and hasattr(psutil, "disk_usage"):
            try:
                usage = psutil.disk_usage(self._mount_point)
                if getattr(usage, "total", 0) <= 0:
                    return math.nan
                return _safe_float(usage.used / usage.total, scale=100.0)
            except Exception:
                pass
        try:
            usage = shutil.disk_usage(self._mount_point)
        except (FileNotFoundError, PermissionError, OSError):
            return math.nan
        if usage.total <= 0:
            return math.nan
        return float((usage.used / usage.total) * 100.0)

    def _sample_temperature(self) -> float:
        psutil = self._psutil
        if psutil is None or not hasattr(psutil, "sensors_temperatures"):
            return math.nan
        try:
            temps: Dict[str, Iterable[object]] = psutil.sensors_temperatures() or {}
        except Exception:
            return math.nan
        for readings in temps.values():
            for entry in readings:
                current = getattr(entry, "current", None)
                value = _safe_float(current)
                if not math.isnan(value):
                    return value
        return math.nan

    def _sample_uptime(self) -> float:
        psutil = self._psutil
        if psutil is not None and hasattr(psutil, "boot_time"):
            try:
                boot_time = _safe_float(psutil.boot_time())
                now = self._clock()
                if math.isnan(boot_time):
                    return math.nan
                return max(0.0, now - boot_time)
            except Exception:
                pass
        try:
            with open("/proc/uptime", "r", encoding="utf-8") as proc_uptime:
                first_field = proc_uptime.read().split()[0]
                return _safe_float(first_field)
        except (FileNotFoundError, PermissionError, OSError, IndexError):
            return math.nan

    def _sample_swap_memory(self) -> tuple[float, float, float]:
        psutil = self._psutil
        if psutil is not None and hasattr(psutil, "swap_memory"):
            try:
                swap = psutil.swap_memory()
                total_mb = _safe_float(getattr(swap, "total", math.nan), scale=1.0 / (1024 * 1024))
                used_mb = _safe_float(getattr(swap, "used", math.nan), scale=1.0 / (1024 * 1024))
                percent = _safe_float(getattr(swap, "percent", math.nan))
                if math.isnan(percent) and not math.isnan(total_mb) and total_mb > 0.0 and not math.isnan(used_mb):
                    percent = max(0.0, min(100.0, (used_mb / total_mb) * 100.0))
                return percent, total_mb, used_mb
            except Exception:
                pass
        try:
            data: Dict[str, float] = {}
            with open("/proc/meminfo", "r", encoding="utf-8") as meminfo:
                for line in meminfo:
                    key, value = line.split(":", 1)
                    parts = value.strip().split()
                    if not parts:
                        continue
                    data[key.strip()] = float(parts[0])
            total_kb = data.get("SwapTotal")
            free_kb = data.get("SwapFree")
            if not total_kb or total_kb <= 0.0:
                return math.nan, math.nan, math.nan
            used_kb = total_kb - (free_kb or 0.0)
            total_mb = total_kb / 1024.0
            used_mb = used_kb / 1024.0
            percent = max(0.0, min(100.0, (used_kb / total_kb) * 100.0))
            return percent, total_mb, used_mb
        except (FileNotFoundError, PermissionError, OSError, ValueError):
            return math.nan, math.nan, math.nan

    def _sample_process_count(self) -> float:
        psutil = self._psutil
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
            return math.nan
        count = sum(1 for entry in entries if entry.isdigit())
        return float(count)


def host_shortname() -> str:
    """Return the short hostname for the current machine."""

    _, short = _resolve_host_identity()
    return short


def host_fullname() -> str:
    """Return the fully qualified hostname for the current machine."""

    host, _ = _resolve_host_identity()
    return host


def host_health_topic() -> str:
    """Return the canonical HostHealth topic path for this host."""

    return f"/hosts/health/{host_shortname()}"


__all__ = [
    "HostHealthSample",
    "HostHealthSampler",
    "host_health_topic",
    "host_fullname",
    "host_shortname",
]
