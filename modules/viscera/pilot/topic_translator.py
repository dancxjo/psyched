"""Natural language translators for viscera host health topics."""

from __future__ import annotations

import math
from typing import Any, Mapping


def _is_finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _format_percent(value: Any) -> str | None:
    if _is_finite(value):
        return f"{float(value):.0f}%"
    return None


def _format_load(payload: Mapping[str, Any]) -> str | None:
    loads = []
    for key in ("load_avg_1", "load_avg_5", "load_avg_15"):
        value = payload.get(key)
        if _is_finite(value):
            loads.append(f"{float(value):.2f}")
    if loads:
        return "/".join(loads)
    return None


def _format_gib(value: Any) -> str | None:
    if _is_finite(value):
        gib = float(value) / 1024.0
        return f"{gib:.1f} GiB" if gib < 10 else f"{gib:.0f} GiB"
    return None


def _format_memory(payload: Mapping[str, Any]) -> str | None:
    percent = _format_percent(payload.get("mem_used_percent"))
    used = _format_gib(payload.get("mem_used_mb"))
    total = _format_gib(payload.get("mem_total_mb"))
    parts: list[str] = []
    if percent:
        parts.append(percent)
    span: str | None = None
    if used and total:
        span = f"{used}/{total}"
    elif used:
        span = used
    elif total:
        span = f"{total} total"
    if span:
        if parts:
            parts.append(f"({span})")
        else:
            parts.append(span)
    if parts:
        return "memory " + " ".join(parts)
    return None


def _format_temperature(value: Any) -> str | None:
    if _is_finite(value):
        return f"{float(value):.0f}°C"
    return None


def _format_uptime(value: Any) -> str | None:
    if not _is_finite(value):
        return None
    seconds = int(float(value))
    if seconds <= 0:
        return None
    days, rem = divmod(seconds, 86_400)
    hours, rem = divmod(rem, 3_600)
    minutes, _ = divmod(rem, 60)
    parts: list[str] = []
    if days:
        parts.append(f"{days}d")
    if hours:
        parts.append(f"{hours}h")
    if minutes and len(parts) < 2:
        parts.append(f"{minutes}m")
    if not parts:
        parts.append(f"{seconds}s")
    return " ".join(parts)


def _format_processes(value: Any) -> str | None:
    if _is_finite(value):
        count = int(float(value))
        if count > 0:
            return f"{count} processes"
    return None


def summarise_host_health(payload: Any) -> str:
    """Return a human-friendly summary for ``psyched_msgs/msg/HostHealth``.

    Examples
    --------
    >>> summarise_host_health({"host_short": "pete"})
    'pete vitals: awaiting metrics.'
    >>> summarise_host_health({
    ...     "host_short": "pete",
    ...     "cpu_percent": 42.0,
    ...     "mem_used_percent": 65.0,
    ...     "mem_total_mb": 8192.0,
    ...     "mem_used_mb": 4096.0,
    ... })
    'pete vitals: CPU 42%, memory 65% (4.0/8.0 GiB).'
    """

    if not isinstance(payload, Mapping):
        return "Host vitals: awaiting metrics."
    host = str(payload.get("host_short") or payload.get("host") or "host").strip() or "host"
    parts: list[str] = []
    cpu = _format_percent(payload.get("cpu_percent"))
    if cpu:
        parts.append(f"CPU {cpu}")
    load = _format_load(payload)
    if load:
        parts.append(f"load {load}")
    memory = _format_memory(payload)
    if memory:
        parts.append(memory)
    disk = _format_percent(payload.get("disk_used_percent_root"))
    if disk:
        parts.append(f"disk {disk}")
    temp = _format_temperature(payload.get("temp_c"))
    if temp:
        parts.append(f"temp {temp}")
    uptime = _format_uptime(payload.get("uptime_sec"))
    if uptime:
        parts.append(f"uptime {uptime}")
    processes = _format_processes(payload.get("process_count"))
    if processes:
        parts.append(processes)
    if not parts:
        return f"{host} vitals: awaiting metrics."
    return f"{host} vitals: {', '.join(parts)}."


TOPIC_TRANSLATORS = {
    "/hosts/health/*": summarise_host_health,
}

STATIC_PROMPT_SECTIONS = [
    (
        "Viscera monitors host vitals via /hosts/health/{HOST_SHORT}. It "
        "summarises CPU, load, memory, disk, temperature, uptime, and process "
        "counts as fresh metrics arrive. Read these vitals as rolling instant "
        "snapshots—older measurements drop away once they fall outside the "
        "window."
    )
]
