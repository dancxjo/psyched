"""Tests for the host health sampler."""
from __future__ import annotations

import math

from viscera.host_health import HostHealthSampler, host_health_topic, host_shortname


def test_sampler_without_psutil() -> None:
    """Ensure fallback sampling works when psutil is unavailable."""

    sampler = HostHealthSampler(psutil_module=None)
    sample = sampler.sample()
    assert isinstance(sample.cpu_percent, float)
    assert isinstance(sample.mem_used_percent, float)
    # Disk usage should either be a float percentage or NaN on exotic filesystems.
    assert isinstance(sample.disk_used_percent_root, float)
    # Uptime should be non-negative when available.
    assert math.isnan(sample.uptime_sec) or sample.uptime_sec >= 0.0


class _FakeVm:
    percent = 42.0
    total = 8 * 1024**3
    used = 3 * 1024**3
    available = total - used


class _FakeDisk:
    total = 100
    used = 40


class _FakeTemp:
    current = 55.5


class _FakePsutil:
    def __init__(self) -> None:
        self._cpu_calls = 0

    def cpu_percent(self, interval=None):
        self._cpu_calls += 1
        return 66.6 if self._cpu_calls > 1 else 0.0

    def virtual_memory(self):
        return _FakeVm()

    def disk_usage(self, mount_point):
        return _FakeDisk()

    def sensors_temperatures(self):
        return {"cpu": [_FakeTemp()]}

    def boot_time(self):
        return 100.0


def test_sampler_with_psutil_stub() -> None:
    """Verify that psutil integrations populate deterministic values."""

    fake_psutil = _FakePsutil()
    sampler = HostHealthSampler(psutil_module=fake_psutil, clock=lambda: 200.0)
    sample = sampler.sample()

    assert math.isclose(sample.cpu_percent, 66.6, rel_tol=1e-3)
    assert math.isclose(sample.mem_used_percent, 42.0, rel_tol=1e-3)
    assert math.isclose(sample.mem_total_mb, 8192.0, rel_tol=1e-3)
    assert math.isclose(sample.mem_used_mb, 3072.0, rel_tol=1e-3)
    assert math.isclose(sample.disk_used_percent_root, 40.0, rel_tol=1e-3)
    assert math.isclose(sample.temp_c, 55.5, rel_tol=1e-3)
    assert math.isclose(sample.uptime_sec, 100.0, rel_tol=1e-3)


def test_host_topic_helpers() -> None:
    """The topic helper should always include the short hostname."""

    shortname = host_shortname()
    assert shortname
    topic = host_health_topic()
    assert topic.endswith(shortname)
