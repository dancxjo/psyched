"""Behaviour driven tests for the viscera emotional mapper."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import List

from viscera.feelers import (
    hunger_feeler,
    load_feeler,
    stability_feeler,
    temperature_feeler,
    uptime_feeler,
    swap_feeler,
)
from viscera.monitor import SystemMetricsProbe, Viscera
from viscera.state import BatteryState, FootState, SystemState


def make_state(**kwargs) -> SystemState:
    """Helper to build a :class:`~viscera.state.SystemState` with sane defaults."""
    defaults = dict(
        timestamp=datetime.now(timezone.utc),
        battery=BatteryState(charge_fraction=0.5, is_charging=False),
        cpu_load=0.2,
        memory_load=0.2,
        disk_fill_level=0.2,
        foot=None,
        swap_fraction=None,
        uptime_sec=None,
        temperature_c=None,
        process_count=None,
    )
    defaults.update(kwargs)
    return SystemState(**defaults)


class TestHungerFeeler:
    """Scenarios for the hunger feeler lifecycle."""

    def test_appetite_grows_before_fatigue(self) -> None:
        """Battery charge maps to nuanced feelings rather than binary alerts."""
        state = make_state(battery=BatteryState(charge_fraction=0.62, is_charging=False))
        sentiments = hunger_feeler(state)
        assert any(
            "appetite" in sentiment.narrative.lower()
            for sentiment in sentiments
        ), sentiments

    def test_panicky_phase_before_lethargy(self) -> None:
        state = make_state(battery=BatteryState(charge_fraction=0.28, is_charging=False))
        sentiments = hunger_feeler(state)
        assert any(
            "weak and a bit panicky" in sentiment.narrative.lower()
            for sentiment in sentiments
        ), sentiments

    def test_lethargy_when_energy_is_critical(self) -> None:
        state = make_state(battery=BatteryState(charge_fraction=0.08, is_charging=False))
        sentiments = hunger_feeler(state)
        assert any(
            "lethargic" in sentiment.narrative.lower()
            for sentiment in sentiments
        ), sentiments


class TestLoadFeeler:
    """Stress mapping for computational resource pressure."""

    def test_disk_bloat_feels_nauseous(self) -> None:
        state = make_state(disk_fill_level=0.95)
        sentiments = load_feeler(state)
        assert any(
            "nauseous" in sentiment.narrative.lower()
            for sentiment in sentiments
        ), sentiments


class TestStabilityFeeler:
    """Foot telemetry influences balance-related feelings."""

    def test_unstable_footing_creates_wobbliness(self) -> None:
        foot_state = FootState(hazard_contacts=2, is_slipping=True, contact_confidence=0.2)
        state = make_state(foot=foot_state)
        sentiments = stability_feeler(state)
        assert any(
            "wobbly" in sentiment.narrative.lower()
            for sentiment in sentiments
        ), sentiments


class TestTemperatureFeeler:
    """Thermal readings influence comfort."""

    def test_overheated_when_too_warm(self) -> None:
        state = make_state(temperature_c=86.0)
        sentiments = temperature_feeler(state)
        assert any("scorched" in sentiment.narrative.lower() for sentiment in sentiments), sentiments


class TestUptimeFeeler:
    """Reboots and long runs influence restfulness."""

    def test_recent_reboot_feels_refreshed(self) -> None:
        state = make_state(uptime_sec=2 * 3600.0)
        sentiments = uptime_feeler(state)
        assert any("refreshed" in sentiment.narrative.lower() for sentiment in sentiments), sentiments


class TestSwapFeeler:
    """Swap pressure nudges the robot's mood."""

    def test_heavy_swap_feels_swamped(self) -> None:
        state = make_state(swap_fraction=0.9)
        sentiments = swap_feeler(state)
        assert any("swamped" in sentiment.narrative.lower() for sentiment in sentiments), sentiments


class TestVisceraCoordinator:
    """Integration scenarios for the high level coordinator."""

    def test_feelings_are_sorted_by_intensity(self) -> None:
        state = make_state(
            battery=BatteryState(charge_fraction=0.12, is_charging=False),
            disk_fill_level=0.93,
        )
        viscera = Viscera(feelers=[hunger_feeler, load_feeler])
        sentiments = viscera.feelings(state)
        assert sentiments == sorted(sentiments, key=lambda s: s.intensity, reverse=True)

    def test_narrative_projection(self) -> None:
        state = make_state(
            battery=BatteryState(charge_fraction=0.12, is_charging=False),
            disk_fill_level=0.93,
        )
        viscera = Viscera(feelers=[hunger_feeler, load_feeler])
        narrative: List[str] = viscera.narrate(state)
        assert all(sentence.endswith(".") for sentence in narrative)
        assert any("nauseous" in sentence.lower() for sentence in narrative)


class TestSystemMetricsProbe:
    """Ensure we can sample metrics without psutil being present."""

    def test_fallback_sampling(self) -> None:
        probe = SystemMetricsProbe(psutil_module=None, foot_state_provider=lambda: FootState(hazard_contacts=1))
        snapshot = probe.sample()
        assert 0.0 <= (snapshot.cpu_load or 0.0) <= 1.0
        assert 0.0 <= (snapshot.memory_load or 0.0) <= 1.0
        assert 0.0 <= (snapshot.disk_fill_level or 0.0) <= 1.0
        if snapshot.swap_fraction is not None:
            assert 0.0 <= snapshot.swap_fraction <= 1.0
        if snapshot.uptime_sec is not None:
            assert snapshot.uptime_sec >= 0.0
        if snapshot.process_count is not None:
            assert snapshot.process_count >= 0.0
        assert snapshot.foot is not None
        assert snapshot.foot.hazard_contacts == 1
