from __future__ import annotations

import unittest
from itertools import count

from pilot_cockpit.foot import (
    FootTelemetryStateMachine,
    TwistSummary,
    charging_state_label,
    mode_label,
)


def monotonic_now(start: int = 0):
    counter = count(start)

    def _now() -> int:
        return next(counter)

    return _now


class FootStateMachineTests(unittest.TestCase):
    def test_charge_ratio_updates_percentage_and_snapshot(self) -> None:
        machine = FootTelemetryStateMachine(now=monotonic_now(1))
        events = machine.update_charge_ratio(0.5)
        self.assertEqual(events[0].topic, FootTelemetryStateMachine.TELEMETRY_TOPIC)
        payload = events[0].payload
        self.assertAlmostEqual(payload["battery"]["percentage"], 50.0)
        self.assertAlmostEqual(payload["battery"]["charge_ratio"], 0.5)
        self.assertEqual(payload["last_update_ms"], 1)

    def test_handle_subscribe_returns_current_snapshot(self) -> None:
        machine = FootTelemetryStateMachine(now=monotonic_now(100))
        empty = machine.handle_subscribe(FootTelemetryStateMachine.TELEMETRY_TOPIC)
        self.assertEqual(empty[0].payload, {})

        machine.update_voltage(13.7)
        snapshot = machine.handle_subscribe(FootTelemetryStateMachine.TELEMETRY_TOPIC)[0].payload
        self.assertAlmostEqual(snapshot["battery"]["voltage_v"], 13.7)

    def test_record_command_emits_cmd_vel_and_snapshot(self) -> None:
        machine = FootTelemetryStateMachine(now=monotonic_now(500))
        summary = TwistSummary(
            linear_x=0.4,
            linear_y=-0.1,
            linear_z=0.0,
            angular_x=0.0,
            angular_y=0.0,
            angular_z=0.75,
        )
        events = machine.record_command(summary)
        self.assertEqual(
            [event.topic for event in events],
            [FootTelemetryStateMachine.CMD_VEL_TOPIC, FootTelemetryStateMachine.TELEMETRY_TOPIC],
        )
        cmd_payload = events[0].payload
        self.assertAlmostEqual(cmd_payload["linear"]["x"], 0.4)
        telemetry = events[1].payload
        self.assertAlmostEqual(telemetry["motion"]["command"]["linear"]["x"], 0.4)
        self.assertEqual(telemetry["status"]["last_command_ms"], 500)

    def test_hazard_updates_include_bumpers_and_cliffs(self) -> None:
        machine = FootTelemetryStateMachine(now=monotonic_now(10))
        machine.update_bumper(True, False, True, False, False, True)
        events = machine.update_cliff(False, True, True, False)
        hazards = events[0].payload["hazards"]
        self.assertTrue(hazards["bumper_left"])
        self.assertTrue(hazards["bumper_light_front"])
        self.assertTrue(hazards["bumper_light_center"])
        self.assertTrue(hazards["cliff_front_left"])
        self.assertTrue(hazards["cliff_right"])

    def test_label_helpers_cover_known_values(self) -> None:
        self.assertEqual(charging_state_label(0), "Idle")
        self.assertEqual(charging_state_label(5), "Fault")
        self.assertEqual(charging_state_label(99), "Unknown")
        self.assertEqual(mode_label(2), "Safe")
        self.assertEqual(mode_label(42), "Unknown")


if __name__ == "__main__":
    unittest.main()
