"""Foot telemetry aggregation for the cockpit websocket bridge."""
from __future__ import annotations

import copy
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Set

from .protocol import OutboundMessage, make_message


_DEFAULT_CHARGING_LABELS: Dict[int, str] = {
    0: "Idle",
    1: "Recondition",
    2: "Full",
    3: "Trickle",
    4: "Waiting",
    5: "Fault",
}
_DEFAULT_CHARGING_ACTIVE: Set[int] = {1, 2, 3, 4}
_DEFAULT_MODE_LABELS: Dict[int, str] = {
    0: "Off",
    1: "Passive",
    2: "Safe",
    3: "Full",
}


@dataclass
class TwistSummary:
    """Compact representation of a twist for websocket broadcasts."""

    linear_x: float
    linear_y: float
    linear_z: float
    angular_x: float
    angular_y: float
    angular_z: float

    def to_dict(self) -> Dict[str, Dict[str, float]]:
        return {
            "linear": {
                "x": self.linear_x,
                "y": self.linear_y,
                "z": self.linear_z,
            },
            "angular": {
                "x": self.angular_x,
                "y": self.angular_y,
                "z": self.angular_z,
            },
        }


@dataclass
class BatterySnapshot:
    percentage: Optional[float] = None
    charge_ratio: Optional[float] = None
    charge_ah: Optional[float] = None
    capacity_ah: Optional[float] = None
    current_a: Optional[float] = None
    voltage_v: Optional[float] = None
    temperature_c: Optional[float] = None
    charging_state: Optional[str] = None
    is_charging: Optional[bool] = None

    def to_dict(self) -> Dict[str, object]:
        return {k: v for k, v in self.__dict__.items() if v is not None}


@dataclass
class StatusSnapshot:
    mode: Optional[str] = None
    mode_code: Optional[int] = None
    charging_state: Optional[str] = None
    charging_state_code: Optional[int] = None
    last_command_ms: Optional[int] = None

    def to_dict(self) -> Dict[str, object]:
        return {k: v for k, v in self.__dict__.items() if v is not None}


@dataclass
class MotionSnapshot:
    command: Optional[TwistSummary] = None
    odometry: Optional[TwistSummary] = None

    def to_dict(self) -> Dict[str, object]:
        data: Dict[str, object] = {}
        if self.command is not None:
            data["command"] = self.command.to_dict()
        if self.odometry is not None:
            data["odometry"] = self.odometry.to_dict()
        return data


@dataclass
class HazardSnapshot:
    bumper_left: Optional[bool] = None
    bumper_right: Optional[bool] = None
    bumper_light_front: Optional[bool] = None
    bumper_light_center: Optional[bool] = None
    cliff_left: Optional[bool] = None
    cliff_front_left: Optional[bool] = None
    cliff_right: Optional[bool] = None
    cliff_front_right: Optional[bool] = None

    def to_dict(self) -> Dict[str, object]:
        return {k: v for k, v in self.__dict__.items() if v is not None}


@dataclass
class FootTelemetrySnapshot:
    battery: Optional[BatterySnapshot] = None
    status: Optional[StatusSnapshot] = None
    motion: Optional[MotionSnapshot] = None
    hazards: Optional[HazardSnapshot] = None
    last_update_ms: Optional[int] = None

    def to_dict(self) -> Dict[str, object]:
        data: Dict[str, object] = {}
        if self.battery:
            battery = self.battery.to_dict()
            if battery:
                data["battery"] = battery
        if self.status:
            status = self.status.to_dict()
            if status:
                data["status"] = status
        if self.motion:
            motion = self.motion.to_dict()
            if motion:
                data["motion"] = motion
        if self.hazards:
            hazards = self.hazards.to_dict()
            if hazards:
                data["hazards"] = hazards
        if self.last_update_ms is not None:
            data["last_update_ms"] = self.last_update_ms
        return data

    def clone(self) -> "FootTelemetrySnapshot":
        return copy.deepcopy(self)


@dataclass
class FootEvent:
    topic: str
    payload: Dict[str, object]


class FootTelemetryStateMachine:
    """Pure state machine for Foot telemetry fan-out."""

    TELEMETRY_TOPIC = "/foot/telemetry"
    CMD_VEL_TOPIC = "/cmd_vel"

    def __init__(
        self,
        now: Optional[Callable[[], int]] = None,
        *,
        charging_labels: Optional[Dict[int, str]] = None,
        charging_active_states: Optional[Set[int]] = None,
        mode_labels: Optional[Dict[int, str]] = None,
    ) -> None:
        self._snapshot = FootTelemetrySnapshot()
        self._now = now or (lambda: int(time.time() * 1000))
        self._charging_labels = charging_labels or _DEFAULT_CHARGING_LABELS
        self._charging_active = charging_active_states or _DEFAULT_CHARGING_ACTIVE
        self._mode_labels = mode_labels or _DEFAULT_MODE_LABELS

    def snapshot(self) -> FootTelemetrySnapshot:
        return self._snapshot.clone()

    def handle_subscribe(self, topic: str) -> List[FootEvent]:
        if topic == self.TELEMETRY_TOPIC:
            return [FootEvent(topic=topic, payload=self.snapshot().to_dict())]
        if topic == self.CMD_VEL_TOPIC:
            summary = self._snapshot.motion.command if self._snapshot.motion else None
            if summary is None:
                return []
            return [FootEvent(topic=topic, payload=summary.to_dict())]
        return []

    def update_charge_ratio(self, ratio: float) -> List[FootEvent]:
        bounded = max(0.0, min(1.0, ratio))
        battery = self._ensure_battery()
        battery.charge_ratio = bounded
        battery.percentage = bounded * 100.0
        return self._touch()

    def update_capacity(self, capacity: float) -> List[FootEvent]:
        self._ensure_battery().capacity_ah = capacity
        return self._touch()

    def update_charge(self, charge: float) -> List[FootEvent]:
        self._ensure_battery().charge_ah = max(0.0, charge)
        return self._touch()

    def update_current(self, current: float) -> List[FootEvent]:
        battery = self._ensure_battery()
        battery.current_a = current
        battery.is_charging = current > 0.0
        return self._touch()

    def update_voltage(self, voltage: float) -> List[FootEvent]:
        self._ensure_battery().voltage_v = voltage
        return self._touch()

    def update_temperature(self, temperature: float) -> List[FootEvent]:
        self._ensure_battery().temperature_c = temperature
        return self._touch()

    def update_charging_state(self, state: int) -> List[FootEvent]:
        label = charging_state_label(state, labels=self._charging_labels)
        battery = self._ensure_battery()
        battery.charging_state = label
        battery.is_charging = state in self._charging_active
        status = self._ensure_status()
        status.charging_state = label
        status.charging_state_code = state
        return self._touch()

    def update_mode(self, mode: int) -> List[FootEvent]:
        status = self._ensure_status()
        status.mode_code = mode
        status.mode = mode_label(mode, labels=self._mode_labels)
        return self._touch()

    def update_bumper(
        self,
        is_left_pressed: bool,
        is_right_pressed: bool,
        is_light_front_left: bool,
        is_light_front_right: bool,
        is_light_center_left: bool,
        is_light_center_right: bool,
    ) -> List[FootEvent]:
        hazards = self._ensure_hazards()
        hazards.bumper_left = is_left_pressed
        hazards.bumper_right = is_right_pressed
        hazards.bumper_light_front = is_light_front_left or is_light_front_right
        hazards.bumper_light_center = is_light_center_left or is_light_center_right
        return self._touch()

    def update_cliff(
        self,
        is_cliff_left: bool,
        is_cliff_front_left: bool,
        is_cliff_right: bool,
        is_cliff_front_right: bool,
    ) -> List[FootEvent]:
        hazards = self._ensure_hazards()
        hazards.cliff_left = is_cliff_left
        hazards.cliff_front_left = is_cliff_front_left
        hazards.cliff_right = is_cliff_right
        hazards.cliff_front_right = is_cliff_front_right
        return self._touch()

    def update_odometry(self, summary: TwistSummary) -> List[FootEvent]:
        self._ensure_motion().odometry = summary
        return self._touch()

    def record_command(self, summary: TwistSummary) -> List[FootEvent]:
        motion = self._ensure_motion()
        motion.command = summary
        status = self._ensure_status()
        now = self._now()
        status.last_command_ms = now
        self._snapshot.last_update_ms = now
        telemetry = FootEvent(topic=self.TELEMETRY_TOPIC, payload=self.snapshot().to_dict())
        cmd = FootEvent(topic=self.CMD_VEL_TOPIC, payload=summary.to_dict())
        return [cmd, telemetry]

    def _touch(self) -> List[FootEvent]:
        self._snapshot.last_update_ms = self._now()
        return [FootEvent(topic=self.TELEMETRY_TOPIC, payload=self.snapshot().to_dict())]

    def _ensure_battery(self) -> BatterySnapshot:
        if self._snapshot.battery is None:
            self._snapshot.battery = BatterySnapshot()
        return self._snapshot.battery

    def _ensure_status(self) -> StatusSnapshot:
        if self._snapshot.status is None:
            self._snapshot.status = StatusSnapshot()
        return self._snapshot.status

    def _ensure_motion(self) -> MotionSnapshot:
        if self._snapshot.motion is None:
            self._snapshot.motion = MotionSnapshot()
        return self._snapshot.motion

    def _ensure_hazards(self) -> HazardSnapshot:
        if self._snapshot.hazards is None:
            self._snapshot.hazards = HazardSnapshot()
        return self._snapshot.hazards


def charging_state_label(state: int, *, labels: Optional[Dict[int, str]] = None) -> str:
    table = labels or _DEFAULT_CHARGING_LABELS
    return table.get(state, "Unknown")


def mode_label(mode: int, *, labels: Optional[Dict[int, str]] = None) -> str:
    table = labels or _DEFAULT_MODE_LABELS
    return table.get(mode, "Unknown")


def summary_from_twist(twist) -> TwistSummary:
    return TwistSummary(
        linear_x=float(twist.linear.x),
        linear_y=float(twist.linear.y),
        linear_z=float(twist.linear.z),
        angular_x=float(twist.angular.x),
        angular_y=float(twist.angular.y),
        angular_z=float(twist.angular.z),
    )


def summary_from_odometry(odometry) -> TwistSummary:
    twist = odometry.twist.twist
    return summary_from_twist(twist)


class FootTelemetryBridge:
    """ROS-aware facade that feeds :class:`FootTelemetryStateMachine`."""

    def __init__(self, node, broadcast: Callable[[OutboundMessage], None], now: Optional[Callable[[], int]] = None) -> None:
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32, Int16

        self._node = node
        self._broadcast = broadcast
        clock_now = now or self._clock_millis
        logger = node.get_logger() if hasattr(node, "get_logger") else None
        charging_labels: Dict[int, str] = dict(_DEFAULT_CHARGING_LABELS)
        charging_active: Set[int] = set(_DEFAULT_CHARGING_ACTIVE)
        mode_labels: Dict[int, str] = dict(_DEFAULT_MODE_LABELS)
        create_msgs_available = True
        try:
            from create_msgs.msg import Bumper, ChargingState, Cliff, Mode  # type: ignore[import-not-found]
        except ModuleNotFoundError:
            create_msgs_available = False
            if logger is not None:
                logger.warning(
                    "create_msgs package missing; foot telemetry will skip bumper, cliff, charging state, and mode topics."
                )
        else:
            charging_labels = {
                int(ChargingState.CHARGE_NONE): "Idle",
                int(ChargingState.CHARGE_RECONDITION): "Recondition",
                int(ChargingState.CHARGE_FULL): "Full",
                int(ChargingState.CHARGE_TRICKLE): "Trickle",
                int(ChargingState.CHARGE_WAITING): "Waiting",
                int(ChargingState.CHARGE_FAULT): "Fault",
            }
            charging_active = {
                int(ChargingState.CHARGE_RECONDITION),
                int(ChargingState.CHARGE_FULL),
                int(ChargingState.CHARGE_TRICKLE),
                int(ChargingState.CHARGE_WAITING),
            }
            mode_labels = {
                int(Mode.MODE_OFF): "Off",
                int(Mode.MODE_PASSIVE): "Passive",
                int(Mode.MODE_SAFE): "Safe",
                int(Mode.MODE_FULL): "Full",
            }
        self._state = FootTelemetryStateMachine(
            now=clock_now,
            charging_labels=charging_labels,
            charging_active_states=charging_active,
            mode_labels=mode_labels,
        )
        qos = 10
        self._subscriptions = [
            node.create_subscription(Float32, "/battery/charge_ratio", lambda msg: self._dispatch(self._state.update_charge_ratio, float(msg.data)), qos),
            node.create_subscription(Float32, "/battery/capacity", lambda msg: self._dispatch(self._state.update_capacity, float(msg.data)), qos),
            node.create_subscription(Float32, "/battery/charge", lambda msg: self._dispatch(self._state.update_charge, float(msg.data)), qos),
            node.create_subscription(Float32, "/battery/current", lambda msg: self._dispatch(self._state.update_current, float(msg.data)), qos),
            node.create_subscription(Float32, "/battery/voltage", lambda msg: self._dispatch(self._state.update_voltage, float(msg.data)), qos),
            node.create_subscription(Int16, "/battery/temperature", lambda msg: self._dispatch(self._state.update_temperature, float(msg.data)), qos),
            node.create_subscription(Twist, "/cmd_vel", lambda msg: self._dispatch(self._state.record_command, summary_from_twist(msg)), qos),
            node.create_subscription(Odometry, "/odom", lambda msg: self._dispatch(self._state.update_odometry, summary_from_odometry(msg)), qos),
        ]
        if create_msgs_available:
            self._subscriptions.extend(
                [
                    node.create_subscription(ChargingState, "/battery/charging_state", lambda msg: self._dispatch(self._state.update_charging_state, int(msg.state)), qos),
                    node.create_subscription(Mode, "/mode", lambda msg: self._dispatch(self._state.update_mode, int(msg.mode)), qos),
                    node.create_subscription(
                        Bumper,
                        "/bumper",
                        lambda msg: self._dispatch(
                            self._state.update_bumper,
                            bool(msg.is_left_pressed),
                            bool(msg.is_right_pressed),
                            bool(msg.is_light_front_left),
                            bool(msg.is_light_front_right),
                            bool(msg.is_light_center_left),
                            bool(msg.is_light_center_right),
                        ),
                        qos,
                    ),
                    node.create_subscription(
                        Cliff,
                        "/cliff",
                        lambda msg: self._dispatch(
                            self._state.update_cliff,
                            bool(msg.is_cliff_left),
                            bool(msg.is_cliff_front_left),
                            bool(msg.is_cliff_right),
                            bool(msg.is_cliff_front_right),
                        ),
                        qos,
                    ),
                ]
            )

    @staticmethod
    def handles_topic(topic: str) -> bool:
        return topic in {FootTelemetryStateMachine.TELEMETRY_TOPIC, FootTelemetryStateMachine.CMD_VEL_TOPIC}

    def handle_subscribe(self, topic: str) -> List[OutboundMessage]:
        events = self._state.handle_subscribe(topic)
        return [make_message(event.topic, event.payload) for event in events]

    def handle_unsubscribe(self, _topic: str) -> None:
        return None

    def record_command(self, twist) -> None:
        summary = summary_from_twist(twist)
        events = self._state.record_command(summary)
        self._send_events(events)

    def _dispatch(self, handler: Callable[..., List[FootEvent]], *args) -> None:
        events = handler(*args)
        self._send_events(events)

    def _send_events(self, events: List[FootEvent]) -> None:
        for event in events:
            self._broadcast(make_message(event.topic, event.payload))

    def _clock_millis(self) -> int:
        return int(self._node.get_clock().now().nanoseconds / 1_000_000)
