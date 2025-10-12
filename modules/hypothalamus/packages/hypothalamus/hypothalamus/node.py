"""ROS 2 node that exposes DHT-series temperature and humidity data."""

from __future__ import annotations

import contextlib
from dataclasses import dataclass
from typing import Optional, Protocol

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RelativeHumidity, Temperature
from std_msgs.msg import Float32, String

from .sensing import ThermalSample, simulated_samples

__all__ = ["ThermoregulationNode", "main"]


class SensorReadError(RuntimeError):
    """Raised when the physical sensor fails to provide a measurement."""


class SensorBackend(Protocol):
    """Protocol implemented by concrete temperature/humidity backends."""

    def sample(self, sequence: int) -> ThermalSample:
        """Return the next thermal reading."""


@dataclass(slots=True)
class SimulatedBackend:
    """Deterministic pseudo-random waveform used when hardware is absent."""

    seed: Optional[int] = None

    def __post_init__(self) -> None:
        self._generator = simulated_samples(seed=self.seed)

    def sample(self, sequence: int) -> ThermalSample:
        base = next(self._generator)
        return ThermalSample(
            temperature_celsius=base.temperature_celsius,
            humidity_percent=base.humidity_percent,
            sequence=sequence,
            source="simulated",
        )


class AdafruitDHTBackend:
    """Backend that reads from Adafruit's CircuitPython DHT implementation."""

    def __init__(self, *, sensor_type: str, gpio_pin: str, use_pulseio: bool = False) -> None:
        self._sensor_type = sensor_type.upper()
        self._gpio_pin = gpio_pin
        self._use_pulseio = use_pulseio
        self._sensor = self._create_sensor()

    def _create_sensor(self):  # pragma: no cover - exercised on hardware
        import adafruit_dht
        import board

        sensor_cls_map = {
            "DHT11": adafruit_dht.DHT11,
            "DHT22": adafruit_dht.DHT22,
            "AM2302": adafruit_dht.DHT22,
        }

        try:
            sensor_cls = sensor_cls_map[self._sensor_type]
        except KeyError as exc:
            raise SensorReadError(f"Unsupported DHT sensor type: {self._sensor_type}") from exc

        try:
            pin = getattr(board, self._gpio_pin)
        except AttributeError as exc:
            raise SensorReadError(f"Unknown GPIO pin label: {self._gpio_pin}") from exc

        return sensor_cls(pin, use_pulseio=self._use_pulseio)

    def sample(self, sequence: int) -> ThermalSample:
        try:  # pragma: no cover - hardware interaction
            temperature = self._sensor.temperature
            humidity = self._sensor.humidity
        except RuntimeError as exc:
            raise SensorReadError(str(exc)) from exc

        if temperature is None or humidity is None:
            raise SensorReadError("DHT sensor returned incomplete reading")

        return ThermalSample(
            temperature_celsius=float(temperature),
            humidity_percent=float(humidity),
            sequence=sequence,
            source=f"{self._sensor_type}@{self._gpio_pin}",
        )


class ThermoregulationNode(Node):
    """ROS 2 node that republishes thermal readings to well-known topics."""

    def __init__(self) -> None:
        super().__init__("hypothalamus")

        self.declare_parameter("sensor_type", "DHT11")
        self.declare_parameter("gpio_pin", "D4")
        self.declare_parameter("poll_interval", 2.5)
        self.declare_parameter("frame_id", "environment_link")
        self.declare_parameter("simulate_when_unavailable", True)
        self.declare_parameter("simulation_seed", -1)
        self.declare_parameter("temperature_topic", "/environment/temperature")
        self.declare_parameter("temperature_fahrenheit_topic", "/environment/temperature_fahrenheit")
        self.declare_parameter("humidity_topic", "/environment/humidity")
        self.declare_parameter("humidity_percent_topic", "/environment/humidity_percent")
        self.declare_parameter("status_topic", "/environment/thermostat_status")

        self._sequence = 0
        self._simulate_on_error = bool(self.get_parameter("simulate_when_unavailable").value)
        self._simulation_seed = self._seed_from_parameter()
        self._simulated_backend: Optional[SimulatedBackend] = None
        self._backend = self._create_backend()

        self._temperature_pub = self.create_publisher(Temperature, self._param_value("temperature_topic"), 10)
        self._fahrenheit_pub = self.create_publisher(Float32, self._param_value("temperature_fahrenheit_topic"), 10)
        self._humidity_pub = self.create_publisher(RelativeHumidity, self._param_value("humidity_topic"), 10)
        self._humidity_percent_pub = self.create_publisher(Float32, self._param_value("humidity_percent_topic"), 10)
        self._status_pub = self.create_publisher(String, self._param_value("status_topic"), 10)

        poll_interval = float(self.get_parameter("poll_interval").value)
        self._timer = self.create_timer(poll_interval, self._poll_once)

        self.get_logger().info(
            "hypothalamus thermal node ready: backend=%s poll_interval=%.2fs", self._backend.__class__.__name__, poll_interval
        )

    def _param_value(self, name: str) -> str:
        value = self.get_parameter(name).value
        if not isinstance(value, str):
            raise TypeError(f"Expected parameter '{name}' to be a string, received {type(value)!r}")
        return value

    def _seed_from_parameter(self) -> Optional[int]:
        raw = self.get_parameter("simulation_seed").value
        if isinstance(raw, int) and raw >= 0:
            return raw
        return None

    def _create_backend(self) -> SensorBackend:
        sensor_type = str(self.get_parameter("sensor_type").value)
        gpio_pin = str(self.get_parameter("gpio_pin").value)

        try:
            backend: SensorBackend = AdafruitDHTBackend(sensor_type=sensor_type, gpio_pin=gpio_pin)
        except Exception as exc:  # pragma: no cover - executed when hardware libs missing
            self.get_logger().warning(
                "Falling back to simulated DHT backend: %s", exc, throttle_duration_sec=30.0
            )
            backend = self._ensure_simulated_backend()
        return backend

    def _ensure_simulated_backend(self) -> SimulatedBackend:
        if self._simulated_backend is None:
            self._simulated_backend = SimulatedBackend(seed=self._simulation_seed)
        return self._simulated_backend

    def _poll_once(self) -> None:
        try:
            sample = self._backend.sample(self._sequence)
        except SensorReadError as exc:
            self.get_logger().warning("Sensor read failed: %s", exc, throttle_duration_sec=30.0)
            if not self._simulate_on_error:
                return
            sample = self._ensure_simulated_backend().sample(self._sequence)
        except Exception as exc:  # pragma: no cover - defensive guard
            self.get_logger().exception("Unexpected sensor failure: %s", exc)
            if not self._simulate_on_error:
                return
            sample = self._ensure_simulated_backend().sample(self._sequence)

        self._sequence += 1
        self._publish_sample(sample)

    def _publish_sample(self, sample: ThermalSample) -> None:
        now = self.get_clock().now().to_msg()
        frame_id = str(self.get_parameter("frame_id").value)

        temp_msg = Temperature()
        temp_msg.header.stamp = now
        temp_msg.header.frame_id = frame_id
        temp_msg.temperature = float(sample.temperature_celsius)
        temp_msg.variance = float("nan")

        humidity_msg = RelativeHumidity()
        humidity_msg.header = temp_msg.header
        humidity_msg.relative_humidity = max(0.0, min(1.0, float(sample.humidity_percent) / 100.0))
        humidity_msg.variance = float("nan")

        fahrenheit_msg = Float32()
        fahrenheit_msg.data = float(sample.fahrenheit)

        humidity_percent_msg = Float32()
        humidity_percent_msg.data = float(sample.humidity_percent)

        status_msg = String()
        status_msg.data = sample.source or "unknown"

        self._temperature_pub.publish(temp_msg)
        self._humidity_pub.publish(humidity_msg)
        self._fahrenheit_pub.publish(fahrenheit_msg)
        self._humidity_percent_pub.publish(humidity_percent_msg)
        self._status_pub.publish(status_msg)

    def destroy_node(self) -> None:  # pragma: no cover - lifecycle cleanup
        with contextlib.suppress(Exception):
            super().destroy_node()
        with contextlib.suppress(Exception):
            if hasattr(self._backend, "close"):
                self._backend.close()  # type: ignore[attr-defined]


def main(args: Optional[list[str]] = None) -> None:  # pragma: no cover - ROS 2 entrypoint
    rclpy.init(args=args)
    node = ThermoregulationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
