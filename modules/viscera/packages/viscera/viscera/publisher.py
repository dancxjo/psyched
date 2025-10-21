"""ROS 2 publisher for Pete's host health metrics."""
from __future__ import annotations

import argparse
import math
import sys
from typing import Iterable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args

from psyched_msgs.msg import HostHealth

from .host_health import HostHealthSample, HostHealthSampler, host_health_topic


def _sanitise(value: float) -> float:
    """Return *value* coerced to a finite float or NaN."""

    if value is None:
        return math.nan
    if not math.isfinite(value):
        return math.nan
    return float(value)


class HostHealthPublisher(Node):
    """Publish :class:`psyched_msgs.msg.HostHealth` samples on a fixed cadence."""

    def __init__(
        self,
        *,
        sampler: Optional[HostHealthSampler] = None,
        topic: Optional[str] = None,
        interval_sec: float = 5.0,
        frame_id: str = "",
    ) -> None:
        super().__init__("host_health_publisher")
        self._sampler = sampler or HostHealthSampler()
        self._topic = topic or host_health_topic()
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(HostHealth, self._topic, qos)
        self._timer = self.create_timer(max(0.1, interval_sec), self._publish_once)
        self._frame_id = frame_id
        self.get_logger().info("Publishing host health metrics on %s", self._topic)

    def _publish_once(self) -> None:
        sample = self._sampler.sample()
        msg = self._to_message(sample)
        self._publisher.publish(msg)

    def _to_message(self, sample: HostHealthSample) -> HostHealth:
        msg = HostHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id or sample.host_short
        msg.host = sample.host
        msg.host_short = sample.host_short
        msg.cpu_percent = _sanitise(sample.cpu_percent)
        msg.load_avg_1 = _sanitise(sample.load_avg_1)
        msg.load_avg_5 = _sanitise(sample.load_avg_5)
        msg.load_avg_15 = _sanitise(sample.load_avg_15)
        msg.mem_used_percent = _sanitise(sample.mem_used_percent)
        msg.mem_total_mb = _sanitise(sample.mem_total_mb)
        msg.mem_used_mb = _sanitise(sample.mem_used_mb)
        msg.disk_used_percent_root = _sanitise(sample.disk_used_percent_root)
        msg.temp_c = _sanitise(sample.temp_c)
        msg.uptime_sec = _sanitise(sample.uptime_sec)
        msg.swap_used_percent = _sanitise(sample.swap_used_percent)
        msg.swap_total_mb = _sanitise(sample.swap_total_mb)
        msg.swap_used_mb = _sanitise(sample.swap_used_mb)
        msg.process_count = _sanitise(sample.process_count)
        return msg


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish Pete host health metrics.")
    parser.add_argument(
        "--interval",
        type=float,
        default=5.0,
        help="Seconds between host health samples (default: 5.0).",
    )
    parser.add_argument(
        "--frame-id",
        default="",
        help="Frame ID attached to the HostHealth message header (default: '').",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="Override the published topic (default: /hosts/health/$HOSTNAME).",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    """Entry point for the ``viscera_host_health`` console script."""

    parsed_args = argv if argv is not None else sys.argv
    ros_args = remove_ros_args(list(parsed_args))
    parser = _build_parser()
    cli_args = parser.parse_args(ros_args[1:])

    rclpy.init(args=parsed_args)
    node = HostHealthPublisher(
        interval_sec=max(0.1, cli_args.interval),
        frame_id=cli_args.frame_id,
        topic=cli_args.topic or None,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - graceful shutdown for Ctrl+C.
        pass
    finally:
        node.destroy_timer(node._timer)
        node.destroy_node()
        rclpy.shutdown()
    return 0


__all__ = ["HostHealthPublisher", "main"]
