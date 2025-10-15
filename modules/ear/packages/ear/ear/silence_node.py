"""ROS 2 node that emits silence state based on PCM audio energy."""

from __future__ import annotations

import audioop
from collections import deque
from typing import Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, UInt8MultiArray


class SilenceDetectorNode(Node):
    """Classify audio frames as silence using an RMS threshold."""

    def __init__(self) -> None:
        super().__init__("ear_silence_detector")
        self._audio_topic = str(self.declare_parameter("audio_topic", "/audio/raw").value)
        self._silence_topic = str(self.declare_parameter("silence_topic", "/ear/silence").value)
        self._sample_width = int(self.declare_parameter("sample_width", 2).value)
        self._threshold = float(self.declare_parameter("rms_threshold", 500.0).value)
        window_param = int(self.declare_parameter("average_window", 3).value)
        self._average_window = max(window_param, 1)
        self._publish_on_change = bool(self.declare_parameter("publish_on_change", True).value)
        reliability_param = str(self.declare_parameter("reliability", "reliable").value).strip().lower()
        self._recent: deque[float] = deque(maxlen=self._average_window)
        self._last_state: bool | None = None
        qos_profile = QoSProfile(depth=20)
        if reliability_param == "best_effort":
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        else:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self._publisher = self.create_publisher(Bool, self._silence_topic, qos_profile)
        self.create_subscription(UInt8MultiArray, self._audio_topic, self._handle_audio, qos_profile)

    def _handle_audio(self, msg: UInt8MultiArray) -> None:
        data = bytes(msg.data)
        if len(data) < self._sample_width:
            return
        try:
            rms = audioop.rms(data, self._sample_width)
        except audioop.error as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning(f"Failed to compute RMS for audio chunk: {exc}")
            return
        self._recent.append(float(rms))
        average_rms = sum(self._recent) / len(self._recent)
        is_silence = average_rms < self._threshold
        if not self._publish_on_change or is_silence != self._last_state:
            bool_msg = Bool()
            bool_msg.data = is_silence
            self._publisher.publish(bool_msg)
            self._last_state = is_silence


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SilenceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Silence detector interrupted; shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["SilenceDetectorNode", "main"]
