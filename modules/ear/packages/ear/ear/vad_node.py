"""ROS 2 node that applies voice activity detection to PCM audio chunks."""

from __future__ import annotations

from collections import deque
from typing import Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, UInt8MultiArray
import webrtcvad


class VadNode(Node):
    """Detect speech using WebRTC VAD and publish boolean state changes."""

    def __init__(self) -> None:
        super().__init__("ear_vad")
        self._audio_topic = str(self.declare_parameter("audio_topic", "/audio/raw").value)
        self._speech_topic = str(self.declare_parameter("speech_topic", "/ear/speech_active").value)
        self._sample_rate = int(self.declare_parameter("sample_rate", 16000).value)
        if self._sample_rate not in (8000, 16000, 32000, 48000):
            raise ValueError("sample_rate must be one of 8000, 16000, 32000, 48000 for WebRTC VAD")
        self._channels = int(self.declare_parameter("channels", 1).value)
        if self._channels != 1:
            self.get_logger().warning("VAD currently expects mono audio; received %s channels", self._channels)
        self._frame_duration_ms = int(self.declare_parameter("frame_duration_ms", 20).value)
        if self._frame_duration_ms not in (10, 20, 30):
            raise ValueError("frame_duration_ms must be 10, 20, or 30")
        self._publish_on_change = bool(self.declare_parameter("publish_on_change", True).value)
        aggressiveness = int(self.declare_parameter("aggressiveness", 2).value)
        self._vad = webrtcvad.Vad(aggressiveness)
        self._frame_bytes = int(self._sample_rate * (self._frame_duration_ms / 1000.0) * 2 * max(self._channels, 1))
        smoothing_param = int(self.declare_parameter("smoothing_window", 1).value)
        smoothing_param = max(smoothing_param, 1)
        self._buffer = bytearray()
        self._recent: deque[bool] = deque(maxlen=smoothing_param)
        self._last_state: bool | None = None
        qos_profile = QoSProfile(depth=20)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self._publisher = self.create_publisher(Bool, self._speech_topic, qos_profile)
        self.create_subscription(UInt8MultiArray, self._audio_topic, self._handle_audio, qos_profile)

    def _handle_audio(self, msg: UInt8MultiArray) -> None:
        data = bytes(msg.data)
        if not data:
            return
        self._buffer.extend(data)
        while len(self._buffer) >= self._frame_bytes:
            frame = bytes(self._buffer[: self._frame_bytes])
            del self._buffer[: self._frame_bytes]
            is_speech = self._vad.is_speech(frame, self._sample_rate)
            self._recent.append(is_speech)
            smoothed = all(self._recent) if self._recent else is_speech
            if not self._publish_on_change or smoothed != self._last_state:
                bool_msg = Bool()
                bool_msg.data = smoothed
                self._publisher.publish(bool_msg)
                self._last_state = smoothed


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("VAD node interrupted; shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["VadNode", "main"]
