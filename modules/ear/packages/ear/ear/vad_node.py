#!/usr/bin/env python3
"""Voice activity detection node that emits VAD-tagged audio frames."""
from __future__ import annotations

import audioop
import rclpy
import webrtcvad
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

from .qos import sensor_data_qos

try:
    from psyched_msgs.msg import VadFrame
except ImportError:  # pragma: no cover - enables unit tests without generated interfaces
    from dataclasses import dataclass
    from typing import Any

    @dataclass
    class VadFrame:  # type: ignore[override]
        stamp: Any = None
        sample_rate: int = 0
        frame_samples: int = 0
        is_speech: bool = False
        audio: bytes = b""

from .audio_utils import coerce_pcm_bytes


class VADNode(Node):
    """ROS 2 node that tags incoming audio frames with VAD decisions."""

    def __init__(self) -> None:
        super().__init__('vad_node')

        mode = int(self.declare_parameter('vad_mode', 3).value)
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(max(0, min(3, mode)))

        self.target_sample_rate = int(self.declare_parameter('target_sample_rate', 16000).value)
        self.frame_duration_ms = int(self.declare_parameter('frame_duration_ms', 30).value)
        self.frame_size = int(self.target_sample_rate * (self.frame_duration_ms / 1000.0) * 2)

        self._input_topic = self.declare_parameter('input_topic', '/audio/raw').value
        self._frame_topic = self.declare_parameter('frame_topic', '/audio/vad_frames').value

        self._buffer = b''

        self._subscription = self.create_subscription(
            ByteMultiArray, self._input_topic, self.audio_callback, sensor_data_qos()
        )
        self._publisher = self.create_publisher(VadFrame, self._frame_topic, sensor_data_qos())

        self.get_logger().info(
            (
                'VAD node started: input=%s target_rate=%dHz frame=%dms topic=%s'
                % (self._input_topic, self.target_sample_rate, self.frame_duration_ms, self._frame_topic)
            )
        )

    def audio_callback(self, msg: ByteMultiArray) -> None:
        """Process incoming audio frames and publish per-frame VAD metadata."""
        raw_audio = coerce_pcm_bytes(msg.data)
        if not raw_audio:
            return

        resampled_audio, _ = audioop.ratecv(raw_audio, 2, 1, 44100, self.target_sample_rate, None)
        self._buffer += resampled_audio

        while len(self._buffer) >= self.frame_size:
            frame = self._buffer[: self.frame_size]
            self._buffer = self._buffer[self.frame_size :]

            is_speech = bool(self.vad.is_speech(frame, self.target_sample_rate))

            message = VadFrame()
            message.stamp = self.get_clock().now().to_msg()
            message.sample_rate = self.target_sample_rate
            message.frame_samples = len(frame) // 2
            message.is_speech = is_speech
            message.audio = bytes(frame)

            self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VADNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
