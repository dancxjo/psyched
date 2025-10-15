"""ROS 2 node that bridges audio topics into transcription backends."""

from __future__ import annotations

from typing import Sequence

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, UInt8MultiArray

from .backends import (
    AudioAwareBackend,
    ConsoleEarBackend,
    EarBackend,
    FasterWhisperEarBackend,
    ServiceASREarBackend,
)
from .worker import EarWorker


class TranscriberNode(Node):
    """Publish transcripts derived from audio topics using configured backends."""

    def __init__(self) -> None:
        super().__init__("ear_transcriber")
        transcript_topic_param = str(self.declare_parameter("transcript_topic", "").value).strip()
        if transcript_topic_param:
            self._transcript_topic = transcript_topic_param
        else:
            self._transcript_topic = self._declare_topic("hole_topic", "/ear/hole")
        self._publisher = self.create_publisher(String, self._transcript_topic, 10)
        backend = self._create_backend()
        self._worker = EarWorker(backend=backend, publisher=self._publish_text, logger=self.get_logger())
        self._worker.start()

        self._text_subscription = None
        text_topic = self._declare_topic("text_input_topic", "")
        if text_topic and text_topic != self._transcript_topic:
            self._text_subscription = self.create_subscription(String, text_topic, self._handle_text, 10)
        elif text_topic == self._transcript_topic:
            self.get_logger().warning("text_input_topic matches transcript_topic; ignoring to avoid loop")

        self._audio_subscription = None
        self._audio_sample_rate = int(self.declare_parameter("audio_sample_rate", 16000).value)
        self._audio_channels = int(self.declare_parameter("audio_channels", 1).value)
        reliability_param = str(self.declare_parameter("audio_reliability", "best_effort").value).strip().lower()
        audio_qos = QoSProfile(depth=10)
        if reliability_param == "reliable":
            audio_qos.reliability = QoSReliabilityPolicy.RELIABLE
        else:
            audio_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        if isinstance(backend, AudioAwareBackend):
            audio_topic = self._declare_topic("audio_topic", "/audio/raw")
            self._audio_subscription = self.create_subscription(
                UInt8MultiArray,
                audio_topic,
                self._handle_audio,
                audio_qos,
            )

        self.get_logger().info(
            f"Transcriber ready (backend={backend.__class__.__name__}, transcript_topic={self._transcript_topic})",
        )

    def _declare_topic(self, name: str, default: str) -> str:
        parameter = self.declare_parameter(name, default)
        value = str(parameter.value).strip()
        return value or default

    def _create_backend(self) -> EarBackend:
        backend_name = str(self.declare_parameter("backend", "console").value).strip().lower()
        if backend_name in {"console", "stdin", "text"}:
            return ConsoleEarBackend()
        if backend_name in {"faster_whisper", "whisper"}:
            try:
                import faster_whisper  # type: ignore[import-not-found]
            except ImportError:
                self.get_logger().warning(
                    "faster-whisper backend requested but dependency is missing; falling back to console",
                )
                return ConsoleEarBackend()
            options = self._read_faster_whisper_options()
            return FasterWhisperEarBackend(**options)
        if backend_name in {"service", "asr", "websocket"}:
            uri = str(self.declare_parameter("service_uri", "ws://127.0.0.1:8089/ws").value).strip() or "ws://127.0.0.1:8089/ws"
            options = self._read_faster_whisper_options()
            return ServiceASREarBackend(uri=uri, fallback_factory=lambda: FasterWhisperEarBackend(**options))
        self.get_logger().warning(
            f"Unknown backend '{backend_name}'; defaulting to console backend",
        )
        return ConsoleEarBackend()

    def _read_faster_whisper_options(self) -> dict[str, object]:
        model = str(self.declare_parameter("faster_whisper_model", "base").value).strip() or "base"
        device = str(self.declare_parameter("faster_whisper_device", "cpu").value).strip() or "cpu"
        compute_type = str(self.declare_parameter("faster_whisper_compute_type", "int8").value).strip() or "int8"
        language_param = str(self.declare_parameter("faster_whisper_language", "").value).strip()
        beam_size_param = self.declare_parameter("faster_whisper_beam_size", 5).value
        beam_size: int | None
        if isinstance(beam_size_param, (int, float)):
            beam_size = int(beam_size_param)
        else:
            beam_size = 5
        return {
            "model_size": model,
            "device": device,
            "compute_type": compute_type,
            "language": language_param or None,
            "beam_size": beam_size,
        }

    def _handle_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self._publish_text(text)

    def _handle_audio(self, msg: UInt8MultiArray) -> None:
        if not msg.data:
            return
        pcm = bytes(msg.data)
        self._worker.submit_audio(pcm, self._audio_sample_rate, self._audio_channels)

    def _publish_text(self, text: str) -> None:
        ros_msg = String()
        ros_msg.data = text
        self._publisher.publish(ros_msg)
        self.get_logger().info(f"Heard: {text}")

    def destroy_node(self) -> bool:
        self._worker.stop()
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TranscriberNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Transcriber interrupted; shutting down")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


EarNode = TranscriberNode

__all__ = ["TranscriberNode", "EarNode", "main"]
