"""ROS 2 node orchestrating transcription backends for the ear module."""

from __future__ import annotations


import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

from .backends import (
    AudioAwareBackend,
    ConsoleEarBackend,
    EarBackend,
    FasterWhisperEarBackend,
    ServiceASREarBackend,
)
from .worker import EarWorker


class EarNode(Node):
    """ROS node that publishes recognised transcripts to a topic."""

    def __init__(self) -> None:
        super().__init__("ear")
        self._hole_topic = self._declare_topic("hole_topic", "/ear/hole")
        self._publisher = self.create_publisher(String, self._hole_topic, 10)
        backend = self._create_backend()
        self._worker = EarWorker(backend=backend, publisher=self._publish_text, logger=self.get_logger())
        self._worker.start()

        self._text_subscription = None
        text_topic = self._declare_topic("text_input_topic", "")
        if text_topic and text_topic != self._hole_topic:
            self._text_subscription = self.create_subscription(String, text_topic, self._handle_text, 10)
        elif text_topic == self._hole_topic:
            self.get_logger().warning("text_input_topic matches hole_topic; ignoring to avoid republishing loop")

        self._audio_subscription = None
        self._audio_sample_rate = 16000
        self._audio_channels = 1
        if isinstance(backend, AudioAwareBackend):
            audio_topic = self._declare_topic("audio_topic", "/audio/raw")
            self._audio_sample_rate = int(self.declare_parameter("audio_sample_rate", 16000).value)
            self._audio_channels = int(self.declare_parameter("audio_channels", 1).value)
            self._audio_subscription = self.create_subscription(
                UInt8MultiArray,
                audio_topic,
                self._handle_audio,
                10,
            )

        self.get_logger().info(
            "Ear node ready (backend=%s, hole_topic=%s)",
            backend.__class__.__name__,
            self._hole_topic,
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
                self.get_logger().warning("faster-whisper backend requested but dependency is missing; falling back to console")
                return ConsoleEarBackend()
            model = str(self.declare_parameter("faster_whisper_model", "base").value).strip() or "base"
            device = str(self.declare_parameter("faster_whisper_device", "cpu").value).strip() or "cpu"
            compute_type = str(self.declare_parameter("faster_whisper_compute_type", "int8").value).strip() or "int8"
            language_param = str(self.declare_parameter("faster_whisper_language", "").value).strip()
            beam_size = self.declare_parameter("faster_whisper_beam_size", 5).value
            beam_size_value = int(beam_size) if isinstance(beam_size, (int, float)) else 5
            return FasterWhisperEarBackend(
                model_size=model,
                device=device,
                compute_type=compute_type,
                language=language_param or None,
                beam_size=beam_size_value,
            )
        if backend_name in {"service", "asr", "websocket"}:
            uri = str(self.declare_parameter("service_uri", "ws://127.0.0.1:8089/ws").value).strip() or "ws://127.0.0.1:8089/ws"
            return ServiceASREarBackend(uri=uri)
        self.get_logger().warning("Unknown backend '%s'; defaulting to console backend", backend_name)
        return ConsoleEarBackend()

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
        self.get_logger().info("Heard: %s", text)

    def destroy_node(self) -> bool:
        self._worker.stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EarNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Ear node interrupted; shutting down")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
