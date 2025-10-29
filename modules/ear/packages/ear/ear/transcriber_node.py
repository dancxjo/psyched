"""ROS 2 node that bridges audio topics into transcription backends."""

from __future__ import annotations

import importlib
import json
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
    TranscriptSegment,
    TranscriptionEvent,
    ServiceASREarBackend,
)
from .worker import EarWorker


def _thread_from_topic(topic: str, *, prefix: str, default: str) -> str:
    """Return the conversation thread identifier encoded in *topic*."""

    cleaned_topic = (topic or "").strip()
    if not cleaned_topic:
        return default

    candidate = cleaned_topic.rstrip("/").split("/")[-1]
    cleaned_prefix = (prefix or "").strip()
    if cleaned_prefix:
        trimmed_prefix = cleaned_prefix.rstrip("/")
        if trimmed_prefix and cleaned_topic.startswith(trimmed_prefix):
            remainder = cleaned_topic[len(trimmed_prefix) :].lstrip("/")
            if remainder:
                candidate = remainder.split("/", 1)[0]

    return candidate or default


class TranscriberNode(Node):
    """Publish transcripts derived from audio topics using configured backends."""

    def __init__(self) -> None:
        super().__init__("ear_transcriber")
        self._warned_missing_faster_whisper = False
        transcript_topic_param = str(self.declare_parameter("transcript_topic", "").value).strip()
        if transcript_topic_param:
            self._transcript_topic = transcript_topic_param
        else:
            self._transcript_topic = self._declare_topic("hole_topic", "/ear/hole")
        self._publisher = self.create_publisher(String, self._transcript_topic, 10)
        self._event_topic = self._declare_topic("event_topic", "/ear/asr_event")
        self._event_publisher = self.create_publisher(String, self._event_topic, 10)
        backend = self._create_backend()
        self._worker = EarWorker(backend=backend, publisher=self._handle_event, logger=self.get_logger())
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

        self._conversation_topic_prefix = str(
            self.declare_parameter("conversant_conversation_prefix", "/conversation").value
        ).strip() or "/conversation"
        self._default_conversation_thread = str(
            self.declare_parameter("conversant_default_thread", "default").value
        ).strip() or "default"
        self._conversant_user_id = str(self.declare_parameter("conversant_user_id", "").value).strip()
        self._active_conversation_thread = self._default_conversation_thread

        self._conversant_concern_topic = self._declare_topic("conversant_concern_topic", "/conversant/concern")
        self._conversant_topic_topic = self._declare_topic("conversant_topic_topic", "/conversant/topic")
        self._conversant_publisher = self.create_publisher(String, self._conversant_concern_topic, 10)
        self._conversation_topic_subscription = self.create_subscription(
            String,
            self._conversant_topic_topic,
            self._handle_conversation_topic,
            10,
        )

        backend_name = backend.__class__.__name__
        self.get_logger().info(
            f"Transcriber ready (backend={backend_name}, transcript_topic={self._transcript_topic}, "
            f"event_topic={self._event_topic})"
        )

    def _declare_topic(self, name: str, default: str) -> str:
        parameter = self.declare_parameter(name, default)
        value = str(parameter.value).strip()
        return value or default

    def _faster_whisper_available(self) -> bool:
        try:
            importlib.import_module("faster_whisper")  # type: ignore[import-not-found]
        except ImportError:
            if not self._warned_missing_faster_whisper:
                self._warned_missing_faster_whisper = True
                self.get_logger().error(
                    "faster-whisper dependency is missing; install it with `psh mod pip ear` "
                    "or `python3 -m pip install --break-system-packages faster-whisper` to enable the offline fallback.",
                )
            return False
        return True

    def _create_backend(self) -> EarBackend:
        backend_name = str(self.declare_parameter("backend", "console").value).strip().lower()
        if backend_name in {"console", "stdin", "text"}:
            return ConsoleEarBackend()
        if backend_name in {"faster_whisper", "whisper"}:
            if not self._faster_whisper_available():
                self.get_logger().warning(
                    "faster-whisper backend requested but dependency is missing; falling back to console",
                )
                return ConsoleEarBackend()
            options = self._read_faster_whisper_options()
            return FasterWhisperEarBackend(**options)
        if backend_name in {"service", "asr", "websocket"}:
            uri = (
                str(self.declare_parameter("service_uri", "ws://127.0.0.1:5003/asr").value).strip()
                or "ws://127.0.0.1:5003/asr"
            )
            options = self._read_faster_whisper_options()
            fallback_factory = (
                (lambda: FasterWhisperEarBackend(**options))
                if self._faster_whisper_available()
                else None
            )
            if fallback_factory is None:
                self.get_logger().warning(
                    "ASR service fallback to faster-whisper is disabled until the dependency is installed.",
                )
            return ServiceASREarBackend(uri=uri, fallback_factory=fallback_factory)
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
        self._publish_final_text(text)

    def _handle_audio(self, msg: UInt8MultiArray) -> None:
        if not msg.data:
            return
        pcm = bytes(msg.data)
        self._worker.submit_audio(pcm, self._audio_sample_rate, self._audio_channels)

    def _handle_event(self, event: TranscriptionEvent) -> None:
        """Serialise transcription events and forward final text to ROS topics."""

        try:
            payload = json.dumps(event.to_dict(), ensure_ascii=False)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Failed to encode transcription event: {error}")
            return

        event_msg = String()
        event_msg.data = payload
        self._event_publisher.publish(event_msg)

        if event.is_partial:
            self.get_logger().debug(f"Partial transcription: {event.text}")
            return

        text = event.text.strip()
        if not text:
            text = self._segments_to_text(event.segments)
        if not text:
            return
        self._publish_final_text(text)

    def _publish_final_text(self, text: str) -> None:
        ros_msg = String()
        ros_msg.data = text
        self._publisher.publish(ros_msg)
        self.get_logger().info(f"Heard: {text}")
        self._forward_transcript_to_conversant(text)

    @staticmethod
    def _segments_to_text(segments: Sequence[TranscriptionSegment]) -> str:
        parts = [segment.text.strip() for segment in segments if getattr(segment, "text", "").strip()]
        return " ".join(parts)

    def _handle_conversation_topic(self, msg: String) -> None:
        topic = str(getattr(msg, "data", "") or "")
        thread_id = _thread_from_topic(
            topic,
            prefix=self._conversation_topic_prefix,
            default=self._default_conversation_thread,
        )
        if thread_id != self._active_conversation_thread:
            topic_display = topic or "<empty>"
            self.get_logger().debug(f"Active conversation thread -> {thread_id} (topic={topic_display})")
        self._active_conversation_thread = thread_id or self._default_conversation_thread

    def _forward_transcript_to_conversant(self, text: str) -> None:
        publisher = getattr(self, "_conversant_publisher", None)
        if publisher is None:
            return

        cleaned = text.strip()
        if not cleaned:
            return

        thread_id = self._active_conversation_thread or self._default_conversation_thread
        payload: dict[str, str] = {"concern": cleaned, "thread_id": thread_id}
        if self._conversant_user_id:
            payload["user_id"] = self._conversant_user_id

        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(message)
        self.get_logger().debug(f"Forwarded transcript to Conversant thread {thread_id}")

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
