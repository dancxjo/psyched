"""ROS 2 node orchestrating queued speech playback."""

from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .backends import (
    EspeakSpeechBackend,
    PrintSpeechBackend,
    SpeechBackend,
    WebsocketTTSSpeechBackend,
)
from .queue import SpeechQueue


class VoiceNode(Node):
    """ROS node that serializes text-to-speech playback.

    The node listens for string messages on the configured input topic and
    dispatches them to a :class:`~voice.queue.SpeechQueue`. Playback is performed
    by a configurable :class:`~voice.backends.SpeechBackend`. When playback
    completes the spoken text is republished on the ``spoken_topic`` so other
    nodes can reconcile what was said. Additional control topics allow pausing,
    resuming, and clearing queued utterances. Partial playback acknowledgements
    are not emitted—the node only publishes once an utterance finishes.
    """

    def __init__(self) -> None:
        super().__init__("voice")
        self._queue: SpeechQueue | None = None
        self._spoken_pub = self.create_publisher(String, self._declare_topic("spoken_topic", "/voice/spoken"), 10)
        backend = self._create_backend()
        self._queue = SpeechQueue(backend, on_spoken=self._publish_spoken)

        input_topic = self._declare_topic("input_topic", "/voice/text")
        pause_topic = self._declare_topic("pause_topic", "/voice/pause")
        resume_topic = self._declare_topic("resume_topic", "/voice/resume")
        clear_topic = self._declare_topic("clear_topic", "/voice/clear")

        self.create_subscription(String, input_topic, self._handle_text, 10)
        self.create_subscription(Empty, pause_topic, self._handle_pause, 10)
        self.create_subscription(Empty, resume_topic, self._handle_resume, 10)
        self.create_subscription(Empty, clear_topic, self._handle_clear, 10)

        self.get_logger().info(
            "Voice node ready (backend=%s, input_topic=%s, spoken_topic=%s)",
            backend.__class__.__name__,
            input_topic,
            self._spoken_pub.topic_name,
        )

    # ------------------------------------------------------------------ helpers
    def _declare_topic(self, name: str, default: str) -> str:
        parameter = self.declare_parameter(name, default)
        value = str(parameter.value).strip() or default
        return value

    def _create_backend(self) -> SpeechBackend:
        backend_name = (
            str(self.declare_parameter("backend", "print").value).strip().lower()
        )
        if backend_name == "print":
            return PrintSpeechBackend()
        if backend_name == "espeak":
            try:
                return EspeakSpeechBackend()
            except FileNotFoundError as error:
                self.get_logger().warning(
                    "espeak backend unavailable (%s); falling back to print backend",
                    error,
                )
                return PrintSpeechBackend()
        if backend_name in {"websocket", "coqui"}:
            backend = self._create_websocket_backend()
            if backend is not None:
                return backend
            self.get_logger().warning(
                "Falling back to print backend after websocket backend initialisation failure",
            )
        self.get_logger().warning(
            "Unknown backend '%s'; defaulting to print backend", backend_name
        )
        return PrintSpeechBackend()

    def _create_websocket_backend(self) -> SpeechBackend | None:
        url_param = str(self.declare_parameter("tts_url", "").value).strip()
        if url_param:
            url = url_param
        else:
            scheme = str(self.declare_parameter("tts_scheme", "ws").value).strip() or "ws"
            host = str(self.declare_parameter("tts_host", "127.0.0.1").value).strip() or "127.0.0.1"
            port = int(self.declare_parameter("tts_port", 5002).value)
            path = str(self.declare_parameter("tts_path", "/tts").value).strip() or "/tts"
            if not path.startswith("/"):
                path = "/" + path
            url = f"{scheme}://{host}:{port}{path}"

        speaker = str(self.declare_parameter("tts_speaker", "").value).strip() or None
        language = str(self.declare_parameter("tts_language", "").value).strip() or None
        player_param = self.declare_parameter("tts_player_command", []).value
        player_command: list[str] | None = None
        if isinstance(player_param, (list, tuple)):
            player_command = [str(item) for item in player_param if str(item).strip()]
        else:
            command_text = str(player_param).strip()
            if command_text:
                player_command = command_text.split()
        if player_command and not player_command[0].strip():
            player_command = None

        try:
            return WebsocketTTSSpeechBackend(
                url=url,
                speaker=speaker,
                language=language,
                player_command=player_command,
            )
        except Exception as error:
            self.get_logger().error(
                "Failed to initialise websocket speech backend: %s",
                error,
            )
            return None

    # ---------------------------------------------------------------- callbacks
    def _handle_text(self, msg: String) -> None:
        assert self._queue is not None
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().debug("Queueing text: %s", text)
        self._queue.enqueue(text)

    def _handle_pause(self, _: Empty) -> None:
        if self._queue is None:
            return
        self.get_logger().info("Pausing speech queue")
        self._queue.pause()

    def _handle_resume(self, _: Empty) -> None:
        if self._queue is None:
            return
        self.get_logger().info("Resuming speech queue")
        self._queue.resume()

    def _handle_clear(self, _: Empty) -> None:
        if self._queue is None:
            return
        self.get_logger().info("Clearing speech queue")
        self._queue.clear()

    def _publish_spoken(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._spoken_pub.publish(msg)
        self.get_logger().info("Spoken: %s", text)

    # ---------------------------------------------------------------- lifecycle
    def destroy_node(self) -> bool:
        if self._queue is not None:
            self._queue.shutdown()
            self._queue = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Voice node interrupted; shutting down")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

