#!/usr/bin/env python3
"""Speak queued messages using a configurable text-to-speech provider."""

from __future__ import annotations

import os
import random
import queue
import threading
import time
from typing import Callable, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String, UInt32

try:  # pragma: no cover - requires ROS runtime
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
except ImportError:  # pragma: no cover - exercised by unit tests
    QoSDurabilityPolicy = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    QoSReliabilityPolicy = None  # type: ignore[assignment]


def _best_effort_qos(*, depth: int = 10):
    if (
        QoSProfile is None
        or QoSHistoryPolicy is None
        or QoSReliabilityPolicy is None
        or QoSDurabilityPolicy is None
    ):
        return depth
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

from psyched_msgs.msg import Message as MsgMessage

from .providers import (
    EspeakConfig,
    MsEdgeConfig,
    PiperConfig,
    ProviderUnavailable,
    WebsocketConfig,
    build_provider_with_fallback,
    build_tts_provider,
)
from .utils import fetch_fortune_text

DEFAULT_ENGINE = os.getenv("VOICE_ENGINE", "websocket")
DEFAULT_WS_URL = os.getenv("VOICE_TTS_URL", "ws://forebrain.local:5002/tts")

FALLBACK_FORTUNES: tuple[str, ...] = (
    "Stay curious.",
    "Keep exploring.",
    "Adventure awaits.",
    "Wonder is contagious.",
    "Hello from the void.",
    "Dreams fuel journeys.",
)


class VoiceNode(Node):
    """Queue text and play it back via the configured TTS provider."""

    def __init__(self) -> None:
        super().__init__("voice")

        self.declare_parameter("topic", "/voice")
        self.declare_parameter("engine", DEFAULT_ENGINE)
        self.declare_parameter("volume", 1.0)
        self.declare_parameter("startup_greeting", "Hi there! My name is Pete Knighty-kell.")
        self.declare_parameter("enable_ping", True)
        self.declare_parameter("ping_interval_sec", 30)
        self.declare_parameter("conversation_topic", "/conversation")
        self.declare_parameter("pause_topic", "/voice/interrupt")
        self.declare_parameter("resume_topic", "/voice/resume")
        self.declare_parameter("clear_topic", "/voice/clear")
        self.declare_parameter("interrupt_topic", "/voice/interrupt")

        # Websocket provider parameters
        self.declare_parameter("tts_ws_url", DEFAULT_WS_URL)
        self.declare_parameter("tts_ws_speaker", "")
        self.declare_parameter("tts_ws_language", "")
        self.declare_parameter("tts_ws_open_timeout", 5.0)
        self.declare_parameter("tts_ws_close_timeout", 5.0)
        self.declare_parameter("tts_ws_metadata_timeout", 2.0)
        self.declare_parameter("tts_ws_chunk_timeout", 1.0)
        self.declare_parameter("tts_ws_ping_interval", 20.0)
        self.declare_parameter("tts_ws_ping_timeout", 20.0)
        self.declare_parameter("playback_backend", "ffplay")
        self.declare_parameter("playback_device", "")

        # Legacy espeak parameters for compatibility
        self.declare_parameter("espeak_voice", "mb-en1")
        self.declare_parameter("espeak_rate", 170)
        self.declare_parameter("espeak_pitch", 50)
        self.declare_parameter("espeak_volume", 1.0)
        self.declare_parameter("espeak_extra_args", "")
        self.declare_parameter("aplay", True)
        self.declare_parameter("aplay_device", "")

        # Microsoft Edge compatibility parameters
        self.declare_parameter("edge_voice", "en-US-GuyNeural")
        self.declare_parameter("edge_rate", "+0%")
        self.declare_parameter("edge_volume", "+0%")
        self.declare_parameter("edge_pitch", "+0Hz")
        self.declare_parameter("edge_style", "")

        # Piper compatibility parameters
        self.declare_parameter("model", "")
        self.declare_parameter("voices_dir", "")
        self.declare_parameter("wav_out_dir", "")
        self.declare_parameter("length_scale", 1.0)
        self.declare_parameter("noise_scale", 0.667)
        self.declare_parameter("noise_w_scale", 0.8)
        self.declare_parameter("normalize_audio", True)

        self.engine = self._get_string_param("engine", DEFAULT_ENGINE).lower()
        self.volume = self._get_float_param("volume", 1.0)
        self.volume = max(0.0, min(2.0, self.volume))

        self.topic = self._get_string_param("topic", "/voice")
        self._pub_done = self.create_publisher(String, "voice_done", _best_effort_qos(depth=5))
        self._conversation_topic = self._get_string_param("conversation_topic", "/conversation")
        self._pub_conversation = self.create_publisher(
            MsgMessage,
            self._conversation_topic,
            _best_effort_qos(depth=10),
        )
        self.autophony_pub = self.create_publisher(
            UInt32,
            "/audio/autophony_duration",
            _best_effort_qos(depth=10),
        )

        self.create_subscription(String, self.topic, self.enqueue, 10)
        self.create_subscription(String, "voice_interrupt", lambda _msg: self._on_pause(None), 10)

        pause_topic = self._get_string_param("pause_topic", "/voice/interrupt")
        resume_topic = self._get_string_param("resume_topic", "/voice/resume")
        clear_topic = self._get_string_param("clear_topic", "/voice/clear")
        interrupt_topic = self._get_string_param("interrupt_topic", "/voice/interrupt")

        self._pause_sub = self.create_subscription(Empty, pause_topic, self._on_pause, 1)
        self._resume_sub = self.create_subscription(Empty, resume_topic, self._on_resume, 1)
        self._clear_sub = self.create_subscription(Empty, clear_topic, self._on_clear, 1)
        self._interrupt_sub = self.create_subscription(Empty, interrupt_topic, self._on_pause, 1)
        self._volume_sub = self.create_subscription(Float32, "/voice/volume", self._on_volume, 1)

        self._queue: queue.Queue[str] = queue.Queue()
        self._stop_event = threading.Event()
        self._unpaused = threading.Event()
        self._unpaused.set()
        self._worker: Optional[threading.Thread] = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

        self._provider_lock = threading.Lock()
        self._provider = None
        self._provider_engine: Optional[str] = None

        self._ping_timer = None
        self._announce_startup_message()
        self._setup_ping_timer()

        self.get_logger().info(f"Voice node listening on topic: {self.topic}")
        self.get_logger().info(f"Using TTS engine: {self.engine}")

    # ------------------------------------------------------------------
    # Parameter helpers
    def _get_string_param(self, name: str, default: str = "") -> str:
        try:
            value = self.get_parameter(name).get_parameter_value().string_value
        except Exception:
            value = default
        if not value:
            try:
                value = str(self.get_parameter(name).value)
            except Exception:
                value = default
        return value or default

    def _get_float_param(self, name: str, default: float = 0.0) -> float:
        try:
            return float(self.get_parameter(name).get_parameter_value().double_value)
        except Exception:
            try:
                return float(self.get_parameter(name).get_parameter_value().integer_value)
            except Exception:
                try:
                    return float(self.get_parameter(name).value)
                except Exception:
                    return default

    def _get_int_param(self, name: str, default: int = 0) -> int:
        try:
            return int(self.get_parameter(name).get_parameter_value().integer_value)
        except Exception:
            try:
                return int(self.get_parameter(name).value)
            except Exception:
                return default

    def _get_bool_param(self, name: str, default: bool = False) -> bool:
        try:
            parameter = self.get_parameter(name)
        except Exception:
            return default

        value = getattr(parameter, "value", default)
        return self._coerce_bool(value, default)

    @staticmethod
    def _coerce_bool(value: object, default: bool) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "1", "yes", "on"}:
                return True
            if normalized in {"false", "0", "no", "off"}:
                return False
            return default
        if isinstance(value, (int, float)):
            return bool(value)
        return default

    # ------------------------------------------------------------------
    # Provider configuration
    def _get_provider(self):
        engine = (self.engine or "websocket").strip().lower()
        with self._provider_lock:
            if self._provider is not None and self._provider_engine != engine:
                try:
                    self._provider.stop()
                except Exception:
                    pass
                self._provider = None
                self._provider_engine = None

            if self._provider is None:
                try:
                    fallbacks: list[str] = []
                    if engine in {"websocket", "ws"}:
                        fallbacks.append("espeak")
                    if fallbacks:
                        selected_engine, provider = build_provider_with_fallback(
                            preferred=engine,
                            fallbacks=fallbacks,
                            logger=self.get_logger(),
                            config_getter_factory=self._make_config_getter,
                        )
                    else:
                        selected_engine = engine
                        config_getter = self._make_config_getter(engine)
                        provider = build_tts_provider(engine, logger=self.get_logger(), config_getter=config_getter)
                    if selected_engine != engine:
                        self.get_logger().warning(
                            f"TTS provider '{engine}' unavailable; switching to '{selected_engine}'",
                        )
                        self.engine = selected_engine
                    self._provider = provider
                    self._provider_engine = selected_engine
                except ProviderUnavailable as exc:
                    self.get_logger().error(f"TTS provider unavailable ({engine}): {exc}")
                    self._provider = None
                    self._provider_engine = None
        return self._provider

    def _make_config_getter(self, engine: str) -> Callable[[], object]:
        if engine in {"websocket", "ws"}:
            def getter() -> WebsocketConfig:
                return WebsocketConfig(
                    uri=self._get_string_param("tts_ws_url", DEFAULT_WS_URL),
                    speaker=self._get_string_param("tts_ws_speaker", "") or None,
                    language=self._get_string_param("tts_ws_language", "") or None,
                    open_timeout=self._get_float_param("tts_ws_open_timeout", 5.0),
                    close_timeout=self._get_float_param("tts_ws_close_timeout", 5.0),
                    metadata_timeout=self._get_float_param("tts_ws_metadata_timeout", 2.0),
                    chunk_timeout=self._get_float_param("tts_ws_chunk_timeout", 1.0),
                    ping_interval=self._get_float_param("tts_ws_ping_interval", 20.0),
                    ping_timeout=self._get_float_param("tts_ws_ping_timeout", 20.0),
                    playback_backend=self._get_string_param("playback_backend", "ffplay"),
                    playback_device=self._get_string_param("playback_device", "") or None,
                )
            return getter

        if engine in {"espeak", "espeak-ng"}:
            def getter() -> EspeakConfig:
                return EspeakConfig(
                    voice=self._get_string_param("espeak_voice", "mb-en1"),
                    rate=self._get_int_param("espeak_rate", 170),
                    pitch=self._get_int_param("espeak_pitch", 50),
                    base_volume=self._get_float_param("espeak_volume", 1.0),
                    extra_args=self._get_string_param("espeak_extra_args", ""),
                    aplay_enabled=self._get_bool_param("aplay", True),
                    aplay_device=self._get_string_param("aplay_device", "") or None,
                )
            return getter

        if engine in {"ms-edge", "msedge", "edge", "edge_tts"}:
            def getter() -> MsEdgeConfig:
                return MsEdgeConfig(
                    voice=self._get_string_param("edge_voice", "en-US-GuyNeural"),
                    rate=self._get_string_param("edge_rate", "+0%"),
                    volume=self._get_string_param("edge_volume", "+0%"),
                    pitch=self._get_string_param("edge_pitch", "+0Hz"),
                    style=self._get_string_param("edge_style", "" ) or None,
                    output_format="audio-16khz-64kbitrate-mono-mp3",
                    ffplay_args=("-nodisp", "-autoexit", "-loglevel", "error"),
                )
            return getter

        if engine == "piper":
            def getter() -> PiperConfig:
                return PiperConfig(
                    model=self._get_string_param("model", ""),
                    voices_dir=self._get_string_param("voices_dir", ""),
                    wav_out_dir=self._get_string_param("wav_out_dir", ""),
                    length_scale=self._get_float_param("length_scale", 1.0),
                    noise_scale=self._get_float_param("noise_scale", 0.667),
                    noise_w_scale=self._get_float_param("noise_w_scale", 0.8),
                    normalize_audio=self._get_bool_param("normalize_audio", True),
                    aplay_enabled=self._get_bool_param("aplay", True),
                    aplay_device=self._get_string_param("aplay_device", "") or None,
                )
            return getter

        raise ProviderUnavailable(f"Unsupported TTS engine: {engine}")

    # ------------------------------------------------------------------
    # Queue management and playback
    def enqueue(self, msg: String) -> None:
        if msg.data:
            self._queue.put(msg.data)

    def interrupt(self, _msg: Optional[String] = None) -> None:
        self._stop_active_provider()
        with self._queue.mutex:  # type: ignore[attr-defined]
            self._queue.queue.clear()  # type: ignore[attr-defined]

    def _on_pause(self, _msg: Optional[Empty]) -> None:
        self._stop_active_provider()
        self._unpaused.clear()
        self.get_logger().info("Voice paused")

    def _on_resume(self, _msg: Optional[Empty]) -> None:
        if not self._unpaused.is_set():
            self.get_logger().info("Voice resumed")
        self._unpaused.set()

    def _on_clear(self, _msg: Optional[Empty]) -> None:
        cleared = 0
        try:
            while True:
                self._queue.get_nowait()
                cleared += 1
        except queue.Empty:
            pass
        self.get_logger().info(f"Cleared {cleared} queued utterance(s)")

    def _on_volume(self, msg: Float32) -> None:
        try:
            new_volume = float(msg.data)
        except Exception:
            return
        new_volume = max(0.0, min(2.0, new_volume))
        if abs(new_volume - self.volume) < 1e-3:
            return
        self.volume = new_volume
        self.get_logger().info(f"Voice volume set to {self.volume:.2f}")

    def _run_worker(self) -> None:
        while not self._stop_event.is_set():
            if not self._unpaused.wait(timeout=0.1):
                continue
            try:
                first = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            parts = [first]
            start = time.monotonic()
            while time.monotonic() - start < 0.35:
                try:
                    parts.append(self._queue.get_nowait())
                except queue.Empty:
                    break
            text = " ".join(segment.strip() for segment in parts if segment and segment.strip())
            if not text:
                continue

            provider = self._get_provider()
            if provider is None:
                self.get_logger().error("Skipping speech because no provider is available")
                continue

            self._publish_autophony_ms(0)
            try:
                success = provider.speak(text, volume=self.volume, on_duration=self._publish_autophony_ms)
            except ProviderUnavailable as exc:
                self.get_logger().error(f"TTS synthesis failed: {exc}")
                success = False
            except Exception as exc:
                self.get_logger().exception(f"Unexpected TTS failure: {exc}")
                success = False

            if not success:
                self._publish_autophony_ms(0)
                continue

            self._publish_conversation(text)
            self._publish_done(text)
            self._publish_autophony_ms(0)

    # ------------------------------------------------------------------
    # Publishing helpers
    def _publish_done(self, text: str) -> None:
        msg = String()
        msg.data = text
        try:
            self._pub_done.publish(msg)
        except Exception:
            pass

    def _publish_conversation(self, text: str) -> None:
        msg = MsgMessage()
        msg.role = "assistant"
        msg.content = text
        msg.speaker = "assistant"
        msg.confidence = 1.0
        try:
            msg.segments = []  # type: ignore[assignment]
            msg.words = []  # type: ignore[assignment]
        except Exception:
            pass
        try:
            self._pub_conversation.publish(msg)
        except Exception:
            pass

    def _publish_autophony_ms(self, duration_ms: int) -> None:
        msg = UInt32()
        msg.data = max(0, int(duration_ms))
        try:
            self.autophony_pub.publish(msg)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Timers and lifecycle
    def _announce_startup_message(self) -> None:
        greeting = self._get_string_param("startup_greeting", "")
        if greeting:
            self._queue.put(greeting)

    def _setup_ping_timer(self) -> None:
        if not self._get_bool_param("enable_ping", True):
            self._ping_timer = None
            return
        interval = max(5.0, float(self._get_int_param("ping_interval_sec", 30)))
        self._ping_timer = self.create_timer(interval, self._send_fortune_ping)

    def _send_fortune_ping(self) -> None:
        message = fetch_fortune_text()
        if not message:
            message = random.choice(FALLBACK_FORTUNES)
        self._queue.put(message)

    def destroy_node(self) -> None:
        self._stop_event.set()
        self._unpaused.set()
        self._stop_active_provider()
        if self._worker is not None:
            self._worker.join(timeout=5)
            self._worker = None
        if self._ping_timer is not None:
            try:
                self._ping_timer.cancel()
            except Exception:
                pass
        return super().destroy_node()

    def _stop_active_provider(self) -> None:
        with self._provider_lock:
            provider = self._provider
        if provider is None:
            return
        try:
            provider.stop()
        except Exception:
            pass


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = VoiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - entry point guard.
    main()
