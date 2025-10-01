"""Raw audio publisher for ``/audio/raw`` using ``arecord``.

Despite the legacy name, this node now captures PCM16 audio via ``arecord``
from ALSA instead of PyAudio.  The change sidesteps the long-running issues we
observed with PyAudio streams silently stalling after a few minutes on the
robot.  The output contract remains the same: ``std_msgs/msg/ByteMultiArray``
messages on ``/audio/raw`` suitable for the existing silence monitor, VAD, and
segmenter nodes.

The implementation intentionally keeps the runtime dependencies minimal and the
control flow straightforward.  We spawn ``arecord`` in a background thread,
pipe its raw stdout into ROS messages, and restart the process if the stream
goes quiet or the subprocess exits.  A small shim layer emulates the ROS APIs
whenever the tests run outside a ROS installation so we can exercise the logic
with standard unit tests.
"""
from __future__ import annotations

import audioop
import subprocess
import threading
import time
import wave
from contextlib import suppress
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Optional, Sequence

import webrtcvad

from .audio_utils import coerce_pcm_bytes
from .qos import sensor_data_qos
from .segment_accumulator_node import SegmentAccumulator
from .segmenter_node import SpeechSegmenter

try:  # pragma: no cover - exercised only when ROS 2 is available
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import ByteMultiArray
    from psyched_msgs.msg import VadFrame
except ImportError:  # pragma: no cover - unit tests rely on these lightweight stubs
    rclpy = None  # type: ignore[assignment]

    class _LoggerStub:
        def info(self, msg: str) -> None:  # noqa: D401 - parity with rclpy logger
            """Log informational messages (ignored during tests)."""

        def warning(self, msg: str) -> None:
            pass

        # rclpy exposes ``warn``; keep compatibility with existing code.
        warn = warning

        def error(self, msg: str) -> None:
            pass

    class _PublisherStub:
        """Minimal publisher that records outbound messages for assertions."""

        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.published: list[Any] = []

        def publish(self, msg: Any) -> None:
            self.published.append(msg)

    class _TimerStub:
        def __init__(self, callback: Callable[[], None]) -> None:
            self.callback = callback

    class Node:  # type: ignore[override]
        """Lightweight stand-in for :class:`rclpy.node.Node` used in tests."""

        def __init__(self, name: str) -> None:
            self._name = name
            self._logger = _LoggerStub()
            self._parameters: Dict[str, Any] = {}

        def declare_parameter(self, name: str, default_value: Any) -> Any:
            self._parameters[name] = default_value
            return type("_Param", (), {"value": default_value})()

        def get_parameter(self, name: str) -> Any:
            value = self._parameters.get(name)
            return type("_Param", (), {"value": value})()

        def create_publisher(self, msg_type: Any, topic: str, qos: Any) -> _PublisherStub:
            return _PublisherStub(topic)

        def create_timer(self, interval: float, callback: Callable[[], None]) -> _TimerStub:
            return _TimerStub(callback)

        def get_logger(self) -> _LoggerStub:
            return self._logger

        def destroy_node(self) -> None:
            pass

    class ByteMultiArray:  # type: ignore[override]
        def __init__(self, data: Iterable[int] | bytes | bytearray | None = None) -> None:
            self.data = bytes(data or b"")

    @dataclass
    class VadFrame:  # type: ignore[override]
        stamp: Any = None
        sample_rate: int = 0
        frame_samples: int = 0
        is_speech: bool = False
        audio: bytes = b""


PopenFactory = Callable[..., subprocess.Popen]


class PyAudioEarNode(Node):  # type: ignore[misc]
    """Publish raw PCM16 audio frames on ``/audio/raw`` using ALSA ``arecord``."""

    def __init__(
        self,
        *,
        popen_factory: Optional[PopenFactory] = None,
        parameter_overrides: Optional[Dict[str, object]] = None,
    ) -> None:
        super().__init__('ear')

        overrides = parameter_overrides or {}
        self._popen_factory: PopenFactory = popen_factory or subprocess.Popen

        self._chunk_size = self._get_int_param('chunk_size', 2048, overrides)
        self._sample_rate = self._get_int_param('sample_rate', 44100, overrides)
        self._channels = self._get_int_param('channels', 1, overrides)
        self._device_id = self._get_int_param('device_id', 0, overrides)
        self._alsa_device = self._get_str_param('alsa_device', '', overrides)
        self._topic = self._get_str_param('topic', '/audio/raw', overrides)
        legacy_topic = self._get_str_param('vad_input_topic', self._topic, overrides)
        if legacy_topic and legacy_topic != self._topic:
            self._topic = legacy_topic

        segmenter_topic_default = self._get_str_param('segmenter_segment_topic', '/audio/speech_segment', overrides)
        segmenter_accum_default = self._get_str_param(
            'segmenter_accum_topic', '/audio/speech_segment_accumulating', overrides
        )
        speech_accum_default = self._get_str_param(
            'speech_accumulator_accum_topic', '/audio/speech_accumulating', overrides
        )

        self._segment_topic = self._get_str_param('segment_topic', segmenter_topic_default, overrides)
        self._segment_accum_topic = self._get_str_param(
            'segment_accum_topic', segmenter_accum_default, overrides
        )
        self._speech_accum_topic = self._get_str_param(
            'speech_accum_topic', speech_accum_default, overrides
        )

        self._segment_sample_rate = self._get_int_param('segment_sample_rate', 16000, overrides)
        self._vad_frame_ms = self._get_int_param('vad_frame_duration_ms', 30, overrides)
        self._vad_mode = max(0, min(3, self._get_int_param('vad_mode', 3, overrides)))

        silence_release = self._get_float_param('segmenter_silence_release_ms', 450.0, overrides)
        lead_silence = self._get_float_param('segmenter_lead_silence_ms', 120.0, overrides)
        min_speech = self._get_float_param('segmenter_min_speech_ms', 300.0, overrides)
        max_segment = self._get_float_param('segmenter_max_segment_ms', 12000.0, overrides)
        trim_window = self._get_float_param('segmenter_trim_window_ms', 30.0, overrides)
        trim_keep = self._get_float_param('segmenter_trim_keep_ms', 60.0, overrides)
        trim_ratio = self._get_float_param('segmenter_trim_rms_ratio', 0.12, overrides)
        trim_floor = self._get_float_param('segmenter_trim_rms_floor', 200.0, overrides)

        reset_timeout = self._get_float_param('speech_accumulator_reset_timeout', 12.0, overrides)
        max_segments = self._get_int_param('speech_accumulator_max_segments', 8, overrides)

        dump_dir_default = self._get_str_param('segment_dump_dir', 'log/ear_segments', overrides)
        self._segment_dump_enabled = self._get_bool_param('segment_dump_enabled', True, overrides)
        self._segment_dump_dir = Path(dump_dir_default).expanduser()
        if self._segment_dump_enabled:
            with suppress(Exception):
                self._segment_dump_dir.mkdir(parents=True, exist_ok=True)

        self.audio_pub = self.create_publisher(ByteMultiArray, self._topic, sensor_data_qos())
        self._segment_pub = self.create_publisher(ByteMultiArray, self._segment_topic, sensor_data_qos())
        self._segment_accum_pub = self.create_publisher(ByteMultiArray, self._segment_accum_topic, sensor_data_qos())
        self._speech_accum_pub = self.create_publisher(ByteMultiArray, self._speech_accum_topic, sensor_data_qos())

        self._vad = webrtcvad.Vad()
        self._vad.set_mode(self._vad_mode)

        max_segment_ms = max_segment if max_segment > 0 else None
        self._segmenter = SpeechSegmenter(
            silence_release_ms=silence_release,
            lead_silence_ms=lead_silence,
            min_speech_ms=min_speech,
            max_segment_ms=max_segment_ms,
            trim_window_ms=trim_window,
            trim_keep_ms=trim_keep,
            trim_rms_ratio=trim_ratio,
            trim_rms_floor=trim_floor,
        )
        self._segment_active = False
        self._segment_buffer = bytearray()
        self._segment_accumulator = SegmentAccumulator(
            reset_timeout=reset_timeout,
            max_segments=max_segments,
        )

        self._ratecv_state: Optional[tuple[Any, ...]] = None
        self._resample_buffer = bytearray()
        self._frame_bytes = max(1, int(self._segment_sample_rate * (self._vad_frame_ms / 1000.0)) * 2)
        self._segment_counter = 0

        self._proc: Optional[subprocess.Popen] = None
        self._stop_evt = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self._frames_published = 0
        self._last_heartbeat = time.time()
        self._last_frames = 0

        heartbeat_enabled = self.declare_parameter('heartbeat_logging', False)
        heartbeat_value = getattr(heartbeat_enabled, 'value', False)
        self._heartbeat_enabled = bool(heartbeat_value) if isinstance(heartbeat_value, bool) else str(heartbeat_value).lower() in {"1", "true", "yes", "on"}
        if self._heartbeat_enabled:
            try:
                self.create_timer(5.0, self._heartbeat)
            except Exception:
                pass

        self.get_logger().info(
            f'Arecord Ear started: device={self._resolve_device()} '
            f'rate={self._sample_rate}Hz channels={self._channels} '
            f'chunk={self._chunk_size} -> {self._topic}'
        )

    # ------------------------------------------------------------------
    # Parameter helpers
    def _get_param(self, name: str, default: Any, overrides: Dict[str, object]) -> Any:
        param = self.declare_parameter(name, default)
        value = getattr(param, 'value', default)
        if name in overrides:
            value = overrides[name]
        return value

    def _get_int_param(self, name: str, default: int, overrides: Dict[str, object]) -> int:
        value = self._get_param(name, default, overrides)
        try:
            return int(value)
        except Exception:
            return default

    def _get_str_param(self, name: str, default: str, overrides: Dict[str, object]) -> str:
        value = self._get_param(name, default, overrides)
        try:
            return str(value)
        except Exception:
            return default

    def _get_float_param(self, name: str, default: float, overrides: Dict[str, object]) -> float:
        value = self._get_param(name, default, overrides)
        try:
            return float(value)
        except Exception:
            return default

    def _get_bool_param(self, name: str, default: bool, overrides: Dict[str, object]) -> bool:
        value = self._get_param(name, default, overrides)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return default

    # ------------------------------------------------------------------
    # Capture management
    def _resolve_device(self) -> str:
        if self._alsa_device:
            return self._alsa_device
        return f'hw:{self._device_id},0'

    def _spawn_arecord(self) -> Optional[subprocess.Popen]:
        cmd = [
            'arecord',
            '-q',
            '-D', self._resolve_device(),
            '-f', 'S16_LE',
            '-r', str(self._sample_rate),
            '-c', str(self._channels),
            '-t', 'raw',
        ]
        try:
            proc = self._popen_factory(cmd, stdout=subprocess.PIPE)
            self.get_logger().info('Spawned arecord: %s' % ' '.join(cmd))
            return proc
        except FileNotFoundError:
            self.get_logger().error('arecord not found. Install `alsa-utils`.')
        except Exception as exc:
            self.get_logger().error(f'Failed to start arecord: {exc}')
        return None

    def _reader_loop(self) -> None:
        backoff = 0.5
        while not self._stop_evt.is_set():
            if self._proc is None or self._proc.poll() is not None:
                self._proc = self._spawn_arecord()
                if self._proc is None:
                    time.sleep(min(backoff, 5.0))
                    backoff = min(backoff * 2, 5.0)
                    continue
                backoff = 0.5
                if self._stop_evt.is_set():
                    break

            assert self._proc.stdout is not None
            try:
                data = self._proc.stdout.read(self._chunk_size)
            except Exception as exc:
                self.get_logger().error(f'Error reading from arecord: {exc}')
                self._restart_process()
                time.sleep(0.5)
                continue

            if self._stop_evt.is_set():
                break

            if not data:
                self.get_logger().warning('arecord produced no data; restarting...')
                self._restart_process()
                time.sleep(0.2)
                continue

            try:
                self._handle_raw_chunk(data)
            except Exception as exc:
                self.get_logger().error(f'Failed to process audio frame: {exc}')

        self._cleanup_process()

    def _handle_raw_chunk(self, data: bytes) -> None:
        pcm = bytes(data)
        if not pcm:
            return

        msg = ByteMultiArray()
        msg.data = pcm
        self.audio_pub.publish(msg)
        self._frames_published += 1

        mono = pcm
        if self._channels > 1:
            try:
                mono = audioop.tomono(pcm, 2, 0.5, 0.5)
            except Exception:
                mono = pcm

        try:
            resampled, self._ratecv_state = audioop.ratecv(
                mono,
                2,
                1,
                self._sample_rate,
                self._segment_sample_rate,
                self._ratecv_state,
            )
        except Exception as exc:
            self.get_logger().warning(f'Failed to resample audio chunk: {exc}')
            return

        if not resampled:
            return

        self._resample_buffer.extend(resampled)
        self._drain_resample_buffer()

    def _drain_resample_buffer(self) -> None:
        frame_bytes = self._frame_bytes
        while len(self._resample_buffer) >= frame_bytes:
            frame = bytes(self._resample_buffer[:frame_bytes])
            del self._resample_buffer[:frame_bytes]
            self._process_frame(frame)

    def _process_frame(self, frame: bytes) -> None:
        samples = len(frame) // 2
        if samples <= 0:
            return

        try:
            is_speech = bool(self._vad.is_speech(frame, self._segment_sample_rate))
        except Exception as exc:
            self.get_logger().warning(f'VAD failure, treating frame as silence: {exc}')
            is_speech = False

        msg = VadFrame()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sample_rate = self._segment_sample_rate
        msg.frame_samples = samples
        msg.is_speech = is_speech
        msg.audio = frame

        self._handle_vad_update(msg, frame, is_speech)

    def _handle_vad_update(self, frame_msg: VadFrame, pcm: bytes, is_speech: bool) -> None:
        prev_active = self._segment_active
        update = self._segmenter.process_frame(frame_msg)

        if is_speech and pcm:
            self._segment_buffer.extend(pcm)
            accum_msg = ByteMultiArray()
            accum_msg.data = bytes(self._segment_buffer)
            self._segment_accum_pub.publish(accum_msg)

        active = bool(update.active)
        segment_payload: Optional[bytes] = None

        if update.segment is not None:
            segment_payload = bytes(update.segment)
        elif prev_active and not active and self._segment_buffer:
            segment_payload = bytes(self._segment_buffer)

        if segment_payload:
            self._emit_segment(segment_payload)
        elif not is_speech and not active:
            self._segment_buffer.clear()

        self._segment_active = active

    def _emit_segment(self, segment: bytes) -> None:
        pcm = coerce_pcm_bytes(segment)
        if not pcm:
            return

        msg = ByteMultiArray()
        msg.data = pcm
        self._segment_pub.publish(msg)
        self._segment_buffer.clear()

        combined = self._segment_accumulator.add_segment(pcm)
        if combined:
            combined_msg = ByteMultiArray()
            combined_msg.data = bytes(combined)
            self._speech_accum_pub.publish(combined_msg)

        self._write_segment_wav(pcm)

    def _write_segment_wav(self, pcm: bytes) -> None:
        if not self._segment_dump_enabled or not pcm:
            return
        try:
            timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            self._segment_counter += 1
            filename = f'{timestamp}_{self._segment_counter:05d}.wav'
            target = self._segment_dump_dir / filename
            with wave.open(str(target), 'wb') as handle:
                handle.setnchannels(1)
                handle.setsampwidth(2)
                handle.setframerate(self._segment_sample_rate)
                handle.writeframes(pcm)
        except Exception as exc:
            self.get_logger().warning(f'Failed to write segment WAV: {exc}')
    def _restart_process(self) -> None:
        if self._proc is None:
            return
        with suppress(Exception):
            if self._proc.poll() is None:
                self._proc.terminate()
        with suppress(Exception):
            if self._proc.stdout:
                self._proc.stdout.close()
        self._proc = None

    def _cleanup_process(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        with suppress(Exception):
            if proc.poll() is None:
                proc.terminate()
        with suppress(Exception):
            if proc.poll() is None:
                proc.kill()
        with suppress(Exception):
            if proc.stdout:
                proc.stdout.close()

    # ------------------------------------------------------------------
    # Diagnostics
    def _heartbeat(self) -> None:
        if not getattr(self, '_heartbeat_enabled', False):
            return
        now = time.time()
        elapsed = now - self._last_heartbeat
        frames = self._frames_published
        delta = frames - getattr(self, '_last_frames', 0)
        rate = (delta / elapsed) if elapsed > 0 else 0.0
        self._last_frames = frames
        self._last_heartbeat = now
        try:
            self.get_logger().info(
                f'heartbeat: total_frames={frames} frames_in_last={delta} '
                f'rate={rate:.2f}fps elapsed={elapsed:.1f}s'
            )
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Shutdown
    def destroy_node(self) -> None:
        self._stop_evt.set()
        if self._reader_thread.is_alive():
            with suppress(Exception):
                self._reader_thread.join(timeout=3)
        self._cleanup_process()
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:  # pragma: no cover - requires ROS
    if rclpy is None:
        raise RuntimeError('rclpy is required to run PyAudioEarNode')

    rclpy.init(args=args)
    node = PyAudioEarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['PyAudioEarNode', 'main', 'ByteMultiArray']
