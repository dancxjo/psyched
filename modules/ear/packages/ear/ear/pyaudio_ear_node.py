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

import subprocess
import threading
import time
from contextlib import suppress
from typing import Any, Callable, Dict, Iterable, Optional, Sequence

from .qos import sensor_data_qos

try:  # pragma: no cover - exercised only when ROS 2 is available
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import ByteMultiArray
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

        self.audio_pub = self.create_publisher(ByteMultiArray, self._topic, sensor_data_qos())

        self._proc: Optional[subprocess.Popen] = None
        self._stop_evt = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self._frames_published = 0
        self._last_heartbeat = time.time()
        self._last_frames = 0

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
                msg = ByteMultiArray()
                msg.data = bytes(data)
                self.audio_pub.publish(msg)
                self._frames_published += 1
            except Exception as exc:
                self.get_logger().error(f'Failed to publish audio frame: {exc}')

        self._cleanup_process()

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
