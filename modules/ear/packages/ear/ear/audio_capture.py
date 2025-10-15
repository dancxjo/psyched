"""ROS 2 node that captures PCM audio frames from an external command."""

from __future__ import annotations

import shlex
import subprocess
import threading
from pathlib import Path
from typing import Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout, UInt8MultiArray

_DEFAULT_COMMAND = [
    "arecord",
    "--quiet",
    "-f",
    "S16_LE",
    "-r",
    "16000",
    "-c",
    "1",
    "-t",
    "raw",
]


def _normalize_command(value: object) -> list[str]:
    if isinstance(value, (list, tuple)):
        return [str(part) for part in value if str(part).strip()]
    if isinstance(value, str):
        return shlex.split(value)
    return []


def _set_flag(command: list[str], flag: str, value: str) -> None:
    if flag in command:
        index = command.index(flag)
        if index + 1 < len(command):
            command[index + 1] = value
        else:
            command.append(value)
    else:
        command.extend([flag, value])


class AudioCaptureNode(Node):
    """Spawn an external recorder and publish PCM chunks to a ROS topic."""

    def __init__(self) -> None:
        super().__init__("ear_audio_capture")
        self._audio_topic = str(self.declare_parameter("audio_topic", "/audio/raw").value)
        self._sample_rate = int(self.declare_parameter("sample_rate", 16000).value)
        self._channels = int(self.declare_parameter("channels", 1).value)
        self._chunk_duration_ms = float(self.declare_parameter("chunk_duration_ms", 20.0).value)
        self._chunk_size = int(self.declare_parameter("chunk_size", 0).value)
        self._restart_delay = float(self.declare_parameter("restart_delay", 1.0).value)
        reliability = str(self.declare_parameter("reliability", "best_effort").value).lower()
        command_param = self.declare_parameter("audio_source_command", "").value
        self._command = _normalize_command(command_param) or list(_DEFAULT_COMMAND)
        _set_flag(self._command, "-r", str(self._sample_rate))
        _set_flag(self._command, "-c", str(self._channels))
        if self._chunk_size <= 0:
            bytes_per_sample = 2
            samples_per_chunk = int(self._sample_rate * (self._chunk_duration_ms / 1000.0))
            samples_per_chunk = max(samples_per_chunk, 1)
            self._chunk_size = samples_per_chunk * bytes_per_sample * max(self._channels, 1)
        qos_profile = QoSProfile(depth=10)
        if reliability != "reliable":
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self._publisher = self.create_publisher(UInt8MultiArray, self._audio_topic, qos_profile)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run_loop, name="EarAudioCapture", daemon=True)
        self._thread.start()

    def _spawn_process(self) -> subprocess.Popen[bytes] | None:
        binary = Path(self._command[0])
        try:
            process = subprocess.Popen(
                self._command,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except FileNotFoundError:
            self.get_logger().error("Audio source command not found: %s", binary)
            return None
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().exception("Failed to start audio source: %s", exc)
            return None
        if process.stdout is None:
            self.get_logger().error("Audio source did not expose stdout")
            process.terminate()
            return None
        self.get_logger().info("Started audio source command: %s", " ".join(self._command))
        return process

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            process = self._spawn_process()
            if process is None:
                self._stop_event.wait(self._restart_delay)
                continue
            stdout = process.stdout
            assert stdout is not None
            while not self._stop_event.is_set():
                try:
                    chunk = stdout.read(self._chunk_size)
                except Exception as exc:  # pragma: no cover - defensive logging
                    self.get_logger().exception("Failed reading audio chunk: %s", exc)
                    break
                if not chunk:
                    return_code = process.poll()
                    if return_code is not None:
                        self.get_logger().warning("Audio source exited with code %s", return_code)
                    else:
                        self.get_logger().warning("Audio source produced empty chunk; restarting")
                    break
                msg = UInt8MultiArray()
                msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="pcm", size=len(chunk), stride=1)], data_offset=0)
                msg.data = list(chunk)
                self._publisher.publish(msg)
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()
            if self._stop_event.is_set():
                break
            self._stop_event.wait(self._restart_delay)
        self.get_logger().info("Audio capture loop stopped")

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2)
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AudioCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Audio capture interrupted; shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["AudioCaptureNode", "main"]
