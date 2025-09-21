#!/usr/bin/env python3
"""Microphone PCM publisher.

Captures PCM16 LE audio from ALSA via `arecord` and publishes it on a ROS 2
topic using `audio_common_msgs/AudioData`.

Defaults: mono 16 kHz S16_LE from ALSA device `default` to `/audio/pcm`.

This intentionally uses a lightweight subprocess approach to avoid Python
audio stack dependencies and stay robust on headless systems.
"""
from __future__ import annotations

import os
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import ParameterType
from audio_common_msgs.msg import AudioData


class EarNode(Node):
    def __init__(self) -> None:
        super().__init__('ear')

        # Parameters
        self.declare_parameter('topic', '/audio/pcm')
        self.declare_parameter('device', os.environ.get('ALSA_CAPTURE_DEVICE', 'default'))
        self.declare_parameter('rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk', 2048)  # bytes per message
        self.declare_parameter('publish_info', True)
        self.declare_parameter('info_topic', '/audio/pcm/info')

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value

        def _as_int(name: str, default: int) -> int:
            pv = self.get_parameter(name).get_parameter_value()
            # ParameterValue can carry integer_value or string_value depending on launch
            if pv.type == ParameterType.PARAMETER_INTEGER:
                return int(pv.integer_value)
            if pv.type == ParameterType.PARAMETER_STRING and pv.string_value:
                try:
                    return int(pv.string_value)
                except Exception:
                    pass
            # Try double as well
            if pv.type == ParameterType.PARAMETER_DOUBLE:
                try:
                    return int(pv.double_value)
                except Exception:
                    pass
            return default

        self.rate = _as_int('rate', 16000)
        self.channels = _as_int('channels', 1)
        self.chunk = _as_int('chunk', 2048)

        # Publishers
        self.pub = self.create_publisher(AudioData, self.topic, 10)
        # Optional AudioInfo publisher; import lazily
        self._info_enabled = bool(self.get_parameter('publish_info').value)
        self._info_pub = None
        self._info_msg_type = None
        if self._info_enabled:
            try:
                from audio_common_msgs.msg import AudioInfo  # type: ignore
                self._info_msg_type = AudioInfo
                self._info_pub = self.create_publisher(AudioInfo, self.get_parameter('info_topic').value, 1)
            except Exception:
                self._info_enabled = False

        # Arecord process state
        self._proc = None  # type: ignore[assignment]
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"Ear started: device={self.device} rate={self.rate}Hz ch={self.channels} chunk={self.chunk} -> {self.topic}"
        )
        # Optionally publish an info message periodically for consumers
        if self._info_enabled:
            self._publish_info()
            self.create_timer(5.0, self._publish_info)

    def destroy_node(self):
        self._stop_evt.set()
        if self._proc and self._proc.poll() is None:
            try:
                self._proc.terminate()
            except Exception:
                pass
        try:
            self._thread.join(timeout=3)
        except Exception:
            pass
        return super().destroy_node()

    def _spawn_arecord(self) -> Optional[subprocess.Popen]:
        """Start `arecord` producing raw PCM S16_LE on stdout."""
        cmd = [
            'arecord',
            '-q',              # quiet
            '-D', self.device, # ALSA device
            '-f', 'S16_LE',    # 16-bit signed little-endian
            '-r', str(self.rate),
            '-c', str(self.channels),
            '-t', 'raw',       # raw output to stdout
        ]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            self.get_logger().info(f"Spawned arecord: {' '.join(cmd)}")
            return proc
        except FileNotFoundError:
            self.get_logger().error('arecord not found. Install `alsa-utils`.')
            return None
        except Exception as e:
            self.get_logger().error(f'Failed to start arecord: {e}')
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

            assert self._proc.stdout is not None
            try:
                data = self._proc.stdout.read(self.chunk)
                if not data:
                    # EOF or device issue; restart
                    self.get_logger().warning('arecord produced no data; restarting...')
                    try:
                        self._proc.terminate()
                    except Exception:
                        pass
                    self._proc = None
                    time.sleep(0.2)
                    continue
                msg = AudioData()
                msg.data = bytes(data)
                self.pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error reading from arecord: {e}')
                try:
                    if self._proc and self._proc.poll() is None:
                        self._proc.terminate()
                except Exception:
                    pass
                self._proc = None
                time.sleep(0.5)

    def _publish_info(self) -> None:
        if not self._info_enabled or self._info_pub is None or self._info_msg_type is None:
            return
        try:
            msg = self._info_msg_type()
            msg.sample_rate = int(self.rate)
            msg.channels = int(self.channels)
            if hasattr(msg, 'sample_format'):
                msg.sample_format = 'S16LE'
            self._info_pub.publish(msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = EarNode()
    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
