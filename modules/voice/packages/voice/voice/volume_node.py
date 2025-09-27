#!/usr/bin/env python3
"""Publish and subscribe to /voice/volume and sync with system ALSA/Pulse volume.

This node listens for Float32 messages on /voice/volume and sets the system
playback volume using `amixer` or `pactl` when available. It also polls the
system mixer periodically and publishes the current volume to /voice/volume so
other components (like the pilot frontend) can reflect external changes.

Behavior contract:
- Input: std_msgs/Float32 on /voice/volume (0.0-1.0 normalized) sets system volume
- Output: std_msgs/Float32 publishes current system volume (0.0-1.0) on /voice/volume
- Error modes: missing system utilities => logs warning and operates in-memory
"""
from __future__ import annotations

import shutil
import subprocess
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class VoiceVolumeNode(Node):
    def __init__(self) -> None:
        super().__init__('voice_volume')

        # Topics
        self.topic = '/voice/volume'

        # Publisher and subscriber
        self._pub = self.create_publisher(Float32, self.topic, 10)
        self._sub = self.create_subscription(Float32, self.topic, self._on_set_volume, 10)

        # Polling interval for external/mixer changes
        self.declare_parameter('poll_interval_sec', 1.0)
        self.poll_interval = float(self.get_parameter('poll_interval_sec').get_parameter_value().double_value)

        # Internal state
        self._current_volume = self._read_system_volume()
        if self._current_volume is None:
            # default to 1.0 if we can't read system volume
            self._current_volume = 1.0

        # Start polling thread that publishes current value periodically
        self._stop = threading.Event()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

        self.get_logger().info(f'VoiceVolumeNode started, publishing {self.topic}')

    # --- System mixer helpers ---
    def _has_amixer(self) -> bool:
        return shutil.which('amixer') is not None

    def _has_pactl(self) -> bool:
        return shutil.which('pactl') is not None

    def _read_system_volume(self) -> Optional[float]:
        """Attempt to read the system playback volume (0.0-1.0)."""
        try:
            if self._has_amixer():
                # amixer get Master -> look for [xx%]
                out = subprocess.check_output(['amixer', 'get', 'Master'], text=True, stderr=subprocess.DEVNULL)
                # parse last occurrence of [NN%]
                import re
                m = re.findall(r"\[(\d{1,3})%\]", out)
                if m:
                    val = int(m[-1]) / 100.0
                    return max(0.0, min(1.0, val))
            if self._has_pactl():
                # pactl get-sink-volume @DEFAULT_SINK@ -> e.g. "Volume: front-left: 65536 / 100% / 0.00 dB"
                out = subprocess.check_output(['pactl', 'get-sink-volume', '@DEFAULT_SINK@'], text=True, stderr=subprocess.DEVNULL)
                import re
                m = re.findall(r"(\d{1,3})%", out)
                if m:
                    val = int(m[0]) / 100.0
                    return max(0.0, min(1.0, val))
        except Exception:
            pass
        return None

    def _set_system_volume(self, normalized: float) -> bool:
        """Set system volume given normalized float 0.0-1.0. Returns True if successful."""
        n = max(0.0, min(1.0, float(normalized)))
        try:
            percent = int(round(n * 100))
            if self._has_amixer():
                # amixer set Master 50%
                subprocess.check_call(['amixer', 'set', 'Master', f'{percent}%'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return True
            if self._has_pactl():
                # pactl set-sink-volume @DEFAULT_SINK@ 50%
                subprocess.check_call(['pactl', 'set-sink-volume', '@DEFAULT_SINK@', f'{percent}%'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return True
        except Exception as e:
            self.get_logger().warning(f'Failed to set system volume: {e}')
            return False
        # No mixer available
        return False

    # --- ROS callbacks and loop ---
    def _on_set_volume(self, msg: Float32) -> None:
        try:
            v = float(msg.data)
        except Exception:
            self.get_logger().warning('Received invalid volume message')
            return
        v = max(0.0, min(1.0, v))
        # If we can set system mixer, do so. Otherwise, update internal state and re-publish.
        ok = self._set_system_volume(v)
        if ok:
            # read back to ensure consistent (some systems may map differently)
            read_back = self._read_system_volume()
            if read_back is not None:
                v = read_back
        else:
            # update internal value so polls will at least publish something
            self._current_volume = v
        # Publish normalized current value
        out = Float32()
        out.data = float(v)
        try:
            self._pub.publish(out)
        except Exception:
            pass

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            interval = max(0.1, float(self.poll_interval))
            read = self._read_system_volume()
            if read is not None:
                if abs(read - self._current_volume) > 1e-3:
                    self._current_volume = read
                    msg = Float32()
                    msg.data = float(read)
                    try:
                        self._pub.publish(msg)
                    except Exception:
                        pass
            else:
                # If no system mixer, still publish internal value occasionally
                msg = Float32()
                msg.data = float(self._current_volume)
                try:
                    self._pub.publish(msg)
                except Exception:
                    pass
            self._stop.wait(interval)

    def destroy_node(self):
        self._stop.set()
        try:
            if self._poll_thread is not None:
                self._poll_thread.join(timeout=1.0)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceVolumeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
