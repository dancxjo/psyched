#!/usr/bin/env python3
"""PyAudio-based microphone capture node.

Captures raw PCM audio from the configured device and publishes it verbatim on
``/audio/raw``.  Silence monitoring now lives in :mod:`ear.silence_node`, keeping
this node focused solely on the microphone interface.
"""
from typing import Optional
import time
from contextlib import suppress
import traceback

import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

from .qos import sensor_data_qos


class PyAudioEarNode(Node):
    def __init__(self) -> None:
        super().__init__('ear')
        
        # Audio parameters
        self.device_id = self.declare_parameter('device_id', 0).get_parameter_value().integer_value
        self.sample_rate = self.declare_parameter('sample_rate', 44100).get_parameter_value().integer_value
        self.channels = self.declare_parameter('channels', 1).get_parameter_value().integer_value
        self.chunk_size = self.declare_parameter('chunk_size', 2048).get_parameter_value().integer_value
        self.format = pyaudio.paInt16  # 16-bit PCM
        
        # Publisher for raw PCM payloads
        self.audio_pub = self.create_publisher(ByteMultiArray, '/audio/raw', sensor_data_qos())
        
        # Audio processing
        self.audio = pyaudio.PyAudio()
        self.stream: Optional[pyaudio.Stream] = None
        self.running = False
        
        # Start audio capture; if initial startup fails, shut down the node.
        if not self.start_capture(initial=True):
            # start_capture already logged the error; ensure node is stopped
            return

        # Counters / heartbeat for diagnosing stalls
        self._frames_published = 0
        self._last_heartbeat = time.time()
        self._last_frames = 0
        self._stall_count = 0
        # Callback timing / buffer diagnostics
        self._prev_callback_time: float | None = None
        self._last_callback_time: float | None = None
        self._last_frames_received: int = 0
        self._overflow_count: int = 0
        # Timer runs in the rclpy executor thread so logs are serialized with other node logs
        try:
            # create_timer may fail in non-ROS test environments; guard it
            self.create_timer(5.0, self._heartbeat)
        except Exception:
            pass

        self.get_logger().info(
            f'PyAudio Ear started: device={self.device_id}, rate={self.sample_rate}Hz, '
            f'channels={self.channels}, chunk={self.chunk_size}'
        )

    def start_capture(self, initial: bool = False) -> bool:
        """Start PyAudio stream.

        Returns True on success, False on failure. If `initial` is True then a
        failure is considered fatal and the caller may decide to stop the node.
        When called from the heartbeat restart path we avoid shutting down the
        whole node to allow repeated restart attempts and better diagnostics.
        """
        try:
            device_info = self.audio.get_device_info_by_index(self.device_id)
            self.get_logger().info(f'Using device: {device_info["name"]}')

            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                input_device_index=self.device_id,
                frames_per_buffer=self.chunk_size,
                stream_callback=self.audio_callback,
            )

            self.running = True
            self.stream.start_stream()
            return True

        except Exception as e:
            # Log full traceback to help diagnose ALSA / PyAudio errors
            tb = traceback.format_exc()
            try:
                self.get_logger().error(f'Failed to start audio stream: {type(e).__name__}: {e}')
                for line in tb.splitlines():
                    self.get_logger().error(line)
            except Exception:
                # Fallback print if logging fails
                print(f'Failed to start audio stream: {e}')
                print(tb)

            # If this is the initial startup failure, propagate by returning False.
            # Do NOT call shutdown() here when invoked from the heartbeat restart path.
            return False

    def audio_callback(self, in_data, frame_count, time_info, status):
        """PyAudio callback for processing audio data."""
        # paInputOverflow is status 2, which is a non-critical warning.
        if status:
            # paInputOverflow is status 2; count overflows separately
            try:
                if status == 2:
                    self._overflow_count += 1
                else:
                    self.get_logger().warn(f'Audio callback status: {status}')
            except Exception:
                pass

        # Defensive checks and guarded publish to prevent silent failures
        if not in_data:
            try:
                self.get_logger().warning('audio_callback received empty buffer')
            except Exception:
                pass
            return (in_data, pyaudio.paContinue)

        try:
            audio_msg = ByteMultiArray()
            audio_msg.data = in_data  # in_data is already bytes
            self.audio_pub.publish(audio_msg)
            # increment counter (safe from callback thread)
            try:
                self._frames_published += 1
            except Exception:
                # best-effort increment; don't let metric bookkeeping crash the callback
                pass
            # Record buffer size and timing diagnostics
            try:
                now = time.time()
                self._prev_callback_time = self._last_callback_time
                self._last_callback_time = now
                # compute frames received from bytes (16-bit samples)
                bytes_per_sample = 2  # paInt16
                frames_received = 0
                try:
                    frames_received = int(len(in_data) // (self.channels * bytes_per_sample))
                except Exception:
                    frames_received = 0
                self._last_frames_received = frames_received
            except Exception:
                pass
        except Exception as e:
            # Log publish errors but continue the stream
            try:
                self.get_logger().error(f'Failed to publish audio frame: {type(e).__name__}: {e}')
            except Exception:
                pass

        return (in_data, pyaudio.paContinue)

    def _heartbeat(self) -> None:
        """Periodic heartbeat to show publishing progress and detect stalls."""
        now = time.time()
        elapsed = now - self._last_heartbeat if self._last_heartbeat else 0.0
        frames = getattr(self, '_frames_published', 0)
        delta = frames - getattr(self, '_last_frames', 0)
        rate = (delta / elapsed) if elapsed > 0 else 0.0
        try:
            # Include callback/frame diagnostics when available
            frames_received = getattr(self, '_last_frames_received', 0)
            overflow = getattr(self, '_overflow_count', 0)
            cb_interval = None
            try:
                if self._prev_callback_time and self._last_callback_time:
                    cb_interval = self._last_callback_time - self._prev_callback_time
            except Exception:
                cb_interval = None

            extra = f' frames_received={frames_received} overflows={overflow}'
            if cb_interval:
                extra += f' cb_interval={cb_interval:.3f}s'

            self.get_logger().info(
                f'heartbeat: total_frames={frames} frames_in_last={delta} rate={rate:.2f}fps elapsed={elapsed:.1f}s' + extra
            )
        except Exception:
            pass
        # Debug: log PyAudio stream state to help diagnose callback stoppage
        try:
            if getattr(self, 'stream', None) is None:
                self.get_logger().warning('heartbeat: stream is None')
            else:
                try:
                    active = bool(self.stream.is_active())
                    stopped = bool(self.stream.is_stopped())
                    self.get_logger().debug(f'heartbeat: stream active={active} stopped={stopped}')
                except Exception as e:
                    self.get_logger().warning(f'heartbeat: failed to query stream state: {e}')
        except Exception:
            pass

        # Stall detection and simple auto-restart of stream
        try:
            if delta == 0:
                self._stall_count += 1
            else:
                self._stall_count = 0

            # If we've seen two consecutive empty intervals, attempt to restart the stream
            if self._stall_count >= 2:
                try:
                    self.get_logger().warning(
                        f'No audio frames for {self._stall_count * int(elapsed)}s (delta=0); attempting stream restart'
                    )
                except Exception:
                    pass

                # Attempt a safe restart: stop/close existing stream then reopen
                try:
                    if getattr(self, 'stream', None) is not None:
                        with suppress(Exception):
                            if self.stream.is_active():
                                self.stream.stop_stream()
                        with suppress(Exception):
                            self.stream.close()
                        self.stream = None
                except Exception:
                    pass

                # Try to (re)open the capture stream
                try:
                    # start_capture is safe to call multiple times; it will set stream
                    self.start_capture()
                except Exception as e:
                    try:
                        self.get_logger().error(f'Stream restart failed: {e}')
                    except Exception:
                        pass

                # reset stall counter to avoid aggressive restart loop
                self._stall_count = 0
        except Exception:
            pass
        self._last_heartbeat = now
        self._last_frames = frames

    def shutdown(self) -> None:
        """Clean shutdown of audio stream."""
        self.running = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()
        self.get_logger().info('Audio stream stopped')

    def destroy_node(self) -> None:
        """Override to ensure clean shutdown."""
        self.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PyAudioEarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
