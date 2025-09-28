#!/usr/bin/env python3
"""PyAudio-based microphone capture node.

Captures raw PCM audio from microphone using PyAudio and publishes as ROS2 topics.
Fast, reliable, and simple.
"""
import struct
import threading
from typing import Optional

import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, UInt32

from .silence_tracker import SilenceTracker


class PyAudioEarNode(Node):
    def __init__(self) -> None:
        super().__init__('ear')
        
        # Audio parameters
        self.device_id = self.declare_parameter('device_id', 0).get_parameter_value().integer_value
        self.sample_rate = self.declare_parameter('sample_rate', 44100).get_parameter_value().integer_value
        self.channels = self.declare_parameter('channels', 1).get_parameter_value().integer_value
        self.chunk_size = self.declare_parameter('chunk_size', 1024).get_parameter_value().integer_value
        self.format = pyaudio.paInt16  # 16-bit PCM
        
        # Publishers
        self.audio_pub = self.create_publisher(ByteMultiArray, '/audio/raw', 10)
        self.silence_ms_pub = self.create_publisher(UInt32, '/audio/silence_ms', 10)
        
        # Audio processing
        self.audio = pyaudio.PyAudio()
        self.stream: Optional[pyaudio.Stream] = None
        self.running = False
        
        # Silence detection (RMS-based)
        self.silence_threshold = self.declare_parameter('silence_threshold', 500.0).get_parameter_value().double_value
        self.silence_tracker = SilenceTracker(self.silence_threshold)
        # Autophony tracking (dummy, replace with real logic if available)
        self.autophony_ms = 0
        
        # Start audio capture
        self.start_capture()
        
        self.get_logger().info(f'PyAudio Ear started: device={self.device_id}, rate={self.sample_rate}Hz, '
                              f'channels={self.channels}, chunk={self.chunk_size}, threshold={self.silence_threshold}')

    def start_capture(self) -> None:
        """Start PyAudio stream."""
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
                stream_callback=self.audio_callback
            )
            
            self.running = True
            self.stream.start_stream()
            
        except Exception as e:
            self.get_logger().error(f'Failed to start audio stream: {e}')
            self.shutdown()

    def audio_callback(self, in_data, frame_count, time_info, status):
        """PyAudio callback for processing audio data."""
        # paInputOverflow is status 2, which is a non-critical warning.
        if status and status != 2:
            self.get_logger().warn(f'Audio callback status: {status}')
        
        # Publish raw audio data as bytes
        audio_msg = ByteMultiArray()
        audio_msg.data = in_data  # in_data is already bytes
        self.audio_pub.publish(audio_msg)
        
        # Calculate RMS for silence detection
        rms = self.calculate_rms(in_data)
        # Optionally update autophony_ms here if you have logic for it
        # For now, assume self.autophony_ms is updated elsewhere or is 0
        self.update_silence_detector(rms, self.autophony_ms)
        
        return (in_data, pyaudio.paContinue)

    def calculate_rms(self, audio_data: bytes) -> float:
        """Calculate RMS (Root Mean Square) of audio data."""
        # Convert bytes to 16-bit integers
        samples = struct.unpack(f'<{len(audio_data)//2}h', audio_data)
        
        # Calculate RMS
        if len(samples) == 0:
            return 0.0
        
        sum_of_squares = sum(sample * sample for sample in samples)
        mean_square = sum_of_squares / len(samples)
        rms = mean_square ** 0.5
        
        return rms

    def update_silence_detector(self, rms: float, autophony_ms: int = 0) -> None:
        """Update silence detection based on RMS value and autophony_ms."""
        silence_ms = self.silence_tracker.update(rms=rms, autophony_ms=autophony_ms)
        # Publish silence duration
        silence_msg = UInt32()
        silence_msg.data = silence_ms
        self.silence_ms_pub.publish(silence_msg)

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