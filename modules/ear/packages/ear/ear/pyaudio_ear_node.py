#!/usr/bin/env python3
"""PyAudio-based microphone capture node.

Captures raw PCM audio from the configured device and publishes it verbatim on
``/audio/raw``.  Silence monitoring now lives in :mod:`ear.silence_node`, keeping
this node focused solely on the microphone interface.
"""
from typing import Optional

import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray


class PyAudioEarNode(Node):
    def __init__(self) -> None:
        super().__init__('ear')
        
        # Audio parameters
        self.device_id = self.declare_parameter('device_id', 0).get_parameter_value().integer_value
        self.sample_rate = self.declare_parameter('sample_rate', 44100).get_parameter_value().integer_value
        self.channels = self.declare_parameter('channels', 1).get_parameter_value().integer_value
        self.chunk_size = self.declare_parameter('chunk_size', 1024).get_parameter_value().integer_value
        self.format = pyaudio.paInt16  # 16-bit PCM
        
        # Publisher for raw PCM payloads
        self.audio_pub = self.create_publisher(ByteMultiArray, '/audio/raw', 10)
        
        # Audio processing
        self.audio = pyaudio.PyAudio()
        self.stream: Optional[pyaudio.Stream] = None
        self.running = False
        
        # Start audio capture
        self.start_capture()

        self.get_logger().info(
            f'PyAudio Ear started: device={self.device_id}, rate={self.sample_rate}Hz, '
            f'channels={self.channels}, chunk={self.chunk_size}'
        )

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
        
        return (in_data, pyaudio.paContinue)

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