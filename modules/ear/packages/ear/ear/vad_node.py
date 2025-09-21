#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, UInt32
import webrtcvad
import audioop
import time

class VADNode(Node):
    def __init__(self):
        super().__init__('vad_node')
        
        # VAD parameters
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(3)  # Aggressive mode
        self.target_sample_rate = 16000
        self.frame_duration = 30  # ms
        self.frame_size = int(self.target_sample_rate * (self.frame_duration / 1000.0) * 2)
        
        # Audio buffer
        self._buffer = b''
        
        # Speech tracking
        self.speech_start_time = None
        self.speech_duration = 0
        self.speech_segment = b''
        
        # Subscribers and Publishers
        self.audio_sub = self.create_subscription(ByteMultiArray, '/audio/raw', self.audio_callback, 10)
        self.speech_duration_pub = self.create_publisher(UInt32, '/audio/speech_duration', 10)
        self.speech_audio_pub = self.create_publisher(ByteMultiArray, '/audio/speech_segment', 10)

        self.get_logger().info("VAD node started.")

    def audio_callback(self, msg: ByteMultiArray):
        # The input is 44100 Hz, 16-bit, 1-channel. Resample to 16kHz for VAD.
        resampled_audio, _ = audioop.ratecv(msg.data, 2, 1, 44100, self.target_sample_rate, None)
        self._buffer += resampled_audio
        
        while len(self._buffer) >= self.frame_size:
            frame = self._buffer[:self.frame_size]
            self._buffer = self._buffer[self.frame_size:]
            
            is_speech = self.vad.is_speech(frame, self.target_sample_rate)
            
            current_time = time.time()

            if is_speech:
                if self.speech_start_time is None:
                    self.speech_start_time = current_time
                self.speech_segment += frame
            elif self.speech_start_time is not None:
                # Speech has just ended
                self.get_logger().info(f"Speech ended. Duration: {self.speech_duration}ms. Publishing segment.")
                
                # Publish the accumulated speech segment
                audio_msg = ByteMultiArray()
                audio_msg.data = self.speech_segment
                self.speech_audio_pub.publish(audio_msg)
                
                # Reset for next speech segment
                self.speech_start_time = None
                self.speech_segment = b''
                self.speech_duration = 0

            if self.speech_start_time is not None:
                # Update speech duration if speech is ongoing
                self.speech_duration = int((current_time - self.speech_start_time) * 1000)
            
            # Publish the current speech duration (0 if no speech)
            duration_msg = UInt32()
            duration_msg.data = self.speech_duration
            self.speech_duration_pub.publish(duration_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VADNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
