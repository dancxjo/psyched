#!/usr/bin/env python3
import os
import queue
import threading
import wave
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

try:
    from piper import PiperVoice
    from piper import SynthesisConfig
except Exception:  # pragma: no cover - allow import failure until deps installed
    PiperVoice = None  # type: ignore
    SynthesisConfig = None  # type: ignore


@dataclass
class PiperSettings:
    voice_path: str
    sample_rate: int | None = None  # if None, use voice default
    use_cuda: bool = False


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        # Parameters
    self.declare_parameter('topic', '/voice')
        self.declare_parameter('voice_path', os.environ.get('PIPER_VOICE', ''))
        self.declare_parameter('use_cuda', False)
        self.declare_parameter('volume', 1.0)
        self.declare_parameter('length_scale', 1.0)
        self.declare_parameter('noise_scale', 0.667)
        self.declare_parameter('noise_w_scale', 0.8)
        self.declare_parameter('normalize_audio', True)
        self.declare_parameter('wav_out_dir', '')  # if set, writes wav files instead of playing
        self.declare_parameter('aplay', True)  # try to play via aplay if available

        topic = self.get_parameter('topic').get_parameter_value().string_value
        voice_path = self.get_parameter('voice_path').get_parameter_value().string_value
        use_cuda = self.get_parameter('use_cuda').get_parameter_value().bool_value

    self._queue: queue.Queue[str] = queue.Queue()
        self._stop_event = threading.Event()
    self._unpaused = threading.Event()
    self._unpaused.set()  # start unpaused
        self._worker = threading.Thread(target=self._run_worker, daemon=True)

    self.subscription = self.create_subscription(String, topic, self._on_text, 10)
    # Control topics
    from std_msgs.msg import Empty  # lazy import to keep top tidy
    self._pause_sub = self.create_subscription(Empty, '/voice/pause', self._on_pause, 1)
    self._resume_sub = self.create_subscription(Empty, '/voice/resume', self._on_resume, 1)
    self._clear_sub = self.create_subscription(Empty, '/voice/clear', self._on_clear, 1)

        self._voice: PiperVoice | None = None
        if PiperVoice is None:
            self.get_logger().warning('piper-tts not installed yet. Install with: pip install piper-tts')
        else:
            if voice_path:
                try:
                    self._voice = PiperVoice.load(voice_path, use_cuda=use_cuda)
                    self.get_logger().info(f'Loaded piper voice: {voice_path}')
                except Exception as e:
                    self.get_logger().error(f'Failed to load piper voice at {voice_path}: {e}')
            else:
                self.get_logger().warn('No voice_path provided. Set param voice_path or env PIPER_VOICE to an .onnx file')

        self._worker.start()
        self.get_logger().info(f'Voice node listening on topic: {topic}')

    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self._queue.put(text)
        self.get_logger().debug(f'Enqueued text: {text}')

    def destroy_node(self):
        self._stop_event.set()
        self._unpaused.set()  # ensure worker can exit if paused
        self._worker.join(timeout=5)
        return super().destroy_node()

    # Control callbacks
    def _on_pause(self, _msg) -> None:
        if self._unpaused.is_set():
            self.get_logger().info('Voice paused')
        self._unpaused.clear()

    def _on_resume(self, _msg) -> None:
        if not self._unpaused.is_set():
            self.get_logger().info('Voice resumed')
        self._unpaused.set()

    def _on_clear(self, _msg) -> None:
        cleared = 0
        try:
            while True:
                self._queue.get_nowait()
                cleared += 1
        except queue.Empty:
            pass
        self.get_logger().info(f'Cleared {cleared} queued utterance(s)')

    def _run_worker(self):
        import shutil
        aplay_cmd = shutil.which('aplay') if self.get_parameter('aplay').value else None
        wav_out_dir = self.get_parameter('wav_out_dir').value
        volume = float(self.get_parameter('volume').value)
        length_scale = float(self.get_parameter('length_scale').value)
        noise_scale = float(self.get_parameter('noise_scale').value)
        noise_w_scale = float(self.get_parameter('noise_w_scale').value)
        normalize_audio = bool(self.get_parameter('normalize_audio').value)

        syn_config = None
        if SynthesisConfig is not None:
            syn_config = SynthesisConfig(
                volume=volume,
                length_scale=length_scale,
                noise_scale=noise_scale,
                noise_w_scale=noise_w_scale,
                normalize_audio=normalize_audio,
            )

        while not self._stop_event.is_set():
            # Wait while paused, but remain responsive to shutdown
            if not self._unpaused.wait(timeout=0.1):
                continue
            try:
                text = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            self.get_logger().info(f'Speaking: {text}')

            if self._voice is None:
                self.get_logger().warning('No Piper voice loaded. Skipping speech.')
                continue

            try:
                if wav_out_dir:
                    os.makedirs(wav_out_dir, exist_ok=True)
                    tmp_path = os.path.join(wav_out_dir, 'out.wav')
                    with wave.open(tmp_path, 'wb') as wav_file:
                        self._voice.synthesize_wav(text, wav_file, syn_config=syn_config)
                    self.get_logger().info(f'Wrote WAV to {tmp_path}')
                    if aplay_cmd:
                        os.system(f"{aplay_cmd} '{tmp_path}'")
                else:
                    # Write to a temp wav and play if aplay exists
                    import tempfile
                    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tf:
                        tmp_path = tf.name
                    try:
                        with wave.open(tmp_path, 'wb') as wav_file:
                            self._voice.synthesize_wav(text, wav_file, syn_config=syn_config)
                        if aplay_cmd:
                            os.system(f"{aplay_cmd} '{tmp_path}'")
                        else:
                            self.get_logger().info(f'Generated WAV at {tmp_path} (no player)')
                    finally:
                        try:
                            os.unlink(tmp_path)
                        except Exception:
                            pass
            except Exception as e:
                self.get_logger().error(f'Error during synthesis/playback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        # Use multithreaded executor so sub + worker can coexist if callback blocks briefly
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
