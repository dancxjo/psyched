#!/usr/bin/env python3
"""Queue speech from the ``voice`` topic using Piper TTS.

Piper is a high‑quality, lightweight neural TTS. This service reads text from
the ``voice`` topic, synthesizes speech with Piper, and plays audio via ALSA
(``aplay``).

Examples:
    Run the service::

        $ python3 node.py  # doctest: +SKIP

    Publish text::

        $ ros2 topic pub --once voice std_msgs/String '{data: "hello"}'  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import time
import json
import os
import queue
import subprocess
import threading
import pathlib
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

# Default Piper voice model (downloaded during provisioning)
PIPER_VOICES_DIR = os.getenv("PIPER_VOICES_DIR", "/opt/piper/voices")
PIPER_MODEL = os.getenv("PIPER_MODEL", "en_US-lessac-high")


@dataclass
class PiperSettings:
    voice_path: str
    sample_rate: int | None = None  # if None, use voice default
    use_cuda: bool = False


class VoiceNode(Node):
    """Speak queued messages via Piper TTS.

    Args:
        model: Piper voice model basename (e.g., ``"en_US-amy-medium"``).
        voices_dir: Directory containing ``.onnx`` and ``.onnx.json`` files.

    Examples:
        >>> VoiceNode(model="en_US-amy-medium")  # doctest: +SKIP
    """

    def __init__(self, model: str = PIPER_MODEL, voices_dir: str = PIPER_VOICES_DIR) -> None:
        super().__init__('voice')
        
        # Parameters - keep compatibility with existing node parameters
        self.declare_parameter('topic', '/voice')
        self.declare_parameter('voice_path', os.environ.get('PIPER_VOICE', ''))
        self.declare_parameter('use_cuda', False)
        self.declare_parameter('volume', 1.0)
        self.declare_parameter('length_scale', 1.0)
        self.declare_parameter('noise_scale', 0.667)
        self.declare_parameter('noise_w_scale', 0.8)
        self.declare_parameter('normalize_audio', True)
        self.declare_parameter('wav_out_dir', '')
        self.declare_parameter('aplay', True)

        # New voice model setup
        self.model = model
        self.voices_dir = voices_dir
        self.model_path = os.path.join(self.voices_dir, f"{self.model}.onnx")
        self.config_path = os.path.join(self.voices_dir, f"{self.model}.onnx.json")
        self.sample_rate = self._load_sample_rate(self.config_path)
        
        self._queue: queue.Queue[str] = queue.Queue()
        # Track both piper and aplay for interruption
        self._procs: list[subprocess.Popen] = []
        self._stop_event = threading.Event()
        self._unpaused = threading.Event()
        self._unpaused.set()  # start unpaused

        # Topic setup
        topic = self.get_parameter('topic').get_parameter_value().string_value
        
        # Publishers and subscribers
        self._pub_done = self.create_publisher(String, "voice_done", 10)
        self.create_subscription(String, topic, self.enqueue, 10)
        self.create_subscription(String, "voice_interrupt", self.interrupt, 10)
        
        # Control topics (compatibility)
        from std_msgs.msg import Empty
        self._pause_sub = self.create_subscription(Empty, '/voice/pause', self._on_pause, 1)
        self._resume_sub = self.create_subscription(Empty, '/voice/resume', self._on_resume, 1)
        self._clear_sub = self.create_subscription(Empty, '/voice/clear', self._on_clear, 1)

        # Ensure Piper model files exist; attempt runtime fetch if missing
        try:
            self._ensure_piper_model()
            # Reload sample rate if config just appeared
            self.sample_rate = self._load_sample_rate(self.config_path)
        except Exception:
            pass
            
        # Start worker thread
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()
        
        self.get_logger().info(f'Voice node listening on topic: {topic}')
        self.get_logger().info(f'Using Piper model: {self.model}')

    @staticmethod
    def _load_sample_rate(config_path: str) -> int:
        """Return sample rate from a Piper model JSON config.

        Falls back to 22050 Hz if unavailable.
        """
        try:
            with open(config_path, "r", encoding="utf-8") as fh:
                cfg = json.load(fh)
            # Typical locations: top-level or under "audio"
            return int(cfg.get("sample_rate") or cfg.get("audio", {}).get("sample_rate") or 22050)
        except Exception:
            return 22050

    def enqueue(self, msg: String) -> None:
        """Add ``msg`` to the speech queue."""
        self._queue.put(msg.data)

    def interrupt(self, msg: String | None = None) -> None:
        """Stop current speech and clear the queue."""
        for p in self._procs:
            try:
                p.terminate()
            except Exception:
                pass
        self._procs.clear()
        with self._queue.mutex:  # type: ignore[attr-defined]
            self._queue.queue.clear()  # type: ignore[attr-defined]

    def _ensure_piper_model(self) -> None:
        """Ensure the configured Piper model files are present.

        Downloads ``<model>.onnx`` and ``<model>.onnx.json`` into
        ``PIPER_VOICES_DIR`` when missing. Uses rhasspy/piper-voices URLs by
        default; can be overridden via ``PIPER_MODEL_URL`` and
        ``PIPER_CONFIG_URL`` env vars.
        """
        m = self.model
        voices = pathlib.Path(self.voices_dir)
        try:
            voices.mkdir(parents=True, exist_ok=True)
        except Exception:
            return
        model_url = os.getenv(
            "PIPER_MODEL_URL",
            f"https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/high/{m}.onnx?download=true",
        )
        cfg_url = os.getenv(
            "PIPER_CONFIG_URL",
            f"https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/high/{m}.onnx.json?download=true",
        )
        mp = pathlib.Path(self.model_path)
        cp = pathlib.Path(self.config_path)
        if not mp.exists():
            try:
                self.get_logger().info(f"Downloading Piper model: {m}.onnx")
                subprocess.run(["curl", "-fsSL", "-o", str(mp), model_url], check=True)
            except Exception as e:
                self.get_logger().warning(f"Failed to download model: {e}")
                pass
        if not cp.exists():
            try:
                self.get_logger().info(f"Downloading Piper config: {m}.onnx.json")
                subprocess.run(["curl", "-fsSL", "-o", str(cp), cfg_url], check=True)
            except Exception as e:
                self.get_logger().warning(f"Failed to download config: {e}")
                pass

    # Compatibility methods for existing control topics
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

    def destroy_node(self):
        """Clean shutdown."""
        self.interrupt()  # Stop any ongoing speech
        self._stop_event.set()
        self._unpaused.set()  # ensure worker can exit if paused
        self._worker.join(timeout=5)
        return super().destroy_node()

    def _run_worker(self):
        """Continuously speak queued messages with burst coalescing and subprocess pipeline."""
        import shutil
        
        # Check for both legacy piper-tts library and piper binary
        aplay_cmd = shutil.which('aplay') if self.get_parameter('aplay').value else None
        
        # Try to find piper binary with fallback locations
        piper_binary = shutil.which('piper')
        self.get_logger().info(f'piper binary search: shutil.which returned {piper_binary}')
        if not piper_binary:
            # Try common user installation locations as fallback
            fallback_paths = [
                '/usr/bin/piper',
                os.path.expanduser('~/.local/bin/piper'),
                '/usr/local/bin/piper',
                '/opt/piper/bin/piper'
            ]
            for path in fallback_paths:
                self.get_logger().info(f'Checking fallback path: {path}')
                if os.path.isfile(path) and os.access(path, os.X_OK):
                    piper_binary = path
                    self.get_logger().info(f'Found piper at fallback path: {path}')
                    break
                else:
                    exists = os.path.isfile(path)
                    executable = os.access(path, os.X_OK) if exists else False
                    self.get_logger().info(f'Fallback path {path}: exists={exists}, executable={executable}')
        
        self.get_logger().info(f'Final piper binary: {piper_binary}')
        self.get_logger().info(f'Model path: {self.model_path}, exists: {os.path.exists(self.model_path)}')
        self.get_logger().info(f'Config path: {self.config_path}, exists: {os.path.exists(self.config_path)}')
        wav_out_dir = self.get_parameter('wav_out_dir').value
        volume = float(self.get_parameter('volume').value)
        length_scale = float(self.get_parameter('length_scale').value)
        noise_scale = float(self.get_parameter('noise_scale').value)
        noise_w_scale = float(self.get_parameter('noise_w_scale').value)
        normalize_audio = bool(self.get_parameter('normalize_audio').value)

        # Setup synthesis config for legacy piper library
        syn_config = None
        if SynthesisConfig is not None:
            syn_config = SynthesisConfig(
                volume=volume,
                length_scale=length_scale,
                noise_scale=noise_scale,
                noise_w_scale=noise_w_scale,
                normalize_audio=normalize_audio,
            )

        # Try to load legacy piper voice if available
        legacy_voice = None
        voice_path = self.get_parameter('voice_path').get_parameter_value().string_value
        use_cuda = self.get_parameter('use_cuda').get_parameter_value().bool_value
        
        if PiperVoice is not None and voice_path:
            try:
                legacy_voice = PiperVoice.load(voice_path, use_cuda=use_cuda)
                self.get_logger().info(f'Loaded legacy piper voice: {voice_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load legacy piper voice at {voice_path}: {e}')

        while not self._stop_event.is_set():
            # Wait while paused, but remain responsive to shutdown
            if not self._unpaused.wait(timeout=0.1):
                continue
                
            try:
                # Coalesce bursts of short messages into one utterance to avoid
                # staccato reading (e.g., "It's. Reading. Like. This.")
                first = self._queue.get(timeout=0.1)
                parts = [first]
                start = time.monotonic()
                # Collect additional queued items for a short window
                try:
                    while time.monotonic() - start < 0.35 and not self._queue.empty():
                        try:
                            parts.append(self._queue.get_nowait())
                        except queue.Empty:
                            break
                except Exception:
                    pass
                    
                text = " ".join(s.strip() for s in parts if s and s.strip())
                
            except queue.Empty:
                continue

            self.get_logger().info(f'Speaking: {text}')

            # Try subprocess piper first (more efficient), fall back to legacy
            success = False
            
            if piper_binary and os.path.exists(self.model_path) and os.path.exists(self.config_path):
                success = self._speak_with_subprocess(text, aplay_cmd, piper_binary)
            
            if not success and legacy_voice is not None:
                success = self._speak_with_legacy(text, legacy_voice, syn_config, wav_out_dir, aplay_cmd)
                
            if not success:
                self.get_logger().warning('No working Piper installation found. Install piper binary or piper-tts library.')
                
            # Signal completion to upstream (e.g., chat service)
            try:
                done = String()
                done.data = text
                self._pub_done.publish(done)
            except Exception:
                pass

    def _speak_with_subprocess(self, text: str, aplay_cmd: str | None, piper_binary: str) -> bool:
        """Use subprocess piper binary for efficient speech synthesis."""
        try:
            # Launch Piper to produce RAW PCM to stdout, then pipe into aplay
            piper = subprocess.Popen(
                [
                    piper_binary,
                    "--model",
                    self.model_path,
                    "--config",
                    self.config_path,
                    "--output_raw",
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
            )
        except (FileNotFoundError, TypeError):
            return False
            
        if not aplay_cmd:
            return False
            
        # Prefer ALSA device from env; if PULSE_SERVER is set, use pulse plugin
        aplay_cmd_list = [
            aplay_cmd,
            "-q",
            "-r",
            str(self.sample_rate),
            "-f",
            "S16_LE",
            "-t",
            "raw",
        ]
        alsa_dev = os.getenv("ALSA_PCM")
        if alsa_dev:
            aplay_cmd_list += ["-D", alsa_dev]
        elif os.getenv("PULSE_SERVER"):
            aplay_cmd_list += ["-D", "pulse"]
        aplay_cmd_list += ["-"]
        
        aplay = subprocess.Popen(aplay_cmd_list, stdin=piper.stdout)
        self._procs = [aplay, piper]
        
        # Send text to Piper
        try:
            assert piper.stdin is not None
            piper.stdin.write((text + "\n").encode("utf-8"))
            piper.stdin.flush()
            piper.stdin.close()
        except Exception:
            pass
            
        # Wait for playback to finish
        aplay.wait()
        try:
            piper.terminate()
        except Exception:
            pass
        self._procs.clear()
        return True

    def _speak_with_legacy(self, text: str, voice, syn_config, wav_out_dir: str, aplay_cmd: str | None) -> bool:
        """Use legacy piper-tts library for speech synthesis."""
        try:
            if wav_out_dir:
                os.makedirs(wav_out_dir, exist_ok=True)
                tmp_path = os.path.join(wav_out_dir, 'out.wav')
                with wave.open(tmp_path, 'wb') as wav_file:
                    voice.synthesize_wav(text, wav_file, syn_config=syn_config)
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
                        voice.synthesize_wav(text, wav_file, syn_config=syn_config)
                    if aplay_cmd:
                        os.system(f"{aplay_cmd} '{tmp_path}'")
                    else:
                        self.get_logger().info(f'Generated WAV at {tmp_path} (no player)')
                finally:
                    try:
                        os.unlink(tmp_path)
                    except Exception:
                        pass
            return True
        except Exception as e:
            self.get_logger().error(f'Error during synthesis/playback: {e}')
            return False


def main(args=None):
    """Start the voice node.

    Respects ``PIPER_MODEL`` and ``PIPER_VOICES_DIR`` environment variables
    and ``--model``/``--voices-dir`` CLI arguments.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=os.getenv("PIPER_MODEL", PIPER_MODEL))
    parser.add_argument("--voices-dir", default=os.getenv("PIPER_VOICES_DIR", PIPER_VOICES_DIR))
    
    # Parse known args to handle ROS2 arguments
    known_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    node = VoiceNode(model=known_args.model, voices_dir=known_args.voices_dir)
    
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
