#!/usr/bin/env python3
"""Queue speech from the ``voice`` topic using a TTS engine.

Supports:
- Piper (neural, requires model files)
- espeak-ng (fast/robust, supports MBROLA voices)

This service reads text from the ``voice`` topic, synthesizes speech with the
selected engine, and plays audio via ALSA (``aplay``) when available.

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
import sys
import shlex
import shutil

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import UInt32

from voice.utils import fetch_fortune_text

try:
    from piper import PiperVoice
    from piper import SynthesisConfig
except Exception:  # pragma: no cover - allow import failure until deps installed
    PiperVoice = None  # type: ignore
    SynthesisConfig = None  # type: ignore

# Engine defaults
ENGINE = os.getenv("VOICE_ENGINE", "piper")  # piper | espeak

# Default Piper voice model (downloaded during provisioning)
PIPER_VOICES_DIR = os.getenv("PIPER_VOICES_DIR", "/opt/piper/voices")
PIPER_MODEL = os.getenv("PIPER_MODEL", "en_US-ryan-high")

# Default espeak-ng voice (use MBROLA voice like 'mb-us1' if installed)
ESPEAK_VOICE = os.getenv("ESPEAK_VOICE", "en-us")


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
        # Engine selection: 'piper' or 'espeak'
        self.declare_parameter('engine', os.environ.get('VOICE_ENGINE', 'piper'))
        self.declare_parameter('voice_path', os.environ.get('PIPER_VOICE', ''))
        self.declare_parameter('use_cuda', False)
        self.declare_parameter('volume', 1.0)
        self.declare_parameter('length_scale', 1.0)
        self.declare_parameter('noise_scale', 0.667)
        self.declare_parameter('noise_w_scale', 0.8)
        self.declare_parameter('normalize_audio', True)
        self.declare_parameter('wav_out_dir', '')
        self.declare_parameter('aplay', True)
        # Startup/heartbeat messages
        self.declare_parameter('startup_greeting', os.environ.get('VOICE_STARTUP_GREETING', 'Hello, voice is online'))
        self.declare_parameter('enable_ping', True)
        self.declare_parameter('ping_interval_sec', 30)
        # espeak-ng parameters
        self.declare_parameter('espeak_voice', os.environ.get('ESPEAK_VOICE', 'en-us'))
        self.declare_parameter('espeak_rate', 170)   # words per minute
        self.declare_parameter('espeak_pitch', 50)   # 0-99
        self.declare_parameter('espeak_volume', 1.0) # 0.0-1.0 scales -a 0-200
        self.declare_parameter('espeak_extra_args', '')
        # Control topics (configurable)
        self.declare_parameter('pause_topic', '/voice/interrupt')  # treat interrupt as pause by default
        self.declare_parameter('resume_topic', '/voice/resume')
        self.declare_parameter('clear_topic', '/voice/clear')
        self.declare_parameter('interrupt_topic', '/voice/interrupt')

        # New voice model setup
        self.model = model
        self.voices_dir = voices_dir
        self.model_path = os.path.join(self.voices_dir, f"{self.model}.onnx")
        self.config_path = os.path.join(self.voices_dir, f"{self.model}.onnx.json")
        self.sample_rate = self._load_sample_rate(self.config_path)

        # Internal state
        self._queue: queue.Queue[str] = queue.Queue()
        # Track both piper and aplay for interruption
        self._procs: list[subprocess.Popen] = []
        self._stop_event = threading.Event()
        self._unpaused = threading.Event()
        self._unpaused.set()  # start unpaused
        self._worker: threading.Thread | None = None
        self._runtime_started = False
        # Runtime volume (0.0 - 2.0), initialized from parameter 'volume'
        try:
            self.volume = float(self.get_parameter('volume').value)
        except Exception:
            self.volume = 1.0
        self.volume = max(0.0, min(2.0, self.volume))

        # Topic setup
        self.topic = self.get_parameter('topic').get_parameter_value().string_value


        # Publishers and subscribers
        self._pub_done = self.create_publisher(String, 'voice_done', 10)
        self.autophony_pub = self.create_publisher(UInt32, '/audio/autophony_duration', 10)
        self.create_subscription(String, self.topic, self.enqueue, 10)
        # Back-compat legacy interrupt topic: treat as pause (do not clear queue)
        self.create_subscription(String, 'voice_interrupt', lambda _msg: self._on_pause(None), 10)

        # Control topics (configurable)
        pause_topic = self.get_parameter('pause_topic').get_parameter_value().string_value or '/voice/interrupt'
        resume_topic = self.get_parameter('resume_topic').get_parameter_value().string_value or '/voice/resume'
        clear_topic = self.get_parameter('clear_topic').get_parameter_value().string_value or '/voice/clear'
        interrupt_topic = self.get_parameter('interrupt_topic').get_parameter_value().string_value or '/voice/interrupt'

        self._pause_sub = self.create_subscription(Empty, pause_topic, self._on_pause, 1)
        self._resume_sub = self.create_subscription(Empty, resume_topic, self._on_resume, 1)
        self._clear_sub = self.create_subscription(Empty, clear_topic, self._on_clear, 1)
        # Preferred interrupt topic as Empty
        self._interrupt_sub = self.create_subscription(Empty, interrupt_topic, self._on_pause, 1)

        # Volume control topic (std_msgs/Float32)
        from std_msgs.msg import Float32
        self._volume_sub = self.create_subscription(Float32, '/voice/volume', self._on_volume, 1)

        # Bring up runtime worker immediately so the node is responsive before
        # any external configuration messages arrive.
        self._initialize_runtime()

    def _on_volume(self, msg) -> None:
        """Update the playback volume while keeping the worker alive."""
        self._initialize_runtime()
        try:
            new_volume = float(getattr(msg, 'data', 1.0))
        except Exception as exc:
            self.get_logger().warning(f'Failed to set volume: {exc}')
            return
        new_volume = max(0.0, min(2.0, new_volume))
        if abs(new_volume - self.volume) < 1e-3:
            return
        self.volume = new_volume
        self.get_logger().info(f'Voice volume set to {self.volume:.2f}')

    def _initialize_runtime(self) -> None:
        """Start background worker and announce readiness if not already running."""
        if self._runtime_started:
            return
        self._runtime_started = True

        try:
            engine = self.get_parameter('engine').get_parameter_value().string_value
        except Exception:
            engine = 'piper'

        if engine == 'piper':
            try:
                self._ensure_piper_model()
                # Reload sample rate if config just appeared
                self.sample_rate = self._load_sample_rate(self.config_path)
            except Exception as exc:
                self.get_logger().debug(f'Piper model check failed: {exc}')

        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

        self.get_logger().info(f'Voice node listening on topic: {self.topic}')
        self.get_logger().info(f'Using Piper model: {self.model}')

        self._announce_startup_message()
        self._setup_ping_timer()

    def _announce_startup_message(self) -> None:
        """Queue an initial utterance so operators know the node is alive."""
        try:
            fortune_text = fetch_fortune_text()
        except Exception:
            fortune_text = None
        if fortune_text:
            self.enqueue(String(data=fortune_text))
            return
        try:
            greeting = self.get_parameter('startup_greeting').get_parameter_value().string_value
        except Exception:
            greeting = ''
        if greeting:
            self.enqueue(String(data=greeting))

    def _setup_ping_timer(self) -> None:
        """Configure the optional periodic "ping" notification."""
        try:
            enable_ping = self.get_parameter('enable_ping').get_parameter_value().bool_value
            ping_interval = int(self.get_parameter('ping_interval_sec').get_parameter_value().integer_value or 30)
        except Exception:
            return
        if enable_ping and ping_interval > 0:
            self.create_timer(ping_interval, self._send_fortune_ping)

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
        # Stop any ongoing playback but keep queued items
        for p in self._procs:
            try:
                p.terminate()
            except Exception:
                pass
        self._procs.clear()
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

    def _send_fortune_ping(self) -> None:
        """Send a fortune as a periodic ping message."""
        try:
            fortune_text = fetch_fortune_text()
        except Exception:
            fortune_text = None
        message = fortune_text or 'I am here.'
        try:
            self.enqueue(String(data=message))
        except Exception:
            self.get_logger().debug('Failed to enqueue ping message', exc_info=True)

    def destroy_node(self):
        """Clean shutdown."""
        self.interrupt()  # Stop any ongoing speech
        self._stop_event.set()
        self._unpaused.set()  # ensure worker can exit if paused
        worker = self._worker
        if worker is not None:
            worker.join(timeout=5)
            self._worker = None
        return super().destroy_node()

    def _run_worker(self):
        """Continuously speak queued messages with burst coalescing and subprocess pipeline."""
        import shutil
        
        # Check for both legacy piper-tts library and piper binary
        aplay_cmd = shutil.which('aplay') if self.get_parameter('aplay').value else None
        engine = self.get_parameter('engine').get_parameter_value().string_value.lower().strip()
        self.get_logger().info(f'Voice engine: {engine}')

        # Use the Python 'piper' package when available; otherwise rely on
        # legacy PiperVoice or external binary handled elsewhere. We'll prefer
        # calling into the package API rather than spawning a separate piper
        # process.
        piper_available = False
        try:
            import importlib
            if importlib.util.find_spec('piper') is not None:
                piper_available = True
        except Exception:
            piper_available = False
        self.get_logger().info(f'Piper package available: {piper_available}')
        wav_out_dir = self.get_parameter('wav_out_dir').value
        volume = float(self.get_parameter('volume').value)
        length_scale = float(self.get_parameter('length_scale').value)
        noise_scale = float(self.get_parameter('noise_scale').value)
        noise_w_scale = float(self.get_parameter('noise_w_scale').value)
        normalize_audio = bool(self.get_parameter('normalize_audio').value)
        # espeak settings
        espeak_voice = self.get_parameter('espeak_voice').get_parameter_value().string_value
        espeak_rate = int(self.get_parameter('espeak_rate').get_parameter_value().integer_value or 170)
        espeak_pitch = int(self.get_parameter('espeak_pitch').get_parameter_value().integer_value or 50)
        espeak_volume = float(self.get_parameter('espeak_volume').get_parameter_value().double_value or 1.0)
        espeak_extra_args = self.get_parameter('espeak_extra_args').get_parameter_value().string_value

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

            # Engine-specific synthesis
            success = False
            # Autophony duration only published while audio is actively playing
            if engine == 'espeak':
                # espeak path; if aplay used, publish autophony during playback
                success = self._speak_with_espeak(text, aplay_cmd, espeak_voice, espeak_rate, espeak_pitch, espeak_volume, espeak_extra_args)
                # Ensure autophony resets to 0 (in case direct playback used)
                autophony_msg = UInt32()
                autophony_msg.data = 0
                self.autophony_pub.publish(autophony_msg)
            else:
                # Piper path
                if piper_available and os.path.exists(self.model_path) and os.path.exists(self.config_path):
                    success = self._speak_with_piper_api(text, aplay_cmd)
                    # Publish 0 autophony duration after playback
                    autophony_msg = UInt32()
                    autophony_msg.data = 0
                    self.autophony_pub.publish(autophony_msg)
                if not success and legacy_voice is not None:
                    success = self._speak_with_legacy(text, legacy_voice, syn_config, wav_out_dir, aplay_cmd)
                    autophony_msg = UInt32()
                    autophony_msg.data = 0
                    self.autophony_pub.publish(autophony_msg)
                if not success:
                    self.get_logger().warning('No working Piper installation found. Install piper binary or piper-tts library, or set engine=espeak.')

            # Signal completion to upstream (e.g., chat service)
            try:
                done = String()
                done.data = text
                self._pub_done.publish(done)
            except Exception:
                pass

    def _speak_with_subprocess(self, text: str, aplay_cmd: str | None, piper_cmd: list[str]) -> bool:
        """Use subprocess piper binary for efficient speech synthesis."""
        # Deprecated/path retained for compatibility but prefer the package API
        # This function is kept to avoid widespread call-site changes but will
        # attempt to use the Python package if available; otherwise return False.
        try:
            import importlib
            if importlib.util.find_spec('piper') is not None:
                return self._speak_with_piper_api(text, aplay_cmd)
        except Exception:
            return False
        return False

    def _speak_with_piper_api(self, text: str, aplay_cmd: str | None) -> bool:
        """Synthesize audio using the piper Python package API and play via aplay.

        This assumes the piper package exposes a high-level API to synthesize
        raw PCM bytes given model/config paths. Because piper may have different
        interfaces across versions, we try a few reasonable entry points.
        """
        try:
            import importlib
            spec = importlib.util.find_spec('piper')
            if spec is None:
                return False
            # Import the package
            import piper as piper_pkg
        except Exception:
            return False

        # Attempt common APIs: piper_pkg.synthesize_raw(model, config, text)
        # or piper_pkg.Synthesizer(...).synthesize(text)
        raw_pcm = None
        sample_rate = self.sample_rate
        try:
            # Preferred: synthesize_raw returning (pcm_bytes, sample_rate)
            if hasattr(piper_pkg, 'synthesize_raw'):
                out = piper_pkg.synthesize_raw(self.model_path, self.config_path, text)
                if isinstance(out, tuple) and len(out) >= 1:
                    raw_pcm = out[0]
                    if len(out) > 1 and isinstance(out[1], int):
                        sample_rate = out[1]
            # Next: function that returns bytes only
            elif hasattr(piper_pkg, 'synthesize'):
                raw_pcm = piper_pkg.synthesize(self.model_path, self.config_path, text)
            # Next: object-oriented API
            elif hasattr(piper_pkg, 'Piper'):
                try:
                    synth = piper_pkg.Piper(model=self.model_path, config=self.config_path)
                    raw_pcm = synth.synthesize_raw(text)
                except Exception:
                    synth = piper_pkg.Piper()
                    raw_pcm = synth.synthesize_raw(text)
            else:
                # Unknown API
                return False
        except Exception as e:
            self.get_logger().warning(f'Piper package synthesis failed: {e}')
            return False

        if not raw_pcm:
            return False

        # If the package returned a numpy array, convert to bytes
        try:
            import numpy as _np
            if isinstance(raw_pcm, _np.ndarray):
                # Ensure int16 little-endian
                if raw_pcm.dtype != _np.int16:
                    raw_pcm = (raw_pcm * 32767).astype(_np.int16)
                pcm_bytes = raw_pcm.tobytes()
            else:
                pcm_bytes = raw_pcm if isinstance(raw_pcm, (bytes, bytearray)) else bytes(raw_pcm)
        except Exception:
            pcm_bytes = raw_pcm if isinstance(raw_pcm, (bytes, bytearray)) else bytes(raw_pcm)

        # If no aplay, we can't play; fall back to writing WAV or returning success
        if not aplay_cmd:
            # Optionally write to wav_out_dir if configured
            wav_out_dir = self.get_parameter('wav_out_dir').value
            if wav_out_dir:
                try:
                    import tempfile
                    import wave as _wave
                    os.makedirs(wav_out_dir, exist_ok=True)
                    tmp_path = os.path.join(wav_out_dir, 'out.wav')
                    with _wave.open(tmp_path, 'wb') as wf:
                        wf.setnchannels(1)
                        wf.setsampwidth(2)
                        wf.setframerate(sample_rate)
                        wf.writeframes(pcm_bytes)
                    self.get_logger().info(f'Wrote WAV to {tmp_path}')
                    return True
                except Exception:
                    return False
            return False

        # Play PCM via aplay
        try:
            aplay_cmd_list = [aplay_cmd, '-q', '-r', str(sample_rate), '-f', 'S16_LE', '-t', 'raw', '-']
            alsa_dev = os.getenv('ALSA_PCM')
            if alsa_dev:
                aplay_cmd_list += ['-D', alsa_dev]
            elif os.getenv('PULSE_SERVER'):
                aplay_cmd_list += ['-D', 'pulse']

            p = subprocess.Popen(aplay_cmd_list, stdin=subprocess.PIPE)
            # Stream bytes
            p.stdin.write(pcm_bytes)
            p.stdin.close()
            self._procs = [p]
            start_time = time.time()
            while p.poll() is None:
                duration = int((time.time() - start_time) * 1000)
                autophony_msg = UInt32()
                autophony_msg.data = duration
                self.autophony_pub.publish(autophony_msg)
                time.sleep(0.1)
            p.wait()
            try:
                p.terminate()
            except Exception:
                pass
            self._procs.clear()
            return True
        except Exception as e:
            self.get_logger().warning(f'Failed to play synthesized audio: {e}')
            try:
                self._procs.clear()
            except Exception:
                pass
            return False

    def _validate_piper_cmd(self, cmd: list[str]) -> bool:
        """Validate a Piper command without synthesizing audio.

        Strategy:
        - Prefer running with ``--help`` (should be fast and rc=0)
        - If CLI insists on a model (some Python module wrappers do), retry
          with ``-m <model> [--config <config>] --help`` if files exist.
        - Reject if output shows missing module errors.
        """
        def _run_check(args: list[str]) -> tuple[int, str]:
            res = subprocess.run(args, capture_output=True, text=True, timeout=5)
            out = (res.stdout or "") + (res.stderr or "")
            return res.returncode, out

        try:
            # 1) Basic --help
            rc, out = _run_check([*cmd, "--help"])
            missing_mod = ("ModuleNotFoundError" in out) or ("No module named 'piper" in out)
            requires_model = ("the following arguments are required: -m/--model" in out) or ("required: -m/--model" in out)
            if rc == 0 and not missing_mod:
                return True

            # 2) Retry with model/config if present
            if (requires_model or rc != 0) and os.path.exists(self.model_path):
                args = [*cmd, "-m", self.model_path, "--help"]
                # Include config when available (harmless for most builds)
                if os.path.exists(self.config_path):
                    args = [*cmd, "-m", self.model_path, "--config", self.config_path, "--help"]
                rc2, out2 = _run_check(args)
                missing_mod2 = ("ModuleNotFoundError" in out2) or ("No module named 'piper" in out2)
                if rc2 == 0 and not missing_mod2:
                    return True
                self.get_logger().warning(
                    f"Piper command failed validation (rc={rc2}): {args} -> {out2.strip()}"
                )
                return False

            # Fallthrough: report first attempt
            if rc != 0 or missing_mod:
                self.get_logger().warning(
                    f"Piper command failed validation (rc={rc}): {[*cmd, '--help']} -> {out.strip()}"
                )
                return False
            return True
        except Exception as e:
            self.get_logger().warning(f"Piper command validation error for {cmd}: {e}")
            return False

    def _resolve_piper_cmd(self) -> list[str] | None:
        """Determine a working Piper invocation.

        Priority:
        1. PIPER_CMD env (space-split respecting quotes)
        2. Current Python: [sys.executable, '-m', 'piper'] if module is importable and validates
        3. System 'piper' binary from PATH or common locations
        """
        # 1) Explicit override
        override = os.getenv("PIPER_CMD")
        if override:
            try:
                cmd = shlex.split(override)
            except Exception:
                cmd = [override]
            self.get_logger().info(f"PIPER_CMD override provided: {cmd}")
            if self._validate_piper_cmd(cmd):
                return cmd
            else:
                self.get_logger().warning("PIPER_CMD override did not validate; will try other options")

        # 2) Prefer current Python environment to avoid system/site mismatch
        try:
            import importlib.util  # local import
            if importlib.util.find_spec("piper") is not None:
                py_cmd = [sys.executable, "-m", "piper"]
                if self._validate_piper_cmd(py_cmd):
                    self.get_logger().info("Using 'python -m piper' in current environment")
                    return py_cmd
        except Exception:
            pass

        # 3) Try to find a system binary
        import shutil
        piper_binary = shutil.which('piper')
        self.get_logger().info(f'piper binary search: shutil.which returned {piper_binary}')
        candidates: list[list[str]] = []
        if piper_binary:
            candidates.append([piper_binary])
        # Common fallbacks
        fallback_paths = [
            '/usr/bin/piper',
            os.path.expanduser('~/.local/bin/piper'),
            '/usr/local/bin/piper',
            '/opt/piper/bin/piper'
        ]
        for path in fallback_paths:
            if os.path.isfile(path) and os.access(path, os.X_OK):
                candidates.append([path])
            else:
                exists = os.path.isfile(path)
                executable = os.access(path, os.X_OK) if exists else False
                self.get_logger().info(f'Fallback path {path}: exists={exists}, executable={executable}')

        for cmd in candidates:
            if self._validate_piper_cmd(cmd):
                return cmd

        return None

    def _speak_with_espeak(
        self,
        text: str,
        aplay_cmd: str | None,
        voice: str,
        rate: int,
        pitch: int,
        volume_scale: float,
        extra_args: str = "",
    ) -> bool:
        """Synthesize speech using espeak-ng, optionally piping WAV to aplay.

        - voice: e.g., 'en-us' or MBROLA voice 'mb-us1' (requires espeak-ng-mbrola + mbrola-us1)
        - rate: words per minute (default ~170)
        - pitch: 0-99
        - volume_scale: 0.0-1.0 mapped to espeak -a 0-200
        - extra_args: raw string of additional arguments for espeak-ng
        """
        espeak = shutil.which('espeak-ng') or shutil.which('espeak')
        if not espeak:
            return False

        # Clamp and convert values
        rate = max(80, min(450, int(rate)))
        pitch = max(0, min(99, int(pitch)))
        amp = int(max(0.0, min(1.5, float(volume_scale))) * 200)

        # If runtime volume is set, override amplitude accordingly
        try:
            vol = max(0.0, min(1.5, float(getattr(self, 'volume', 1.0))))
            amp = int(vol * 200)
        except Exception:
            pass

        base_cmd = [espeak, '-v', voice, '-s', str(rate), '-p', str(pitch), '-a', str(amp)]
        if extra_args:
            try:
                base_cmd += shlex.split(extra_args)
            except Exception:
                # Fallback: append as single arg (harmless if invalid)
                base_cmd.append(extra_args)

        try:
            if aplay_cmd:
                # Stream WAV to aplay
                speak = subprocess.Popen([*base_cmd, '--stdout', text], stdout=subprocess.PIPE)
                aplay = subprocess.Popen([aplay_cmd, '-q', '-t', 'wav', '-'], stdin=speak.stdout)
                self._procs = [aplay, speak]
                # Publish autophony duration while speaking
                start_time = time.time()
                while aplay.poll() is None:
                    duration = int((time.time() - start_time) * 1000)
                    autophony_msg = UInt32()
                    autophony_msg.data = duration
                    self.autophony_pub.publish(autophony_msg)
                    time.sleep(0.1)
                aplay.wait()
                try:
                    speak.terminate()
                except Exception:
                    pass
                self._procs.clear()
                return True
            else:
                # Let espeak-ng play audio directly
                speak = subprocess.run([*base_cmd, text], capture_output=True, text=True)
                return speak.returncode == 0
        except Exception as e:
            self.get_logger().error(f'espeak-ng error: {e}')
            try:
                self._procs.clear()
            except Exception:
                pass
            return False

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
