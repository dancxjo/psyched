#!/usr/bin/env python3
"""Queue speech from the ``voice`` topic using a TTS engine.

Supports:
- espeak-ng (fast/robust, supports MBROLA voices)

This service reads text from the ``voice`` topic, synthesizes speech with espeak-ng,
and plays audio via ALSA (``aplay``) when available.

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

# Engine defaults (Piper removed; use espeak)
ENGINE = os.getenv("VOICE_ENGINE", "espeak")  # espeak
# Default espeak-ng voice (use MBROLA voice like 'mb-us1' or 'en1' if installed)
# Use MBROLA `en1` as the workspace default when available
ESPEAK_VOICE = os.getenv("ESPEAK_VOICE", "mb-en1")

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import UInt32

class VoiceNode(Node):
                            """Speak queued messages via Piper TTS.

                            Args:
                                model: Piper voice model basename (e.g., ``"en_US-amy-medium"``).
                                voices_dir: Directory containing ``.onnx`` and ``.onnx.json`` files.

                            Examples:
                                >>> VoiceNode(model="en_US-amy-medium")  # doctest: +SKIP
                            """

                            def __init__(self) -> None:
                                super().__init__('voice')

                                # Parameters - keep compatibility with existing node parameters
                                self.declare_parameter('topic', '/voice')
                                # Engine selection: 'piper' or 'espeak'
                                # Piper has been intentionally removed; default to espeak
                                self.declare_parameter('engine', os.environ.get('VOICE_ENGINE', 'espeak'))
                                # Runtime volume (0.0 - 2.0) - ensure it's stored as a float
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
                                self.declare_parameter('espeak_voice', os.environ.get('ESPEAK_VOICE', ESPEAK_VOICE))
                                self.declare_parameter('espeak_rate', 170)   # words per minute
                                self.declare_parameter('espeak_pitch', 50)   # 0-99
                                self.declare_parameter('espeak_volume', 1.0) # 0.0-1.0 scales -a 0-200
                                self.declare_parameter('espeak_extra_args', '')
                                # Control topics (configurable)
                                self.declare_parameter('pause_topic', '/voice/interrupt')  # treat interrupt as pause by default
                                self.declare_parameter('resume_topic', '/voice/resume')
                                self.declare_parameter('clear_topic', '/voice/clear')
                                self.declare_parameter('interrupt_topic', '/voice/interrupt')

                                # No Piper model handling here — use espeak by default.
                                self.model = None
                                self.voices_dir = ''
                                self.model_path = ''
                                self.config_path = ''
                                # Default sample rate used for any legacy pipelines
                                self.sample_rate = 22050

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
                                    raw_vol = self.get_parameter('volume').get_parameter_value().double_value
                                    # rclpy returns 0.0 for missing numeric; guard anyway
                                    if raw_vol is None:
                                        raw_vol = 1.0
                                    self.volume = float(raw_vol)
                                except Exception:
                                    # Fallback for older rclpy versions or unexpected types
                                    try:
                                        self.volume = float(self.get_parameter('volume').value)
                                    except Exception:
                                        self.volume = 1.0
                                # Clamp range
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

                                self._worker = threading.Thread(target=self._run_worker, daemon=True)
                                self._worker.start()

                                self.get_logger().info(f'Voice node listening on topic: {self.topic}')
                                self.get_logger().info(f'Using espeak-ng voice: {self.get_parameter("espeak_voice").get_parameter_value().string_value}')

                                self._announce_startup_message()
                                self._setup_ping_timer()

                            def _announce_startup_message(self) -> None:
                                """Queue an initial utterance so operators know the node is alive.

                                This will enqueue the configured startup_greeting parameter if present.
                                """
                                try:
                                    greeting = self.get_parameter('startup_greeting').get_parameter_value().string_value
                                except Exception:
                                    greeting = ''
                                if greeting:
                                    try:
                                        self.enqueue(String(data=greeting))
                                    except Exception:
                                        pass
                                return
                            
                            def _setup_ping_timer(self) -> None:
                                """Create a periodic timer that enqueues a short fortune/ping message.

                                Timer is stored on `self._ping_timer` so it can be cancelled during
                                shutdown.
                                """
                                try:
                                    enable = False
                                    try:
                                        enable = bool(self.get_parameter('enable_ping').get_parameter_value().bool_value)
                                    except Exception:
                                        enable = False
                                    if not enable:
                                        self._ping_timer = None
                                        return
                                    interval = 30
                                    try:
                                        interval = int(self.get_parameter('ping_interval_sec').get_parameter_value().integer_value or 30)
                                    except Exception:
                                        interval = 30
                                    # Use rclpy timer to schedule periodic pings
                                    from rclpy.duration import Duration
                                    # Store the timer so it can be canceled on shutdown
                                    self._ping_timer = self.create_timer(float(interval), self._send_fortune_ping)
                                except Exception:
                                    self._ping_timer = None
                                    return
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

                            def _load_sample_rate(self, config_path: str) -> int:
                                """Load sample_rate from Piper model config JSON if available.

                                Returns an integer sample rate. If the config doesn't exist or
                                cannot be parsed, returns a sensible default (22050).
                                """
                                try:
                                    if not config_path:
                                        return 22050
                                    p = pathlib.Path(config_path)
                                    if not p.exists():
                                        return 22050
                                    import json as _json
                                    with p.open('r', encoding='utf-8') as fh:
                                        cfg = _json.load(fh)
                                    # Configs vary; look for common keys
                                    for key in ('sample_rate', 'sampling_rate', 'sr'):
                                        if key in cfg:
                                            try:
                                                return int(cfg[key])
                                            except Exception:
                                                pass
                                    # Some piper configs embed 'audio' dict
                                    audio = cfg.get('audio') if isinstance(cfg, dict) else None
                                    if isinstance(audio, dict) and 'sample_rate' in audio:
                                        try:
                                            return int(audio['sample_rate'])
                                        except Exception:
                                            pass
                                except Exception:
                                    pass
                                # Sensible default
                                return 22050

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
                                # Cancel ping timer if created
                                try:
                                    if getattr(self, '_ping_timer', None) is not None:
                                        try:
                                            self._ping_timer.cancel()
                                        except Exception:
                                            pass
                                except Exception:
                                    pass
                                return super().destroy_node()

                            def _run_worker(self):
                                """Continuously speak queued messages with burst coalescing and subprocess pipeline."""
                                import shutil


                                # Only support espeak-ng/mbrola via pyttsx3 or espeak-ng subprocess
                                espeak_voice = self.get_parameter('espeak_voice').get_parameter_value().string_value
                                espeak_rate = int(self.get_parameter('espeak_rate').get_parameter_value().integer_value or 170)
                                espeak_pitch = int(self.get_parameter('espeak_pitch').get_parameter_value().integer_value or 50)
                                # Robustly parse espeak_volume (some environments may provide empty string)
                                try:
                                    raw_ev = self.get_parameter('espeak_volume').get_parameter_value().double_value
                                    if raw_ev is None:
                                        raw_ev = 1.0
                                    espeak_volume = float(raw_ev)
                                except Exception:
                                    try:
                                        espeak_volume = float(self.get_parameter('espeak_volume').value)
                                    except Exception:
                                        espeak_volume = 1.0
                                espeak_extra_args = self.get_parameter('espeak_extra_args').get_parameter_value().string_value

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


                                    # Engine-specific synthesis (espeak-ng/mbrola only)
                                    # Determine aplay command (may be disabled via parameter)
                                    try:
                                        aplay_enabled = bool(self.get_parameter('aplay').get_parameter_value().bool_value)
                                    except Exception:
                                        aplay_enabled = True
                                    aplay_cmd = shutil.which('aplay') if aplay_enabled else None

                                    # Call with correct parameter order: (text, aplay_cmd, voice, rate, pitch, volume, extra_args)
                                    success = self._speak_with_espeak(text, aplay_cmd, espeak_voice, espeak_rate, espeak_pitch, espeak_volume, espeak_extra_args)
                                    autophony_msg = UInt32()
                                    autophony_msg.data = 0
                                    self.autophony_pub.publish(autophony_msg)

                                    # Signal completion to upstream (e.g., chat service)
                                    try:
                                        done = String()
                                        done.data = text
                                        self._pub_done.publish(done)
                                    except Exception:
                                        pass

                            def _speak_with_subprocess(self, text: str, aplay_cmd: str | None, piper_cmd: list[str]) -> bool:
                                """Use subprocess piper binary for efficient speech synthesis."""
                                try:
                                    # Launch Piper to produce RAW PCM to stdout, then pipe into aplay
                                    piper = subprocess.Popen(
                                        [
                                            *piper_cmd,
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

                                # Optionally insert a volume scaling stage via 'sox' if available and volume != 1.0
                                use_volume = abs(self.volume - 1.0) > 1e-3
                                sox_path = shutil.which('sox') if use_volume else None
                                if sox_path:
                                    # sox -t raw -r <sr> -e signed-integer -b 16 -c 1 - -t raw -v <vol> -
                                    sox_cmd = [
                                        sox_path,
                                        '-t', 'raw',
                                        '-r', str(self.sample_rate),
                                        '-e', 'signed-integer',
                                        '-b', '16',
                                        '-c', '1',
                                        '-',
                                        '-t', 'raw',
                                        '-v', str(self.volume),
                                        '-',
                                    ]
                                    try:
                                        sox = subprocess.Popen(sox_cmd, stdin=piper.stdout, stdout=subprocess.PIPE)
                                        aplay = subprocess.Popen(aplay_cmd_list, stdin=sox.stdout)
                                        self._procs = [aplay, sox, piper]
                                    except Exception:
                                        # Fallback to direct pipeline if sox fails
                                        aplay = subprocess.Popen(aplay_cmd_list, stdin=piper.stdout)
                                        self._procs = [aplay, piper]
                                else:
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

                                # Publish autophony duration while speaking
                                start_time = time.time()
                                while aplay.poll() is None:
                                    duration = int((time.time() - start_time) * 1000)
                                    autophony_msg = UInt32()
                                    autophony_msg.data = duration
                                    self.autophony_pub.publish(autophony_msg)
                                    time.sleep(0.1)

                                # Wait for playback to finish
                                aplay.wait()
                                try:
                                    piper.terminate()
                                except Exception:
                                    pass
                                self._procs.clear()
                                return True

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
                                    self.get_logger().warning(f'Piper command validation error for {cmd}: {e}')
                                    return False

                            def _resolve_piper_cmd(self) -> list[str] | None:
                                """Determine a working Piper invocation.

                                Priority:
                                1. PIPER_CMD env (space-split respecting quotes)
                                2. Current Python: [sys.executable, '-m', 'piper'] if module is importable and validates
                                3. System 'piper' binary from PATH or common locations
                                """
                                # 1) Explicit override
                                override = os.getenv("PIPER_CMD", "/usr/bin/piper").strip()
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
                                # Safely parse the passed-in volume_scale (may be string or empty)
                                try:
                                    vs = volume_scale
                                    if isinstance(vs, str) and vs.strip() == '':
                                        vs = 1.0
                                    vsf = float(vs)
                                except Exception:
                                    vsf = 1.0
                                amp = int(max(0.0, min(1.5, vsf)) * 200)

                                # If runtime node volume is set, override amplitude accordingly
                                try:
                                    vol = max(0.0, min(1.5, float(getattr(self, 'volume', 1.0))))
                                    amp = int(vol * 200)
                                except Exception:
                                    amp = 200

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
    # No CLI arguments required for espeak-only configuration
    parser = argparse.ArgumentParser()
    # Parse known args to handle ROS2 arguments
    known_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
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
