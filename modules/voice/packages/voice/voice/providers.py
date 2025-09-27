"""Text-to-speech provider abstractions for the voice node."""

from __future__ import annotations

import abc
import asyncio
import base64
import json
import os
import pathlib
import shlex
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

try:
    import edge_tts  # type: ignore
except ImportError:  # pragma: no cover - optional dependency resolved via module.toml
    edge_tts = None


DurationCallback = Callable[[int], None]


class ProviderUnavailable(RuntimeError):
    """Raised when a selected provider cannot operate."""


@dataclass
class EspeakConfig:
    """Runtime configuration values for the espeak provider."""

    voice: str
    rate: int
    pitch: int
    base_volume: float
    extra_args: str
    aplay_enabled: bool
    aplay_device: Optional[str]


@dataclass
class MsEdgeConfig:
    """Runtime configuration for Microsoft Edge TTS.

    Attributes mirror the arguments accepted by ``edge_tts.Communicate`` and
    streaming playback preferences.
    """

    voice: str
    rate: str
    volume: str
    pitch: str
    style: Optional[str]
    output_format: str
    ffplay_args: tuple[str, ...]


@dataclass
class PiperConfig:
    """Runtime configuration for the Piper provider."""

    model: str
    voices_dir: str
    wav_out_dir: str
    length_scale: float
    noise_scale: float
    noise_w_scale: float
    normalize_audio: bool
    aplay_enabled: bool
    aplay_device: Optional[str]


def calculate_espeak_amplitude(base_volume: float, runtime_volume: float) -> int:
    """Map the configured and runtime volumes to an espeak ``-a`` value."""

    try:
        combined = float(base_volume) * float(runtime_volume)
    except Exception:
        combined = 1.0
    combined = max(0.0, min(1.5, combined))
    return int(combined * 200)


class BaseProvider(abc.ABC):
    """Common machinery for TTS providers."""

    def __init__(self, *, logger, config_getter: Callable[[], object]):
        self._logger = logger
        self._config_getter = config_getter
        self._process_lock = threading.Lock()
        self._processes: list[subprocess.Popen] = []
        self._interrupted = threading.Event()

    @property
    def interrupted(self) -> bool:
        """Return ``True`` when playback was interrupted."""

        return self._interrupted.is_set()

    def stop(self) -> None:
        """Signal the provider to abort playback and terminate processes."""

        self._interrupted.set()
        with self._process_lock:
            for proc in self._processes:
                try:
                    proc.terminate()
                except Exception:
                    pass
            self._processes.clear()

    def speak(self, text: str, *, volume: float, on_duration: DurationCallback) -> bool:
        """Synthesize and play ``text`` using the provider-specific engine."""

        self._interrupted.clear()
        config = self._config_getter()
        try:
            return self._speak(text, config=config, volume=volume, on_duration=on_duration)
        finally:
            with self._process_lock:
                self._processes.clear()

    @abc.abstractmethod
    def _speak(
        self,
        text: str,
        *,
        config: object,
        volume: float,
        on_duration: DurationCallback,
    ) -> bool:
        """Engine-specific synthesis implementation."""

    def _register_process(self, proc: subprocess.Popen) -> None:
        with self._process_lock:
            self._processes.append(proc)


class EspeakProvider(BaseProvider):
    """TTS provider backed by ``espeak-ng`` (or ``espeak``)."""

    def _speak(
        self,
        text: str,
        *,
        config: EspeakConfig,
        volume: float,
        on_duration: DurationCallback,
    ) -> bool:
        espeak = shutil.which("espeak-ng") or shutil.which("espeak")
        if not espeak:
            raise ProviderUnavailable("espeak-ng binary not found")

        amp = calculate_espeak_amplitude(config.base_volume, volume)
        rate = max(80, min(450, int(config.rate)))
        pitch = max(0, min(99, int(config.pitch)))

        try:
            extra_args = shlex.split(config.extra_args) if config.extra_args else []
        except Exception:
            extra_args = []

        base_cmd: list[str] = [
            espeak,
            "-v",
            config.voice,
            "-s",
            str(rate),
            "-p",
            str(pitch),
            "-a",
            str(amp),
        ]
        base_cmd.extend(extra_args)

        aplay_cmd = None
        if config.aplay_enabled:
            aplay_cmd = shutil.which("aplay")
            if not aplay_cmd:
                self._logger.warning("aplay requested but not available; espeak will play directly")

        start = time.monotonic()
        if aplay_cmd:
            speak = subprocess.Popen([*base_cmd, "--stdout", text], stdout=subprocess.PIPE)
            assert speak.stdout is not None
            self._register_process(speak)

            aplay_args: list[str] = [aplay_cmd, "-q", "-t", "wav", "-"]
            if config.aplay_device:
                aplay_args.extend(["-D", config.aplay_device])

            aplay = subprocess.Popen(aplay_args, stdin=speak.stdout)
            self._register_process(aplay)

            while aplay.poll() is None and not self.interrupted:
                elapsed_ms = int((time.monotonic() - start) * 1000)
                on_duration(elapsed_ms)
                time.sleep(0.05)

            if self.interrupted:
                return False

            aplay.wait()
            try:
                speak.terminate()
            except Exception:
                pass
            return True

        run = subprocess.run([*base_cmd, text], capture_output=True, text=True)
        if run.returncode != 0:
            self._logger.error("espeak synthesis failed: %s", run.stderr.strip())
        on_duration(0)
        return run.returncode == 0


class MsEdgeProvider(BaseProvider):
    """Provider that streams audio from Microsoft Edge TTS via ``edge-tts``."""

    def _ensure_available(self) -> None:
        if edge_tts is None:
            raise ProviderUnavailable("edge-tts library is not installed")

    def _speak(
        self,
        text: str,
        *,
        config: MsEdgeConfig,
        volume: float,
        on_duration: DurationCallback,
    ) -> bool:
        self._ensure_available()

        ffplay = shutil.which("ffplay")
        if not ffplay:
            raise ProviderUnavailable("ffplay binary not found (required for ms-edge provider)")

        cmd = [ffplay, "-autoexit", "-nodisp", "-loglevel", "error", "-i", "pipe:0", *config.ffplay_args]

        async def _stream() -> bool:
            communicate = edge_tts.Communicate(
                text,
                voice=config.voice,
                rate=config.rate,
                volume=config.volume,
                pitch=config.pitch,
                style=config.style,
                output_format=config.output_format,
            )

            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            self._register_process(proc)

            assert proc.stdin is not None
            start_time = time.monotonic()

            try:
                async for chunk in communicate.stream():
                    if self.interrupted or proc.poll() is not None:
                        return False
                    if chunk["type"] != "audio":
                        continue
                    data = chunk.get("data", b"")
                    if isinstance(data, str):
                        data = base64.b64decode(data)
                    try:
                        proc.stdin.write(data)
                        proc.stdin.flush()
                    except BrokenPipeError:
                        return False
                    elapsed_ms = int((time.monotonic() - start_time) * 1000)
                    on_duration(elapsed_ms)
            except asyncio.CancelledError:
                return False
            except Exception as exc:  # pragma: no cover - network/runtime errors
                self._logger.error("edge-tts streaming error: %s", exc)
                return False
            finally:
                try:
                    proc.stdin.close()
                except Exception:
                    pass

            while proc.poll() is None and not self.interrupted:
                elapsed_ms = int((time.monotonic() - start_time) * 1000)
                on_duration(elapsed_ms)
                await asyncio.sleep(0.05)

            return proc.returncode == 0 and not self.interrupted

        try:
            return asyncio.run(_stream())
        except RuntimeError:
            loop = asyncio.new_event_loop()
            try:
                return loop.run_until_complete(_stream())
            finally:
                loop.close()


class PiperProvider(BaseProvider):
    """Provider that streams audio through Rhasspy Piper."""

    def _speak(
        self,
        text: str,
        *,
        config: PiperConfig,
        volume: float,
        on_duration: DurationCallback,
    ) -> bool:
        model_basename = config.model.strip()
        if not model_basename:
            raise ProviderUnavailable("piper model not configured")

        voices_dir = pathlib.Path(config.voices_dir).expanduser()
        model_path = voices_dir / f"{model_basename}.onnx"
        cfg_path = voices_dir / f"{model_basename}.onnx.json"
        voices_dir.mkdir(parents=True, exist_ok=True)

        self._ensure_piper_model(model_basename, model_path, cfg_path)

        piper_cmd = self._resolve_piper_cmd(model_path, cfg_path)
        if not piper_cmd:
            raise ProviderUnavailable("unable to locate usable piper command")

        sample_rate = self._load_sample_rate(cfg_path)

        aplay_cmd = None
        if config.aplay_enabled:
            aplay_cmd = shutil.which("aplay")
            if not aplay_cmd:
                self._logger.warning("aplay requested but not available; Piper output will not play")

        if not aplay_cmd and not config.wav_out_dir:
            raise ProviderUnavailable("no playback target for Piper output (aplay disabled and wav_out_dir empty)")

        base_cmd: list[str] = [*piper_cmd, "--model", str(model_path)]
        if cfg_path.exists():
            base_cmd += ["--config", str(cfg_path)]
        base_cmd += [
            "--output_raw",
            "--length_scale",
            str(config.length_scale),
            "--noise_scale",
            str(config.noise_scale),
            "--noise_w",
            str(config.noise_w_scale),
        ]
        if config.normalize_audio:
            base_cmd.append("--normalize")

        proc = subprocess.Popen(base_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        assert proc.stdin is not None and proc.stdout is not None
        self._register_process(proc)

        proc.stdin.write((text + "\n").encode("utf-8"))
        proc.stdin.flush()
        proc.stdin.close()

        if aplay_cmd:
            aplay_args = [aplay_cmd, "-q", "-r", str(sample_rate), "-f", "S16_LE", "-t", "raw", "-"]
            if config.aplay_device:
                aplay_args.extend(["-D", config.aplay_device])
            aplay = subprocess.Popen(aplay_args, stdin=proc.stdout)
            self._register_process(aplay)
            start = time.monotonic()
            while aplay.poll() is None and not self.interrupted:
                elapsed_ms = int((time.monotonic() - start) * 1000)
                on_duration(elapsed_ms)
                time.sleep(0.05)

            if self.interrupted:
                return False

            aplay.wait()
        else:
            wav_path = pathlib.Path(config.wav_out_dir)
            wav_path.mkdir(parents=True, exist_ok=True)
            tmp = wav_path / "piper-output.wav"
            with tmp.open("wb") as handle:
                handle.write(proc.stdout.read())
            self._logger.info("Piper wrote audio to %s", tmp)

        try:
            proc.terminate()
        except Exception:
            pass

        return True

    def _ensure_piper_model(self, name: str, model_path: pathlib.Path, cfg_path: pathlib.Path) -> None:
        if model_path.exists() and cfg_path.exists():
            return

        model_url = os.getenv(
            "PIPER_MODEL_URL",
            f"https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/high/{name}.onnx?download=true",
        )
        cfg_url = os.getenv(
            "PIPER_CONFIG_URL",
            f"https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/high/{name}.onnx.json?download=true",
        )

        if not model_path.exists():
            try:
                subprocess.run(["curl", "-fsSL", "-o", str(model_path), model_url], check=True)
            except Exception as exc:
                raise ProviderUnavailable(f"failed to download Piper model: {exc}") from exc

        if not cfg_path.exists():
            try:
                subprocess.run(["curl", "-fsSL", "-o", str(cfg_path), cfg_url], check=True)
            except Exception as exc:
                raise ProviderUnavailable(f"failed to download Piper config: {exc}") from exc

    def _resolve_piper_cmd(self, model_path: pathlib.Path, cfg_path: pathlib.Path) -> Optional[list[str]]:
        override = os.getenv("PIPER_CMD", "").strip()
        if override:
            try:
                candidate = shlex.split(override)
            except Exception:
                candidate = [override]
            if self._validate_piper_cmd(candidate, model_path, cfg_path):
                return candidate

        try:
            import importlib.util  # local import

            if importlib.util.find_spec("piper") is not None:
                candidate = [sys.executable or shutil.which("python3") or "python3", "-m", "piper"]
                if self._validate_piper_cmd(candidate, model_path, cfg_path):
                    return candidate
        except Exception:
            pass

        binary = shutil.which("piper")
        candidates: Iterable[list[str]]
        if binary:
            candidates = ([binary],)
        else:
            candidates = (
                ["/usr/bin/piper"],
                [os.path.expanduser("~/.local/bin/piper")],
                ["/usr/local/bin/piper"],
                ["/opt/piper/bin/piper"],
            )
        for candidate in candidates:
            if not candidate:
                continue
            if self._validate_piper_cmd(candidate, model_path, cfg_path):
                return candidate

        return None

    def _validate_piper_cmd(
        self,
        cmd: list[str],
        model_path: pathlib.Path,
        cfg_path: pathlib.Path,
    ) -> bool:
        try:
            res = subprocess.run([*cmd, "--help"], capture_output=True, text=True, timeout=5)
            output = (res.stdout or "") + (res.stderr or "")
            if res.returncode == 0 and "ModuleNotFoundError" not in output:
                return True
            if "required: -m/--model" in output or res.returncode != 0:
                args = [*cmd, "-m", str(model_path), "--help"]
                if cfg_path.exists():
                    args = [*cmd, "-m", str(model_path), "--config", str(cfg_path), "--help"]
                res2 = subprocess.run(args, capture_output=True, text=True, timeout=5)
                output2 = (res2.stdout or "") + (res2.stderr or "")
                return res2.returncode == 0 and "ModuleNotFoundError" not in output2
        except Exception:
            return False
        return False

    def _load_sample_rate(self, config_path: pathlib.Path) -> int:
        if not config_path.exists():
            return 22050
        try:
            with config_path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
            if isinstance(data, dict):
                for key in ("sample_rate", "sampling_rate", "sr"):
                    if key in data:
                        return int(data[key])
                audio = data.get("audio")
                if isinstance(audio, dict) and "sample_rate" in audio:
                    return int(audio["sample_rate"])
        except Exception:
            return 22050
        return 22050


def build_tts_provider(
    engine: str,
    *,
    logger,
    config_getter: Callable[[], object],
) -> BaseProvider:
    """Instantiate the provider for ``engine``."""

    normalized = (engine or "espeak").strip().lower()
    if normalized in {"espeak", "espeak-ng"}:
        return EspeakProvider(logger=logger, config_getter=config_getter)
    if normalized in {"ms-edge", "msedge", "edge", "edge_tts"}:
        return MsEdgeProvider(logger=logger, config_getter=config_getter)
    if normalized == "piper":
        return PiperProvider(logger=logger, config_getter=config_getter)

    raise ProviderUnavailable(f"unsupported TTS engine: {engine}")
