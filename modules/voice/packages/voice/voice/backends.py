"""Speech backend implementations.

Three backends are provided:

* :class:`PrintSpeechBackend` writes spoken text to a stream (stdout by
  default). It is useful for testing or systems without a speech synthesizer.
* :class:`EspeakSpeechBackend` wraps the ``espeak`` command line utility.
* :class:`WebsocketTTSSpeechBackend` connects to the Coqui-driven websocket
  service in :mod:`services.tts` and streams PCM audio frames to a local
  playback command such as ``aplay``.

All backends implement the :class:`SpeechBackend` protocol and cooperate with
:class:`~voice.queue.SpeechQueue` through the
:class:`~voice.exceptions.SpeechInterrupted` exception.
"""

from __future__ import annotations

import json
import logging
import shutil
import subprocess
import sys
import threading
from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import BinaryIO, Callable, Optional, Protocol, Sequence, TextIO

from .exceptions import SpeechInterrupted

_LOGGER = logging.getLogger(__name__)

ProgressCallback = Callable[[str], None]


class _WebsocketConnection(Protocol):
    """Subset of websocket client behaviour used by the backend."""

    def send(self, message: str | bytes) -> None:
        """Send ``message`` to the websocket peer."""

    def recv(self) -> str | bytes:
        """Receive the next message from the websocket peer."""

    def close(self) -> None:
        """Close the websocket connection."""


class _PlayerProcess(Protocol):
    """Protocol describing the audio playback subprocess used for PCM output."""

    stdin: BinaryIO

    def terminate(self) -> None:
        """Request that the playback process stops."""

    def wait(self, timeout: float | None = None) -> None:
        """Wait for the playback process to exit."""

    def kill(self) -> None:
        """Forcefully kill the playback process."""


ConnectionFactory = Callable[[], AbstractContextManager[_WebsocketConnection]]
PlayerFactory = Callable[[int, int], _PlayerProcess]


class SpeechBackend(Protocol):
    """Protocol implemented by speech backends.

    Backends must block until speech playback completes or raise
    :class:`SpeechInterrupted` when the supplied stop event is triggered.
    """

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> None:
        """Play ``text`` using the backend.

        Args:
            text: The text that should be spoken.
            stop_event: Event set when playback should stop immediately.
            progress_callback: Optional callback invoked with the spoken text
                once playback finishes.
        """


class FailoverSpeechBackend:
    """Wrap a primary backend and fall back to an alternate on failure.

    The wrapper forwards all playback requests to the primary backend until a
    failure matching ``failure_exceptions`` occurs. From that point onward the
    fallback backend is used for all subsequent requests. This keeps recovery
    logic contained within the backend layer so callers such as
    :class:`~voice.queue.SpeechQueue` do not need to reason about retries or
    backend replacement.
    """

    def __init__(
        self,
        primary: SpeechBackend,
        fallback: SpeechBackend,
        *,
        failure_exceptions: tuple[type[Exception], ...] = (
            ConnectionError,
            TimeoutError,
            OSError,
        ),
        log_warning: Callable[[str], None] | None = None,
    ) -> None:
        if not failure_exceptions:
            raise ValueError("failure_exceptions must contain at least one exception type")
        self._primary = primary
        self._fallback = fallback
        self._failure_exceptions = failure_exceptions
        self._using_fallback = False
        self._log_warning = log_warning or _LOGGER.warning
        self._primary_name = primary.__class__.__name__
        self._fallback_name = fallback.__class__.__name__

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> None:
        backend = self._fallback if self._using_fallback else self._primary
        try:
            backend.speak(text, stop_event, progress_callback)
            return
        except SpeechInterrupted:
            raise
        except self._failure_exceptions as error:
            if self._using_fallback:
                raise
            self._using_fallback = True
            self._log_warning(
                f"{self._primary_name} failed with {error!r}; switching to {self._fallback_name}",
            )
            self._fallback.speak(text, stop_event, progress_callback)
            return

@dataclass(slots=True)
class PrintSpeechBackend:
    """Speech backend that prints text to a stream.

    Example
    -------
    >>> backend = PrintSpeechBackend()
    >>> backend.speak("Hello", threading.Event())
    Hello
    """

    stream: TextIO = sys.stdout

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> None:
        if stop_event.is_set():
            raise SpeechInterrupted("stopped before print")
        print(text, file=self.stream, flush=True)
        if progress_callback:
            progress_callback(text)


class EspeakSpeechBackend:
    """Speech backend powered by the ``espeak`` command line utility.

    Parameters
    ----------
    command:
        Sequence representing the base command. Defaults to ``["espeak"]``.
    voice:
        Optional voice identifier passed to ``espeak`` via ``-v``.
    rate:
        Optional speech rate passed to ``espeak`` via ``-s``.
    """

    def __init__(
        self,
        command: Sequence[str] | None = None,
        *,
        voice: str | None = None,
        rate: int | None = None,
    ) -> None:
        base_command = list(command) if command else ["espeak"]
        if not base_command:
            raise ValueError("command must contain at least one element")
        executable = shutil.which(base_command[0])
        if executable is None:
            raise FileNotFoundError(
                f"Unable to locate speech synthesizer '{base_command[0]}'",
            )
        self._command = [executable, *base_command[1:]]
        self._voice = voice
        self._rate = rate

    def _build_command(self) -> list[str]:
        command = [*self._command, "--stdin"]
        if self._voice:
            command.extend(["-v", self._voice])
        if self._rate is not None:
            command.extend(["-s", str(self._rate)])
        return command

    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> None:
        process = subprocess.Popen(
            self._build_command(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        assert process.stdin is not None  # pragma: no cover - for type checkers

        try:
            process.stdin.write(text)
            process.stdin.close()
        except Exception:  # pragma: no cover - defensive logging
            _LOGGER.exception("Failed to send text to espeak")
            process.kill()
            raise

        try:
            while True:
                if stop_event.wait(timeout=0.05):
                    _LOGGER.debug("Stopping espeak process early")
                    process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()
                    raise SpeechInterrupted("stop requested")
                return_code = process.poll()
                if return_code is not None:
                    if return_code != 0:
                        stderr_output = process.stderr.read() if process.stderr else ""
                        raise RuntimeError(
                            f"espeak exited with code {return_code}: {stderr_output.strip()}",
                        )
                    if progress_callback:
                        progress_callback(text)
                    return
        finally:
            if process.stdout:
                process.stdout.close()
            if process.stderr:
                process.stderr.close()


class WebsocketTTSSpeechBackend:
    """Speech backend that streams audio from the Coqui websocket service.

    Parameters
    ----------
    url:
        Websocket URL of the text-to-speech service. Defaults to
        ``"ws://127.0.0.1:5002/tts"``.
    speaker:
        Optional speaker identifier forwarded with each synthesis request.
    language:
        Optional language hint forwarded with each request.
    player_command:
        Optional base command used to play PCM audio. When ``None`` the backend
        uses ``aplay`` with flags appropriate for little-endian 16-bit PCM.
    connection_factory:
        Advanced hook allowing tests to supply a fake websocket connection.
    player_factory:
        Advanced hook allowing tests to provide a fake playback process.
    connect_timeout:
        Maximum number of seconds to wait while establishing the websocket
        connection.
    """

    def __init__(
        self,
        *,
        url: str = "ws://127.0.0.1:5002/tts",
        speaker: str | None = None,
        language: str | None = None,
        player_command: Sequence[str] | None = None,
        connection_factory: ConnectionFactory | None = None,
        player_factory: PlayerFactory | None = None,
        connect_timeout: float = 5.0,
    ) -> None:
        if not url:
            raise ValueError("url must be a non-empty string")
        self._url = url
        self._speaker = speaker
        self._language = language
        self._connect_timeout = connect_timeout
        # Diagnostic logging: surface user-provided configuration so failures
        # during initialisation are easier to debug in the logs.
        _LOGGER.debug(
            "Initialising WebsocketTTSSpeechBackend(url=%s, speaker=%s, language=%s, player_command=%r, connect_timeout=%s)",
            self._url,
            self._speaker,
            self._language,
            player_command,
            self._connect_timeout,
        )

        self._connection_factory = connection_factory or self._default_connection_factory
        # Build a player factory, but capture and log any initialisation errors
        # with the provided player_command so the deployment logs show the root
        # cause rather than a generic exception.
        try:
            self._player_factory = player_factory or self._build_player_factory(player_command)
        except Exception as exc:  # pragma: no cover - surface runtime diagnostics
            _LOGGER.error(
                "Failed to build player factory in WebsocketTTSSpeechBackend: %s; player_command=%r",
                exc,
                player_command,
            )
            raise

    # ----------------------------------------------------------------- protocol
    def speak(
        self,
        text: str,
        stop_event: threading.Event,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> None:
        if stop_event.is_set():
            raise SpeechInterrupted("stop requested before synthesis")

        payload: dict[str, str] = {"text": text}
        if self._speaker:
            payload["speaker"] = self._speaker
        if self._language:
            payload["language"] = self._language

        process: _PlayerProcess | None = None
        stopped = False
        try:
            with self._connection_factory() as websocket:
                websocket.send(json.dumps(payload))
                metadata = self._await_start(websocket, stop_event)
                process = self._player_factory(metadata["sample_rate"], metadata["channels"])
                stdin = getattr(process, "stdin", None)
                if stdin is None:
                    raise RuntimeError("Playback process does not expose a stdin pipe")
                self._stream_audio(websocket, process, stop_event)
        except SpeechInterrupted:
            if process is not None:
                self._stop_process(process)
                stopped = True
            raise
        except Exception:
            if process is not None:
                self._stop_process(process)
                stopped = True
            raise
        else:
            if progress_callback:
                progress_callback(text)
        finally:
            if process is not None and not stopped:
                self._close_process(process)

    # ------------------------------------------------------------------ internals
    def _default_connection_factory(self) -> AbstractContextManager[_WebsocketConnection]:
        sync_import_failed = False
        try:
            from websockets.sync.client import connect  # type: ignore import
        except ModuleNotFoundError as exc:  # pragma: no cover - runtime guard
            # Provide richer diagnostics so we can see which Python paths are
            # visible to the running process and whether a websockets module
            # exists in any of them. Don't raise here; fall back to the async
            # API wrapper implemented below.
            sync_import_failed = True
            _LOGGER.error("websockets.sync client not available: %s", exc)
            try:
                import sys as _sys
                _LOGGER.debug("Python sys.path: %s", _sys.path)
            except Exception:
                pass
            try:
                import importlib
                websockets_mod = importlib.import_module("websockets")
            except Exception as import_exc:
                _LOGGER.debug("Importing 'websockets' failed: %s", import_exc)
            else:
                try:
                    ver = getattr(websockets_mod, "__version__", "<unknown>")
                    src = getattr(websockets_mod, "__file__", "<unknown>")
                    _LOGGER.debug("Found websockets module version=%s at %s", ver, src)
                except Exception:
                    pass

        _LOGGER.debug("Creating websocket connection factory for URL %s (timeout=%s)", self._url, self._connect_timeout)
        try:
            # Newer 'websockets' exposes a sync client API which we prefer.
            return connect(self._url, open_timeout=self._connect_timeout, close_timeout=5, max_size=None)
        except NameError:
            # connect is not defined in this scope; fall through to async fallback
            pass

        # Fallback for older websockets versions that only provide an async
        # connect coroutine. We'll run an asyncio loop in a background thread
        # and expose a synchronous context manager that provides the minimal
        # send/recv/close methods the backend expects.
        try:
            import asyncio
            import inspect

            import importlib

            # Try a few common locations for the async connect API. Older
            # websockets releases sometimes provide a connect that is not a
            # coroutine function but instead returns an async context manager
            # object. We'll detect both possibilities by calling the candidate
            # with the URL and inspecting the result.
            async_connect = None
            async_returns_cm = False
            tried: list[str] = []
            for mod_name in ("websockets.client", "websockets"):
                try:
                    mod = importlib.import_module(mod_name)
                except Exception:
                    tried.append(mod_name + ":import_failed")
                    continue
                cand = getattr(mod, "connect", None)
                if cand is None:
                    tried.append(mod_name + ":no_connect")
                    continue
                # Determine whether the candidate is a coroutine function
                # (the modern async connect) or a callable that returns an
                # async context manager (older versions). We avoid calling
                # the candidate at detection time because some implementations
                # may require an event loop in the current thread.
                if inspect.iscoroutinefunction(cand):
                    async_connect = cand
                    async_returns_cm = False
                    break
                # Otherwise, assume it returns an async context manager when
                # called inside an event loop.
                async_connect = cand
                async_returns_cm = True
                break
            if async_connect is None:
                _LOGGER.debug("Tried websockets locations: %s", tried)
                raise RuntimeError("websockets does not expose a compatible connect API")

            class _ThreadedWSConnection(AbstractContextManager):
                def __init__(self, url: str, timeout: float):
                    self._url = url
                    self._timeout = timeout
                    self._loop: asyncio.AbstractEventLoop | None = None
                    self._thread: threading.Thread | None = None
                    self._ws = None

                def __enter__(self):
                    # Start background event loop
                    self._loop = asyncio.new_event_loop()

                    def run_loop(loop: asyncio.AbstractEventLoop) -> None:
                        asyncio.set_event_loop(loop)
                        loop.run_forever()

                    self._thread = threading.Thread(target=run_loop, args=(self._loop,), daemon=True)
                    self._thread.start()

                    # Open connection in the background loop
                    # Build a coroutine that either awaits the connect
                    # coroutine or enters the async context manager and
                    # returns the underlying websocket/protocol object plus
                    # the context manager so we can exit later.
                    async def _enter_connect():
                        cm = async_connect(self._url)
                        # If cm has __aenter__, it's an async context manager
                        if hasattr(cm, "__aenter__"):
                            ws = await cm.__aenter__()
                            return ws, cm
                        # Otherwise cm is an awaitable returning ws
                        ws = await cm
                        return ws, None

                    ws_obj, cm_obj = asyncio.run_coroutine_threadsafe(_enter_connect(), self._loop).result(timeout=self._timeout + 2)
                    self._ws = ws_obj
                    self._cm = cm_obj

                    class _Conn:
                        def __init__(self, ws, loop):
                            self._ws = ws
                            self._loop = loop

                        def send(self, message: str | bytes) -> None:
                            asyncio.run_coroutine_threadsafe(self._ws.send(message), self._loop).result()

                        def recv(self) -> str | bytes:
                            return asyncio.run_coroutine_threadsafe(self._ws.recv(), self._loop).result()

                        def close(self) -> None:
                            try:
                                if hasattr(self, "_cm") and self._cm is not None:
                                    asyncio.run_coroutine_threadsafe(self._cm.__aexit__(None, None, None), self._loop).result(timeout=2)
                                else:
                                    asyncio.run_coroutine_threadsafe(self._ws.close(), self._loop).result(timeout=2)
                            except Exception:
                                pass

                    return _Conn(self._ws, self._loop)

                def __exit__(self, exc_type, exc, tb):
                    try:
                        if getattr(self, "_cm", None) is not None:
                            try:
                                asyncio.run_coroutine_threadsafe(self._cm.__aexit__(None, None, None), self._loop).result(timeout=2)
                            except Exception:
                                pass
                        elif self._ws is not None:
                            try:
                                asyncio.run_coroutine_threadsafe(self._ws.close(), self._loop).result(timeout=2)
                            except Exception:
                                pass
                    finally:
                        try:
                            if self._loop is not None:
                                self._loop.call_soon_threadsafe(self._loop.stop)
                        except Exception:
                            pass
                        try:
                            if self._thread is not None:
                                self._thread.join(timeout=1)
                        except Exception:
                            pass

            return _ThreadedWSConnection(self._url, self._connect_timeout)
        except Exception as exc:  # pragma: no cover - runtime guards
            _LOGGER.error("Failed to create async fallback websocket connection factory: %s", exc)
            raise

    def _build_player_factory(
        self, player_command: Sequence[str] | None
    ) -> PlayerFactory:

        # Treat an empty sequence the same as ``None``: prefer system players
        # (aplay, then ffplay, then ffmpeg). This avoids erroring when the
        # tts_player_command parameter is an empty list coming from the launch
        # system.
        requested_cmd: list[str] | None
        if player_command is None:
            requested_cmd = None
        else:
            requested_cmd = list(player_command)
            if not requested_cmd:
                requested_cmd = None

        player_type: str | None = None
        base_executable: str | None = None

        if requested_cmd is None:
            # Try aplay first
            exe = shutil.which("aplay")
            if exe:
                player_type = "aplay"
                base_executable = exe
            else:
                # Try ffplay (part of ffmpeg)
                exe = shutil.which("ffplay")
                if exe:
                    player_type = "ffplay"
                    base_executable = exe
                else:
                    exe = shutil.which("ffmpeg")
                    if exe:
                        player_type = "ffmpeg"
                        base_executable = exe
                    else:
                        raise FileNotFoundError(
                            "Unable to locate 'aplay', 'ffplay' or 'ffmpeg' for PCM playback"
                        )
        else:
            # User supplied a command; resolve its executable
            exe = shutil.which(requested_cmd[0])
            if exe is None:
                raise FileNotFoundError(f"Unable to locate playback command '{requested_cmd[0]}'")
            player_type = "custom"
            base_executable = exe
            # preserve any additional args provided by the user
            requested_cmd[0] = exe

        def factory(sample_rate: int, channels: int) -> _PlayerProcess:
            # Build the platform-specific command using the resolved executable
            if player_type == "aplay":
                command = [base_executable, "-q", "-f", "S16_LE", "-c", str(channels), "-r", str(sample_rate)]
            elif player_type == "ffplay":
                # ffplay reads from stdin with -i - ; set format and sample params
                command = [
                    base_executable,
                    "-nodisp",
                    "-autoexit",
                    "-loglevel",
                    "panic",
                    "-f",
                    "s16le",
                    "-ar",
                    str(sample_rate),
                    "-ac",
                    str(channels),
                    "-i",
                    "-",
                ]
            elif player_type == "ffmpeg":
                # Use ffmpeg to play raw PCM to the default ALSA device. This
                # is a best-effort fallback; system configuration may vary.
                command = [
                    base_executable,
                    "-f",
                    "s16le",
                    "-ar",
                    str(sample_rate),
                    "-ac",
                    str(channels),
                    "-i",
                    "-",
                    "-f",
                    "alsa",
                    "default",
                    "-hide_banner",
                    "-loglevel",
                    "panic",
                ]
            else:
                # custom: use the user supplied command verbatim, appending
                # sample format arguments similar to aplay behaviour.
                command = [*requested_cmd, "-f", "S16_LE", "-c", str(channels), "-r", str(sample_rate)]

            process = subprocess.Popen(command, stdin=subprocess.PIPE)
            assert process.stdin is not None  # pragma: no cover - type checker hint
            return process  # type: ignore[return-value]

        return factory

        return factory

    def _await_start(
        self, websocket: _WebsocketConnection, stop_event: threading.Event
    ) -> dict[str, int]:
        while True:
            self._ensure_not_stopped(stop_event)
            message = websocket.recv()
            if isinstance(message, bytes):
                raise RuntimeError("TTS websocket sent binary frame before metadata")
            data = self._parse_json(message)
            event = data.get("event")
            if event == "start":
                try:
                    sample_rate = int(data["sample_rate"])
                    channels = int(data["channels"])
                except (KeyError, TypeError, ValueError) as exc:
                    raise RuntimeError(
                        "TTS websocket start event missing sample_rate or channels"
                    ) from exc
                if sample_rate <= 0 or channels <= 0:
                    raise RuntimeError("TTS websocket reported invalid audio parameters")
                audio_format = str(data.get("format", "")).lower()
                if audio_format and audio_format not in {"pcm_s16le", "pcm16", "pcm16le"}:
                    raise RuntimeError(f"Unsupported audio format '{audio_format}' from TTS websocket")
                return {
                    "sample_rate": sample_rate,
                    "channels": channels,
                }
            if event == "error":
                raise RuntimeError(str(data.get("message", "Unknown TTS error")))
            if event == "end":
                raise RuntimeError("TTS websocket signalled end before streaming audio")

    def _stream_audio(
        self,
        websocket: _WebsocketConnection,
        process: _PlayerProcess,
        stop_event: threading.Event,
    ) -> None:
        stdin = process.stdin
        while True:
            self._ensure_not_stopped(stop_event)
            message = websocket.recv()
            if isinstance(message, bytes):
                if not message:
                    continue
                stdin.write(message)
                stdin.flush()
                continue
            data = self._parse_json(message)
            event = data.get("event")
            if event == "end":
                break
            if event == "error":
                raise RuntimeError(str(data.get("message", "Unknown TTS error")))

    def _ensure_not_stopped(self, stop_event: threading.Event) -> None:
        if stop_event.is_set():
            raise SpeechInterrupted("stop requested")

    def _parse_json(self, payload: str) -> dict[str, object]:
        try:
            return json.loads(payload)
        except json.JSONDecodeError as exc:
            raise RuntimeError("TTS websocket returned invalid JSON") from exc

    def _stop_process(self, process: _PlayerProcess) -> None:
        try:
            process.terminate()
            try:
                process.stdin.close()
            except Exception:  # pragma: no cover - defensive cleanup
                _LOGGER.debug("Failed to close playback stdin during stop", exc_info=True)
            process.wait(timeout=1)
        except Exception:  # pragma: no cover - defensive cleanup
            _LOGGER.debug("Killing playback process after termination failure", exc_info=True)
            try:
                process.kill()
            except Exception:
                _LOGGER.debug("Failed to kill playback process", exc_info=True)

    def _close_process(self, process: _PlayerProcess) -> None:
        try:
            process.stdin.close()
        finally:
            try:
                process.wait(timeout=1)
            except Exception:  # pragma: no cover - defensive cleanup
                _LOGGER.debug("Playback process did not exit cleanly", exc_info=True)
