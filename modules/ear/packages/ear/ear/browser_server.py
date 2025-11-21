"""HTTPS bridge for browser-based ear and eye harnesses.

The server exposes two entry points over HTTPS:

* ``/ear`` serves a microphone harness that streams PCM audio to the ASR
  service and forwards recognised transcripts into ROS topics.
* ``/eye`` serves a lightweight camera harness that streams recorded chunks to
  the Eye module so the feed can stay live in a separate tab.

Audio forwarding is intentionally free of VAD or silence filtering so elongated
("stretch") sounds reach the ASR service unmodified. Transcripts are published
on the regular ear topics so downstream consumers keep functioning.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import logging
import ssl
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping, MutableMapping, Optional

from aiohttp import WSMsgType, web
from rclpy.context import Context
from rclpy.node import Node
from std_msgs.msg import String

from .backends import ServiceASREarBackend, TranscriptionEvent
from .worker import EarWorker

_LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class BridgeConfig:
    """Runtime configuration for the browser bridge."""

    listen_host: str = "0.0.0.0"
    listen_port: int = 9443
    asr_uri: str = "ws://127.0.0.1:5003/asr"
    transcript_topic: str = "/ear/hole"
    event_topic: str = "/ear/asr_event"
    eye_topic: str = "/eye/browser_clip"
    cert_path: Path = Path(".selfsigned/ear_bridge.cert.pem")
    key_path: Path = Path(".selfsigned/ear_bridge.key.pem")


class EarRelay(Node):
    """Publish browser-provided telemetry into ROS topics."""

    def __init__(
        self,
        *,
        config: BridgeConfig,
        context: Optional[Context] = None,
    ) -> None:
        super().__init__("ear_browser_bridge", context=context)
        self._transcript_publisher = self.create_publisher(String, config.transcript_topic, 10)
        self._event_publisher = self.create_publisher(String, config.event_topic, 10)
        self._eye_publisher = self.create_publisher(String, config.eye_topic, 10)

    def publish_transcription(self, event: TranscriptionEvent) -> None:
        """Publish structured events and final transcripts to ROS topics."""

        payload = json.dumps(event.to_dict(), ensure_ascii=False)
        event_msg = String()
        event_msg.data = payload
        self._event_publisher.publish(event_msg)

        if event.is_partial:
            self.get_logger().debug("Partial transcription: %s", event.text)
            return

        text = event.text.strip()
        if not text:
            text = " ".join(
                segment.text.strip() for segment in event.segments if getattr(segment, "text", "").strip()
            )
        if not text:
            return

        transcript_msg = String()
        transcript_msg.data = text
        self._transcript_publisher.publish(transcript_msg)
        self.get_logger().info("Heard (browser): %s", text)

    def publish_eye_frame(self, payload: Mapping[str, object]) -> None:
        """Publish browser camera frames/chunks for the Eye module."""

        message = String()
        message.data = json.dumps(dict(payload), ensure_ascii=False)
        self._eye_publisher.publish(message)
        self.get_logger().info(
            "Forwarded browser eye frame seq=%s (%s bytes)",
            payload.get("sequence"),
            payload.get("bytes"),
        )

    # Backwards compatibility with earlier clip-based harnesses
    def publish_eye_clip(self, payload: Mapping[str, object]) -> None:  # pragma: no cover - compatibility bridge
        self.publish_eye_frame(payload)


class EarSession:
    """Manage a websocket client's audio forwarding lifecycle."""

    def __init__(
        self,
        *,
        relay: EarRelay,
        backend_factory: Callable[[], ServiceASREarBackend],
        loop: asyncio.AbstractEventLoop,
    ) -> None:
        self._relay = relay
        self._backend_factory = backend_factory
        self._loop = loop
        self._queue: asyncio.Queue[dict[str, object]] = asyncio.Queue(maxsize=256)
        self._worker: EarWorker | None = None
        self._closed = False

    def start(self) -> None:
        """Start streaming audio to the ASR service."""

        if self._worker is not None:
            return
        backend = self._backend_factory()
        self._worker = EarWorker(backend=backend, publisher=self._handle_event, logger=_LOGGER)
        self._worker.start()

    def close(self) -> None:
        """Stop streaming and release resources."""

        self._closed = True
        worker = self._worker
        if worker is None:
            return
        worker.stop()
        self._worker = None

    def submit_audio(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Forward audio to the backend worker."""

        if self._worker is None:
            raise RuntimeError("Ear session is not running")
        self._worker.submit_audio(pcm, sample_rate, channels)

    async def next_event(self) -> dict[str, object]:
        """Return the next queued transcription payload."""

        return await self._queue.get()

    def _handle_event(self, event: TranscriptionEvent) -> None:
        try:
            payload = event.to_dict()
        except (TypeError, ValueError) as exc:  # pragma: no cover - defensive guard
            _LOGGER.warning("Failed to encode transcription event: %s", exc)
            return

        try:
            self._relay.publish_transcription(event)
        except Exception:  # pragma: no cover - publication must not break the session
            _LOGGER.exception("Failed to publish browser transcription into ROS")

        def _enqueue() -> None:
            if self._closed:
                return
            try:
                self._queue.put_nowait(payload)
            except asyncio.QueueFull:
                try:
                    self._queue.get_nowait()
                    self._queue.put_nowait(payload)
                except asyncio.QueueEmpty:
                    _LOGGER.warning("Dropping transcription event; queue is full")

        self._loop.call_soon_threadsafe(_enqueue)


async def _ear_page_handler(request: web.Request) -> web.StreamResponse:
    ear_page = _resolve_static_asset("modules/ear/cockpit/browser-ear.html")
    return web.FileResponse(ear_page)


async def _eye_page_handler(request: web.Request) -> web.StreamResponse:
    eye_page = _resolve_static_asset("modules/eye/cockpit/browser-eye.html")
    return web.FileResponse(eye_page)


async def _eye_stream_handler(request: web.Request) -> web.StreamResponse:
    relay: EarRelay = request.app["relay"]

    ws = web.WebSocketResponse(heartbeat=20.0)
    await ws.prepare(request)

    sequence = 0
    try:
        async for msg in ws:
            if msg.type not in {WSMsgType.TEXT, WSMsgType.BINARY}:
                continue
            try:
                payload = json.loads(msg.data if isinstance(msg.data, str) else msg.data.decode("utf-8"))
            except Exception:
                await ws.send_json({"event": "error", "message": "Malformed JSON payload"})
                continue
            try:
                frame = _validate_eye_frame(payload, default_sequence=sequence)
            except web.HTTPBadRequest as exc:
                await ws.send_json({"event": "error", "message": exc.text})
                continue
            sequence = frame.get("sequence", sequence)
            try:
                relay.publish_eye_frame(frame)
            except Exception:  # pragma: no cover - publication must not break the stream
                _LOGGER.exception("Failed to publish browser eye frame into ROS")
                await ws.send_json({"event": "error", "message": "Failed to publish frame"})
                continue
            await ws.send_json({"event": "ack", "sequence": frame.get("sequence")})
    finally:
        await ws.close()
    return ws


async def _ear_stream_handler(request: web.Request) -> web.StreamResponse:
    app = request.app
    relay: EarRelay = app["relay"]
    backend_factory: Callable[[], ServiceASREarBackend] = app["backend_factory"]

    ws = web.WebSocketResponse(heartbeat=20.0)
    await ws.prepare(request)

    session = EarSession(relay=relay, backend_factory=backend_factory, loop=asyncio.get_running_loop())
    session.start()

    sender = asyncio.create_task(_forward_events(ws, session))
    try:
        async for msg in ws:
            if msg.type not in {WSMsgType.TEXT, WSMsgType.BINARY}:
                continue
            if isinstance(msg.data, bytes):
                continue
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                await ws.send_json({"event": "error", "message": "Malformed JSON payload"})
                continue
            try:
                pcm, sample_rate, channels = _decode_audio_payload(payload)
            except ValueError as exc:
                await ws.send_json({"event": "error", "message": str(exc)})
                continue
            try:
                session.submit_audio(pcm, sample_rate, channels)
            except Exception as exc:  # pragma: no cover - defensive guard
                _LOGGER.exception("Failed to submit audio to backend")
                await ws.send_json({"event": "error", "message": str(exc)})
                break
    finally:
        sender.cancel()
        try:
            await sender
        except asyncio.CancelledError:
            pass
        session.close()
        await ws.close()
    return ws


async def _forward_events(ws: web.WebSocketResponse, session: EarSession) -> None:
    while not ws.closed:
        try:
            payload = await session.next_event()
        except asyncio.CancelledError:
            return
        await ws.send_json(payload)


async def _eye_clip_handler(request: web.Request) -> web.Response:
    relay: EarRelay = request.app["relay"]
    try:
        payload = await request.json()
    except Exception as exc:  # pragma: no cover - aiohttp already validates JSON
        raise web.HTTPBadRequest(text=f"Invalid JSON body: {exc}") from exc

    validated = _validate_eye_frame(payload)
    relay.publish_eye_frame(validated)
    return web.json_response({"published": True, "bytes": validated.get("bytes", 0)})


def _validate_eye_frame(payload: Mapping[str, object], *, default_sequence: int = 0) -> MutableMapping[str, object]:
    if not isinstance(payload, Mapping):
        raise web.HTTPBadRequest(text="Payload must be an object")

    kind = str(payload.get("type") or "frame").strip().lower()
    if kind not in {"frame", "chunk", "video"}:
        raise web.HTTPBadRequest(text="type must be frame")

    encoding = str(payload.get("encoding") or "base64").lower().strip()
    if encoding != "base64":
        raise web.HTTPBadRequest(text="Encoding must be base64")

    mime_type = str(payload.get("mime_type") or "video/webm").strip()
    encoded = payload.get("data")
    if not isinstance(encoded, str) or not encoded.strip():
        raise web.HTTPBadRequest(text="data field must contain base64 payload")

    try:
        raw_bytes = base64.b64decode(encoded.strip(), validate=True)
    except Exception as exc:
        raise web.HTTPBadRequest(text="data field must be valid base64") from exc

    byte_size = payload.get("bytes")
    if isinstance(byte_size, (int, float)):
        size_value = int(byte_size)
    else:
        try:
            size_value = int(str(byte_size))
        except Exception:
            size_value = len(raw_bytes)

    if size_value <= 0:
        size_value = len(raw_bytes)

    if size_value > 5 * 1024 * 1024:
        raise web.HTTPBadRequest(text="Frame too large; limit to 5 MiB per chunk")

    sequence_raw = payload.get("sequence", default_sequence + 1)
    try:
        sequence = int(sequence_raw)
    except Exception as exc:  # pragma: no cover - defensive guard
        raise web.HTTPBadRequest(text=f"Invalid sequence: {exc}") from exc

    cleaned: MutableMapping[str, object] = {
        "mime_type": mime_type or "video/webm",
        "data": encoded.strip(),
        "bytes": max(size_value, 0),
        "captured_at": str(payload.get("captured_at") or ""),
        "source": payload.get("source") or "eye-browser",
        "encoding": "base64",
        "sequence": sequence,
    }
    return cleaned


def _decode_audio_payload(payload: Mapping[str, object]) -> tuple[bytes, int, int]:
    kind = str(payload.get("type") or "").strip().lower()
    if kind not in {"chunk", "audio"}:
        raise ValueError("Payload must include type=chunk")

    pcm_text = payload.get("pcm")
    if not isinstance(pcm_text, str) or not pcm_text.strip():
        raise ValueError("PCM chunk missing or empty")
    try:
        pcm_bytes = base64.b64decode(pcm_text.strip(), validate=True)
    except Exception as exc:
        raise ValueError("PCM chunk is not valid base64") from exc

    sample_rate_raw = payload.get("sample_rate")
    channels_raw = payload.get("channels")
    try:
        sample_rate = int(sample_rate_raw)
        channels = int(channels_raw)
    except Exception as exc:
        raise ValueError(f"Invalid sample rate or channels: {exc}") from exc
    if sample_rate <= 0 or channels <= 0:
        raise ValueError("Sample rate and channels must be positive")

    return pcm_bytes, sample_rate, channels


def _resolve_static_asset(relative: str) -> Path:
    candidate = Path(relative)
    if candidate.exists():
        return candidate
    repo_root = Path(__file__).resolve().parents[4]
    path = repo_root / relative
    if not path.exists():
        raise FileNotFoundError(f"Static asset missing: {relative}")
    return path


def _ensure_certificate(cert_path: Path, key_path: Path) -> None:
    cert_path.parent.mkdir(parents=True, exist_ok=True)
    if cert_path.exists() and key_path.exists():
        return

    _LOGGER.info("Generating self-signed certificate for HTTPS harness")
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(
            [
                "openssl",
                "req",
                "-x509",
                "-newkey",
                "rsa:2048",
                "-nodes",
                "-subj",
                "/CN=localhost",
                "-keyout",
                str(key_path),
                "-out",
                str(cert_path),
                "-days",
                "365",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def _build_ssl_context(cert_path: Path, key_path: Path) -> ssl.SSLContext:
    _ensure_certificate(cert_path, key_path)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=str(cert_path), keyfile=str(key_path))
    return context


def create_app(
    *,
    config: BridgeConfig,
    relay: EarRelay,
    backend_factory: Optional[Callable[[], ServiceASREarBackend]] = None,
) -> web.Application:
    """Create an aiohttp application for the browser bridge."""

    app = web.Application()
    app["relay"] = relay
    app["backend_factory"] = backend_factory or (
        lambda: ServiceASREarBackend(uri=config.asr_uri, partial_event_timeout=0.0)
    )

    app.router.add_get("/ear", _ear_page_handler)
    app.router.add_get("/eye", _eye_page_handler)
    app.router.add_get("/ear/stream", _ear_stream_handler)
    app.router.add_get("/eye/stream", _eye_stream_handler)
    app.router.add_post("/eye/clip", _eye_clip_handler)
    app.router.add_get("/health", lambda _request: web.json_response({"status": "ok"}))
    return app


async def _run_server(config: BridgeConfig) -> None:
    rclpy_context = Context()
    rclpy_context.init()
    relay = EarRelay(config=config, context=rclpy_context)

    app = create_app(config=config, relay=relay)
    runner = web.AppRunner(app)
    await runner.setup()
    ssl_context = _build_ssl_context(config.cert_path, config.key_path)
    site = web.TCPSite(
        runner,
        host=config.listen_host,
        port=config.listen_port,
        ssl_context=ssl_context,
    )
    await site.start()

    _LOGGER.info(
        "Ear/Eye browser bridge listening on https://%s:%s (ASR service %s)",
        config.listen_host,
        config.listen_port,
        config.asr_uri,
    )
    stop_event = asyncio.Event()
    try:
        await stop_event.wait()
    finally:
        await runner.cleanup()
        relay.destroy_node()
        rclpy_context.shutdown()


def _parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Browser HTTPS harness for Ear/Eye modules")
    parser.add_argument("--listen-host", default="0.0.0.0", help="Interface to bind the HTTPS server")
    parser.add_argument("--listen-port", type=int, default=9443, help="Port for the HTTPS server")
    parser.add_argument("--asr-uri", default="ws://127.0.0.1:5003/asr", help="ASR websocket endpoint")
    parser.add_argument("--transcript-topic", default="/ear/hole", help="Topic for final transcripts")
    parser.add_argument("--event-topic", default="/ear/asr_event", help="Topic for transcription events")
    parser.add_argument("--eye-topic", default="/eye/browser_clip", help="Topic for browser camera clips")
    parser.add_argument("--cert-path", type=Path, default=Path(".selfsigned/ear_bridge.cert.pem"))
    parser.add_argument("--key-path", type=Path, default=Path(".selfsigned/ear_bridge.key.pem"))
    parser.add_argument("--log-level", default="INFO", help="Logging level")
    return parser.parse_args(argv)


def main(argv: Optional[Iterable[str]] = None) -> None:
    args = _parse_args(argv)
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))
    config = BridgeConfig(
        listen_host=str(args.listen_host),
        listen_port=int(args.listen_port),
        asr_uri=str(args.asr_uri),
        transcript_topic=str(args.transcript_topic),
        event_topic=str(args.event_topic),
        eye_topic=str(args.eye_topic),
        cert_path=Path(args.cert_path),
        key_path=Path(args.key_path),
    )

    asyncio.run(_run_server(config))


__all__ = [
    "BridgeConfig",
    "EarRelay",
    "EarSession",
    "create_app",
    "main",
]
