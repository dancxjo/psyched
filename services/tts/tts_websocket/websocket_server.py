"""Minimal websocket server that streams Coqui TTS audio as PCM frames.

The server loads the requested 🐸Coqui model once at startup and keeps it in
memory so we can service multiple text synthesis requests over a single
websocket connection without reinitialising weights. Each incoming text message
(plain text or JSON) is synthesised and streamed to the client as little-endian
16-bit PCM frames, preceded by a JSON metadata envelope that explains how to
interpret the binary payload.

Example
-------
Run the server (this file is used as the Docker entrypoint)::

    python3 websocket_server.py

With the server up, send text and record the stream::

    import asyncio, json, subprocess
    import websockets

    async def demo():
        async with websockets.connect("ws://forebrain.local:5002/tts") as ws:
            await ws.send(json.dumps({"text": "Streaming feels instant"}))
            meta = json.loads(await ws.recv())
            ffmpeg = subprocess.Popen(
                [
                    "ffmpeg",
                    "-f",
                    "s16le",
                    "-ar",
                    str(meta["sample_rate"]),
                    "-ac",
                    str(meta["channels"]),
                    "-i",
                    "pipe:0",
                    "demo.wav",
                ],
                stdin=subprocess.PIPE,
            )
            while True:
                message = await ws.recv()
                if isinstance(message, bytes):
                    ffmpeg.stdin.write(message)
                else:
                    break

    asyncio.run(demo())
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np
from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed

_LOGGER = logging.getLogger("coqui_tts.websocket")


@dataclass(frozen=True)
class SynthesisRequest:
    """Container describing a single synthesis request from a client."""

    text: str
    speaker: Optional[str]
    language: Optional[str]


def parse_client_message(
    message: str,
    *,
    default_speaker: Optional[str],
    default_language: Optional[str],
) -> SynthesisRequest:
    """Parse a websocket message into a :class:`SynthesisRequest`.

    Clients may send either a JSON payload or plain text. JSON payloads support
    ``{"text": "...", "speaker": "...", "language": "..."}``. Missing
    fields fall back to the defaults supplied by the caller.
    """

    try:
        payload = json.loads(message)
    except json.JSONDecodeError:
        payload = {"text": message}

    text = str(payload.get("text", "")).strip()
    if not text:
        raise ValueError("Text-to-speech requests must include non-empty text.")

    speaker = payload.get("speaker", default_speaker)
    language = payload.get("language", default_language)

    return SynthesisRequest(text=text, speaker=speaker, language=language)


def encode_pcm16(audio: np.ndarray) -> bytes:
    """Convert floating-point audio samples to little-endian PCM16 bytes."""

    if not isinstance(audio, np.ndarray):
        audio = np.asarray(audio, dtype=np.float32)
    if audio.dtype != np.float32:
        audio = audio.astype(np.float32)
    clipped = np.clip(audio, -1.0, 1.0)
    pcm = (clipped * 32767.0).astype("<i2")
    return pcm.tobytes()


def chunk_pcm_bytes(data: bytes, *, chunk_samples: int) -> Iterable[bytes]:
    """Yield successive PCM frames containing ``chunk_samples`` mono samples."""

    if chunk_samples <= 0:
        raise ValueError("chunk_samples must be a positive integer")
    step = chunk_samples * 2  # 2 bytes/sample for mono PCM16.
    for start in range(0, len(data), step):
        yield data[start : start + step]


class TTSSynthesizer:
    """Thin wrapper over :mod:`TTS.api` that keeps the model warm in memory."""

    def __init__(self, tts) -> None:
        self.tts = tts
        self.sample_rate = int(getattr(tts.synthesizer, "output_sample_rate"))

    @classmethod
    def from_env(cls) -> "TTSSynthesizer":
        """Initialize the synthesiser using environment configuration."""

        model_name = os.environ.get("TTS_MODEL", "tts_models/en/vctk/vits")
        # If the operator has mounted a local models directory into the
        # container (e.g. via docker-compose), prefer a matching local model
        # path to avoid pulling from the Coqui registry. The download scripts
    # in this repo place TTS assets under services/tts/models which
        # is mounted as /models in the compose stack.
        tts_model_dir = os.environ.get("TTS_MODEL_DIR")
        if tts_model_dir:
            try:
                # Model name format examples:
                #  - tts_models/multilingual/multi-dataset/xtts_v2
                #  - tts_models/en/ljspeech/tacotron2-DDC
                model_key = model_name.split("/")[-1]
                candidate = os.path.join(tts_model_dir, model_key)
                if os.path.exists(candidate):
                    _LOGGER.info(
                        "Found local model directory for '%s' at '%s'",
                        model_key,
                        candidate,
                    )
                    model_name = candidate
                else:
                    # also allow a file named like <model_key>.zip or .pth
                    for ext in (".zip", ".pth", ""):
                        candidate_file = os.path.join(tts_model_dir, model_key + ext)
                        if os.path.exists(candidate_file):
                            _LOGGER.info(
                                "Found local model file for '%s' at '%s'",
                                model_key,
                                candidate_file,
                            )
                            model_name = candidate_file
                            break
            except Exception:  # pragma: no cover - defensive; fall back to registry name
                _LOGGER.exception(
                    "Error while checking local TTS_MODEL_DIR; will try registry model name '%s'",
                    model_name,
                )
        use_cuda = os.environ.get("TTS_USE_CUDA", "true").lower() in {
            "1",
            "true",
            "yes",
            "on",
        }

        from TTS.api import TTS as CoquiTTS  # Imported lazily to keep tests lightweight.

        _LOGGER.info("Loading Coqui model '%s' (CUDA=%s)", model_name, use_cuda)
        try:
            tts = CoquiTTS(model_name=model_name, progress_bar=False, gpu=use_cuda)
        except KeyError as exc:  # pragma: no cover - model name mismatch at runtime
            _LOGGER.warning(
                "Requested model '%s' is not available in this TTS package: %s. Falling back to a safe default model.",
                model_name,
                exc,
            )
            # Fallback to a commonly-available English TTS model so the
            # websocket service can still boot even if the configured model is
            # missing from the local image or volume.
            fallback = os.environ.get(
                "TTS_FALLBACK_MODEL",
                "tts_models/en/ljspeech/tacotron2-DDC",
            )
            _LOGGER.info("Attempting fallback Coqui model '%s' (CUDA=%s)", fallback, use_cuda)
            tts = CoquiTTS(model_name=fallback, progress_bar=False, gpu=use_cuda)
        return cls(tts)

    def synthesize(self, request: SynthesisRequest) -> np.ndarray:
        """Generate audio for ``request`` and return float32 PCM samples."""

        kwargs = {}
        if request.speaker:
            kwargs["speaker"] = request.speaker

        # Intentionally avoid forwarding the language hint to the TTS backend.
        # Single-language models such as our default VITS voice will raise when
        # given a language override, so we keep the request value for logging but
        # omit it from Coqui's invocation.

        wav = self.tts.tts(text=request.text, **kwargs)
        audio = np.asarray(wav, dtype=np.float32).reshape(-1)
        return audio


async def _stream_response(
    websocket: ServerConnection,
    synthesizer: TTSSynthesizer,
    request: SynthesisRequest,
    *,
    chunk_samples: int,
) -> None:
    """Synthesise ``request`` and stream PCM frames back to ``websocket``."""

    try:
        audio = await asyncio.to_thread(synthesizer.synthesize, request)
    except Exception as exc:  # pragma: no cover - defensive logging path.
        _LOGGER.exception("Synthesis failed")
        await websocket.send(json.dumps({"event": "error", "message": str(exc)}))
        return

    pcm = encode_pcm16(audio)
    metadata = {
        "event": "start",
        "sample_rate": synthesizer.sample_rate,
        "channels": 1,
        "format": "pcm_s16le",
        "num_samples": int(len(pcm) / 2),
    }
    await websocket.send(json.dumps(metadata))

    for chunk in chunk_pcm_bytes(pcm, chunk_samples=chunk_samples):
        await websocket.send(chunk)

    await websocket.send(
        json.dumps(
            {
                "event": "end",
                "num_samples": metadata["num_samples"],
                "duration_s": metadata["num_samples"] / synthesizer.sample_rate,
            }
        )
    )


async def handle_connection(
    websocket: ServerConnection,
    synthesizer: TTSSynthesizer,
    *,
    default_speaker: Optional[str],
    default_language: Optional[str],
    chunk_samples: int,
) -> None:
    """Main websocket handler that accepts text and streams PCM responses."""

    while True:
        try:
            message = await websocket.recv()
        except ConnectionClosed:  # pragma: no cover - depends on network.
            _LOGGER.info("Connection closed")
            return

        if not isinstance(message, str):
            await websocket.send(
                json.dumps({"event": "error", "message": "Binary input is not supported."})
            )
            continue

        try:
            request = parse_client_message(
                message,
                default_speaker=default_speaker,
                default_language=default_language,
            )
        except ValueError as exc:
            await websocket.send(json.dumps({"event": "error", "message": str(exc)}))
            continue

        await _stream_response(
            websocket,
            synthesizer,
            request,
            chunk_samples=chunk_samples,
        )


async def main() -> None:
    """Entry point used by the Docker image to start the websocket server."""

    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"))

    default_speaker = os.environ.get("TTS_DEFAULT_SPEAKER", "p330")
    default_language = os.environ.get("TTS_DEFAULT_LANGUAGE")
    chunk_samples = int(os.environ.get("PCM_CHUNK_SAMPLES", "4096"))
    host = os.environ.get("WEBSOCKET_HOST", "0.0.0.0")
    port = int(os.environ.get("WEBSOCKET_PORT", "5002"))

    synthesizer = TTSSynthesizer.from_env()

    _LOGGER.info(
        "Streaming websocket ready on %s:%s (speaker=%s, chunk=%s samples)",
        host,
        port,
        default_speaker,
        chunk_samples,
    )

    async with serve(
        lambda ws: handle_connection(
            ws,
            synthesizer,
            default_speaker=default_speaker,
            default_language=default_language,
            chunk_samples=chunk_samples,
        ),
        host,
        port,
        max_size=None,
        ping_interval=20,
        ping_timeout=20,
    ):
        await asyncio.Future()


if __name__ == "__main__":  # pragma: no cover - manual invocation guard.
    asyncio.run(main())
