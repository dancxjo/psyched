"""Websocket service that exposes face embeddings via ``face_recognition``."""

from __future__ import annotations

import asyncio
import base64
import binascii
import json
import logging
import os
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

try:  # pragma: no cover - exercised in integration tests.
    from websockets.asyncio.server import ServerConnection, serve
except ImportError:  # pragma: no cover - compatibility with older websockets builds.
    from websockets.server import WebSocketServerProtocol as ServerConnection
    from websockets.server import serve

from websockets.exceptions import ConnectionClosed

try:  # pragma: no cover - dependency resolved at image build time.
    import face_recognition  # type: ignore[import]
except ModuleNotFoundError:  # pragma: no cover - handled at runtime.
    face_recognition = None  # type: ignore[assignment]

_LOGGER = logging.getLogger("faces.service")


class FaceRecognitionUnavailable(RuntimeError):
    """Raised when the face_recognition backend cannot be used."""


def _decode_image(image_b64: str) -> np.ndarray:
    """Return a BGR image decoded from base64 encoded bytes."""

    if not image_b64:
        raise ValueError("image payload is required")
    try:
        data = base64.b64decode(image_b64, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError("image payload is not valid base64") from exc

    array = np.frombuffer(data, dtype=np.uint8)
    if array.size == 0:
        raise ValueError("image payload is empty")
    image = cv2.imdecode(array, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("image payload could not be decoded")
    return image


@dataclass(frozen=True)
class EmbedRequest:
    """Container describing an embedding request."""

    request_id: Optional[str]
    image_b64: str
    model: Optional[str]
    num_jitters: Optional[int]


def _parse_request(message: str) -> EmbedRequest:
    """Parse JSON payloads into :class:`EmbedRequest` instances."""

    try:
        payload = json.loads(message)
    except json.JSONDecodeError as exc:
        raise ValueError("message must be valid JSON") from exc

    request_id_raw = payload.get("id") or payload.get("request_id")
    request_id = str(request_id_raw).strip() or None if request_id_raw is not None else None

    image_b64 = str(payload.get("image", "")).strip()

    model_raw = payload.get("model")
    model = str(model_raw).strip() or None if isinstance(model_raw, str) else None

    num_jitters_raw = payload.get("num_jitters")
    num_jitters: Optional[int]
    if isinstance(num_jitters_raw, (int, float)):
        num_jitters = int(num_jitters_raw)
    elif isinstance(num_jitters_raw, str) and num_jitters_raw.strip():
        try:
            num_jitters = int(num_jitters_raw.strip())
        except ValueError:
            raise ValueError("num_jitters must be an integer")
    else:
        num_jitters = None

    return EmbedRequest(
        request_id=request_id,
        image_b64=image_b64,
        model=model,
        num_jitters=num_jitters,
    )


class FaceEmbeddingEngine:
    """Encapsulates ``face_recognition`` embedding calls."""

    def __init__(self, *, default_model: str = "small", default_num_jitters: int = 1) -> None:
        if face_recognition is None:
            raise FaceRecognitionUnavailable("face_recognition package is not available")
        if default_model not in {"small", "large"}:
            raise ValueError("default_model must be either 'small' or 'large'")
        self.default_model = default_model
        self.default_num_jitters = max(1, int(default_num_jitters))
        self.embedding_dim = 128

    def embed(
        self,
        image_bgr: np.ndarray,
        *,
        model: Optional[str] = None,
        num_jitters: Optional[int] = None,
    ) -> np.ndarray:
        """Return a float32 embedding for the provided face crop."""

        if face_recognition is None:  # pragma: no cover - defensive.
            raise FaceRecognitionUnavailable("face_recognition package is not available")

        if image_bgr.ndim not in (2, 3):
            raise ValueError("image must be grayscale or BGR")
        if image_bgr.size == 0:
            raise ValueError("image payload is empty")

        chosen_model = (model or self.default_model).lower()
        if chosen_model not in {"small", "large"}:
            raise ValueError("model must be 'small' or 'large'")

        chosen_jitters = int(num_jitters if num_jitters and num_jitters > 0 else self.default_num_jitters)

        if image_bgr.ndim == 2:
            crop_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_GRAY2RGB)
        else:
            crop_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        height, width = crop_rgb.shape[:2]
        if height == 0 or width == 0:
            raise ValueError("image payload is empty")

        try:
            encodings = face_recognition.face_encodings(  # type: ignore[operator]
                crop_rgb,
                known_face_locations=[(0, width, height, 0)],
                num_jitters=chosen_jitters,
                model=chosen_model,
            )
        except Exception as exc:  # pragma: no cover - defensive
            raise RuntimeError(f"face_recognition failed to produce embeddings: {exc}") from exc

        if not encodings:
            raise RuntimeError("face_recognition returned no embeddings")

        embedding = np.asarray(encodings[0], dtype=np.float32)
        if embedding.size != self.embedding_dim:
            _LOGGER.warning(
                "Unexpected embedding size %s (expected %s)", embedding.size, self.embedding_dim
            )
        return embedding


def _build_error(event_id: Optional[str], message: str) -> str:
    payload = {"event": "error", "message": message}
    if event_id is not None:
        payload["id"] = event_id
    return json.dumps(payload)


async def handle_connection(websocket: ServerConnection, engine: FaceEmbeddingEngine) -> None:
    """Handle websocket requests and stream embeddings back to the client."""

    if websocket.path not in {"/faces", "/faces/"}:
        await websocket.close(code=1008, reason="Unsupported websocket endpoint")
        return

    remote = getattr(websocket, "remote_address", None)
    _LOGGER.info("Client connected from %s path=%s", remote, websocket.path)

    while True:
        try:
            message = await websocket.recv()
        except ConnectionClosed:  # pragma: no cover - network dependent.
            _LOGGER.info("Connection closed from %s", remote)
            return

        if not isinstance(message, str):
            await websocket.send(_build_error(None, "Binary frames are not supported"))
            continue

        try:
            request = _parse_request(message)
            image = _decode_image(request.image_b64)
        except Exception as exc:
            await websocket.send(_build_error(None, str(exc)))
            continue

        try:
            embedding = await asyncio.to_thread(
                engine.embed,
                image,
                model=request.model,
                num_jitters=request.num_jitters,
            )
        except Exception as exc:
            await websocket.send(_build_error(request.request_id, str(exc)))
            continue

        response = {
            "event": "embedding",
            "id": request.request_id,
            "embedding": embedding.astype(float).tolist(),
            "dim": int(embedding.size),
            "model": request.model or engine.default_model,
            "num_jitters": int(request.num_jitters or engine.default_num_jitters),
        }
        await websocket.send(json.dumps(response))


async def main() -> None:
    """Start the face embedding websocket service."""

    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO").upper())

    host = os.environ.get("WEBSOCKET_HOST", "0.0.0.0")
    port = int(os.environ.get("WEBSOCKET_PORT", "5005"))
    model = os.environ.get("FACES_MODEL", "small")
    num_jitters = int(os.environ.get("FACES_NUM_JITTERS", "1"))

    try:
        engine = FaceEmbeddingEngine(default_model=model, default_num_jitters=num_jitters)
    except Exception:
        _LOGGER.exception("Failed to initialise face embedding engine")
        raise

    _LOGGER.info(
        "Face embedding websocket ready on %s:%s (model=%s, num_jitters=%s)",
        host,
        port,
        model,
        num_jitters,
    )

    async with serve(
        lambda ws: handle_connection(ws, engine),
        host,
        port,
        max_size=8 * 1024 * 1024,
        ping_interval=20,
        ping_timeout=20,
    ):
        await asyncio.Future()


if __name__ == "__main__":  # pragma: no cover - manual entry point.
    asyncio.run(main())
