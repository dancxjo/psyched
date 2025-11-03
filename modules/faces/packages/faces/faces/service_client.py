"""Client utilities for the remote face embedding service."""

from __future__ import annotations

import asyncio
import base64
import json
import uuid
from typing import Any, Coroutine, Mapping, Optional

import cv2
import numpy as np

from .processing import EmbeddingExtractor


class ServiceEmbeddingExtractor(EmbeddingExtractor):
    """Embedding extractor that delegates to the websocket service."""

    def __init__(
        self,
        uri: str,
        *,
        timeout: float = 3.0,
        model: Optional[str] = None,
        num_jitters: Optional[int] = None,
        fallback: EmbeddingExtractor | None = None,
        logger: Any | None = None,
    ) -> None:
        self._uri = uri
        self._timeout = float(max(0.1, timeout))
        self._model = model
        self._num_jitters = num_jitters
        self._fallback = fallback
        self._logger = logger
        self._warned = False
        self._size = 128 if fallback is None else int(fallback.size)

    @property
    def size(self) -> int:
        return self._size

    def embed(self, crop: np.ndarray) -> np.ndarray:
        """Return embeddings from the remote service with optional fallback."""

        try:
            return self._fetch_remote_embedding(crop)
        except Exception as exc:
            if self._fallback is None:
                raise
            if not self._warned and self._logger is not None:
                try:
                    self._logger.warning(
                        "Face embedding service error (%s); falling back to local embeddings (dim=%s)",
                        exc,
                        self._fallback.size,
                    )
                except Exception:
                    pass
                self._warned = True
                self._warned = True
            vector = self._fallback.embed(crop)
            fallback_vec = np.asarray(vector, dtype=np.float32)
            if fallback_vec.size < self._size:
                padded = np.zeros(self._size, dtype=np.float32)
                padded[: fallback_vec.size] = fallback_vec
                return padded
            return fallback_vec[: self._size]

    def _encode_crop(self, crop: np.ndarray) -> str:
        if crop.ndim not in (2, 3):
            raise ValueError("face crop must be grayscale or BGR")
        success, buffer = cv2.imencode(".jpg", crop)
        if not success or buffer.size == 0:
            raise ValueError("failed to encode face crop for service call")
        return base64.b64encode(buffer.tobytes()).decode("ascii")

    async def _call_service(self, payload: Mapping[str, object]) -> Mapping[str, object]:
        import websockets

        async with websockets.connect(
            self._uri,
            open_timeout=self._timeout,
            close_timeout=self._timeout,
            max_size=8 * 1024 * 1024,
            ping_interval=20,
            ping_timeout=20,
        ) as ws:
            await ws.send(json.dumps(payload))
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=self._timeout)
            except asyncio.TimeoutError as exc:
                raise TimeoutError("embedding service response timed out") from exc
        if not isinstance(raw, str):
            raise RuntimeError("embedding service returned binary data")
        try:
            response = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise RuntimeError("embedding service returned invalid JSON") from exc
        return response

    def _run(self, coro: Coroutine[Any, Any, Mapping[str, object]]) -> Mapping[str, object]:
        try:
            return asyncio.run(coro)
        except RuntimeError as exc:
            if "asyncio.run()" in str(exc):
                loop = asyncio.new_event_loop()
                try:
                    asyncio.set_event_loop(loop)
                    return loop.run_until_complete(coro)
                finally:
                    asyncio.set_event_loop(None)
                    loop.close()
            raise

    def _fetch_remote_embedding(self, crop: np.ndarray) -> np.ndarray:
        image_b64 = self._encode_crop(crop)
        payload: dict[str, object] = {
            "id": uuid.uuid4().hex[:12],
            "image": image_b64,
        }
        if self._model:
            payload["model"] = self._model
        if self._num_jitters is not None:
            payload["num_jitters"] = int(self._num_jitters)

        response = self._run(self._call_service(payload))
        event = response.get("event")
        if event != "embedding":
            message = response.get("message") or f"unexpected service response: {response}"
            raise RuntimeError(str(message))

        embedding = response.get("embedding")
        if not isinstance(embedding, list):
            raise RuntimeError("embedding response is missing data")
        vector = np.asarray(embedding, dtype=np.float32)
        if vector.size != self._size:
            raise RuntimeError(
                f"embedding dimension mismatch (expected {self._size}, received {vector.size})"
            )
        return vector
