"""HTTP client helpers for the shared Ollama service."""
from __future__ import annotations

import json
from contextlib import closing
from typing import Callable, Mapping, MutableMapping, Optional, Sequence

try:  # Optional dependency at runtime.
    import requests
except ImportError:  # pragma: no cover - handled in tests via dependency injection.
    requests = None  # type: ignore


class OllamaServiceError(RuntimeError):
    """Raised when the Ollama service rejects or fails to serve a request."""


class OllamaServiceClient:
    """Lightweight helper for issuing chat requests to the services/llm runtime."""

    def __init__(
        self,
        *,
        host: str,
        session: Optional[object] = None,
        timeout: float = 30.0,
        logger=None,
    ) -> None:
        if not host:
            raise ValueError("Ollama host must be provided")
        self._base_url = host.rstrip("/")
        self._session = session
        self._timeout = float(timeout)
        self._logger = logger

    def chat(
        self,
        *,
        model: str,
        messages: Sequence[Mapping[str, str]],
        stream_callback: Optional[Callable[[str], None]] = None,
    ) -> str:
        """Return the assistant's response for ``messages`` using ``model``.

        When ``stream_callback`` is provided, stream partial tokens to the
        callback while still returning the full assistant message at the end.
        """

        payload: MutableMapping[str, object] = {
            "model": model,
            "messages": [
                {
                    "role": str(entry.get("role", "user")),
                    "content": str(entry.get("content", "")),
                }
                for entry in messages
            ],
            "stream": stream_callback is not None,
        }

        http_post = self._resolve_post()
        stream = bool(stream_callback)
        try:
            response = http_post(
                f"{self._base_url}/api/chat",
                json=payload,
                stream=stream,
                timeout=self._timeout,
            )
        except Exception as exc:  # pragma: no cover - network errors depend on runtime.
            raise OllamaServiceError(f"Failed to reach Ollama at {self._base_url}: {exc}") from exc

        tokens: list[str] = []
        with closing(response):
            try:
                response.raise_for_status()
            except Exception as exc:
                raise OllamaServiceError(
                    f"Ollama returned HTTP {getattr(response, 'status_code', 'error')}"
                ) from exc

            if stream:
                for chunk in response.iter_lines(decode_unicode=True):
                    text = self._decode_chunk(chunk)
                    if not text:
                        continue
                    tokens.append(text)
                    if stream_callback is not None:
                        try:
                            stream_callback(text)
                        except Exception:
                            if self._logger is not None:
                                try:
                                    self._logger.warning("stream callback raised", exc_info=True)
                                except Exception:  # pragma: no cover - logging failure path.
                                    pass
                return "".join(tokens).strip()

            data = response.json()
            text = self._extract_text(data)
            if text:
                tokens.append(text)

        return "".join(tokens).strip()

    def _resolve_post(self):
        if self._session is not None:
            return self._session.post  # type: ignore[return-value]
        if requests is None:  # pragma: no cover - exercised when requests missing.
            raise OllamaServiceError("requests library not available")
        return requests.post

    def _decode_chunk(self, raw_chunk: object) -> str:
        if raw_chunk is None:
            return ""
        if isinstance(raw_chunk, bytes):
            try:
                raw_chunk = raw_chunk.decode("utf-8")
            except Exception:
                return ""
        text = str(raw_chunk).strip()
        if not text:
            return ""
        if text.startswith("data:"):
            text = text[5:].strip()
        if not text:
            return ""
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return text
        extracted = self._extract_text(payload)
        return extracted

    def _extract_text(self, payload: object) -> str:
        if isinstance(payload, str):
            return payload
        if not isinstance(payload, Mapping):
            return ""

        message = payload.get("message")
        if isinstance(message, Mapping):
            content = message.get("content")
            if isinstance(content, str):
                return content

        response = payload.get("response")
        if isinstance(response, str):
            return response

        content = payload.get("content")
        if isinstance(content, str):
            return content

        choices = payload.get("choices")
        if isinstance(choices, Sequence) and choices:
            first = choices[0]
            if isinstance(first, Mapping):
                message = first.get("message")
                if isinstance(message, Mapping):
                    content = message.get("content")
                    if isinstance(content, str):
                        return content
                delta = first.get("delta")
                if isinstance(delta, Mapping):
                    content = delta.get("content")
                    if isinstance(content, str):
                        return content

        return ""


__all__ = ["OllamaServiceClient", "OllamaServiceError"]
