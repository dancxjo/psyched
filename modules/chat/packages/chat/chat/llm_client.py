"""Client helpers for the Forebrain LLM websocket service."""
from __future__ import annotations

import asyncio
import json
import time
from typing import Any, Callable, List, Mapping, Sequence

try:  # Optional dependency required for websocket communication.
    from websockets.asyncio.client import connect as websocket_connect
    from websockets.exceptions import ConnectionClosed
except ImportError:  # pragma: no cover - optional dependency for tests.
    websocket_connect = None  # type: ignore
    ConnectionClosed = RuntimeError  # type: ignore


class ForebrainUnavailable(RuntimeError):
    """Raised when the Forebrain websocket service cannot be reached."""


class ForebrainLLMClient:
    """Thin wrapper for streaming completions from the Forebrain LLM service."""

    def __init__(
        self,
        *,
        uri: str,
        connect_timeout: float,
        response_timeout: float,
        logger=None,
        connector=None,
        monotonic: Callable[[], float] | None = None,
    ) -> None:
        if not uri:
            raise ValueError('Forebrain websocket URI must be provided')
        if connector is None and websocket_connect is None:
            raise ForebrainUnavailable('websockets client not available')
        self._uri = uri
        self._connect_timeout = max(0.1, float(connect_timeout))
        self._response_timeout = max(0.1, float(response_timeout))
        self._logger = logger
        self._connector = connector or websocket_connect
        self._monotonic = monotonic or time.monotonic

    def generate(self, history: Sequence[Mapping[str, Any]]) -> str:
        """Return the assistant response for ``history`` messages."""

        if self._connector is None:
            raise ForebrainUnavailable('websocket connector unavailable')
        try:
            return asyncio.run(self._generate_async(list(history)))
        except ForebrainUnavailable:
            raise
        except Exception as exc:
            raise ForebrainUnavailable(str(exc)) from exc

    async def _generate_async(self, history: List[Mapping[str, Any]]) -> str:
        connector = self._connector
        if connector is None:
            raise ForebrainUnavailable('websocket connector unavailable')

        try:
            async with connector(
                self._uri,
                open_timeout=self._connect_timeout,
                close_timeout=self._connect_timeout,
                ping_interval=None,
                ping_timeout=self._response_timeout,
                max_size=None,
            ) as websocket:
                await websocket.send(json.dumps({'command': 'reset'}))
                pending = await self._drain_until_idle(websocket)

                assistant_response: List[str] = []
                if pending and pending.get('role', '').lower() == 'assistant':
                    assistant_response.append(str(pending.get('content', '')))

                for message in history:
                    payload = self._normalise_message(message)
                    await websocket.send(json.dumps(payload))
                    if payload['role'].lower() == 'user':
                        assistant_response.extend(
                            await self._collect_tokens(websocket)
                        )

                return ''.join(assistant_response).strip()
        except ForebrainUnavailable:
            raise
        except ConnectionClosed as exc:  # pragma: no cover - runtime dependent.
            raise ForebrainUnavailable(f'connection closed: {exc}') from exc
        except Exception as exc:
            raise ForebrainUnavailable(str(exc)) from exc

    async def _drain_until_idle(self, websocket) -> dict[str, Any] | None:  # noqa: ANN001
        deadline = self._monotonic() + self._response_timeout
        while True:
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                break
            try:
                message = await asyncio.wait_for(websocket.recv(), timeout=remaining)
            except asyncio.TimeoutError:
                break
            if isinstance(message, (bytes, bytearray)):
                continue
            try:
                payload = json.loads(message)
            except json.JSONDecodeError:
                continue
            if str(payload.get('role', '')).lower() == 'system':
                continue
            return payload
        return None

    async def _collect_tokens(self, websocket) -> List[str]:  # noqa: ANN001
        chunks: List[str] = []
        while True:
            try:
                message = await asyncio.wait_for(websocket.recv(), timeout=self._response_timeout)
            except asyncio.TimeoutError:
                break
            if isinstance(message, (bytes, bytearray)):
                continue
            try:
                payload = json.loads(message)
            except json.JSONDecodeError:
                continue
            role = str(payload.get('role', '')).lower()
            content = str(payload.get('content', ''))
            if role == 'assistant':
                chunks.append(content)
            elif role == 'system':
                # System messages signal either reset confirmations or errors.
                if 'error' in content.lower():
                    raise ForebrainUnavailable(content)
            else:
                # Ignore non-assistant roles to keep the stream aligned.
                continue
        return chunks

    def _normalise_message(self, message: Mapping[str, Any]) -> dict[str, str]:
        role = str(message.get('role', 'user')).strip() or 'user'
        content = str(message.get('content', '')).strip()
        return {'role': role, 'content': content}


__all__ = ['ForebrainLLMClient', 'ForebrainUnavailable']
