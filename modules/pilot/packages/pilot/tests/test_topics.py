"""Unit tests for topic bridging helpers."""

from __future__ import annotations

import asyncio
import importlib.util
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[5]
PACKAGE_DIR = REPO_ROOT / "modules" / "pilot" / "packages" / "pilot" / "pilot"

spec = importlib.util.spec_from_file_location(
    "pilot",
    PACKAGE_DIR / "__init__.py",
    submodule_search_locations=[str(PACKAGE_DIR)],
)
module = importlib.util.module_from_spec(spec)
sys.modules.pop("pilot", None)
sys.modules["pilot"] = module
spec.loader.exec_module(module)

from pilot.topics import _TopicSession  # type: ignore[attr-defined]


class _DummyWebSocket:
    def __init__(self) -> None:
        self.closed = False
        self.close_calls = 0

    async def send_json(self, data: object, **_: object) -> None:
        raise ConnectionResetError("Cannot write to closing transport")

    async def close(self) -> None:
        self.closed = True
        self.close_calls += 1


def test_send_loop_handles_connection_reset() -> None:
    asyncio.run(_exercise_send_loop_handles_connection_reset())


async def _exercise_send_loop_handles_connection_reset() -> None:
    session = object.__new__(_TopicSession)
    session._queue = asyncio.Queue()  # type: ignore[attr-defined]

    ws = _DummyWebSocket()
    await session._queue.put({"foo": "bar"})  # type: ignore[attr-defined]

    await session._send_loop(ws)  # type: ignore[attr-defined]
    assert ws.closed
    assert ws.close_calls == 1
