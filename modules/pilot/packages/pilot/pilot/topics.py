"""ROS topic bridge for the pilot websocket API."""

from __future__ import annotations

import asyncio
import contextlib
import json
from dataclasses import dataclass
from threading import Event, Thread
from typing import Any, Dict, Optional

import rclpy
from aiohttp import web
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from rosidl_runtime_py.utilities import get_message


@dataclass(slots=True)
class TopicBridgeRequest:
    """Parameters describing a websocket bridge session."""

    topic: str
    message_type: str
    role: str

    @classmethod
    def from_request(cls, request: web.Request) -> "TopicBridgeRequest":
        topic = request.query.get("topic")
        message_type = request.query.get("type")
        role = request.query.get("role", "subscribe")
        if not topic or not message_type:
            raise web.HTTPBadRequest(text="Missing required 'topic' or 'type' query parameter")
        if role not in {"subscribe", "publish", "both"}:
            raise web.HTTPBadRequest(text="Unsupported role; expected subscribe, publish, or both")
        return cls(topic=topic, message_type=message_type, role=role)


class RosTopicBridge:
    """Bridge ROS topics to websocket clients."""

    def __init__(self, node_name: str = "pilot_cockpit") -> None:
        self._node_name = node_name
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[Thread] = None
        self._shutdown_event = Event()
        self._ensure_node()

    def _ensure_node(self) -> None:
        if self._node is not None:
            return
        rclpy.init(args=None)
        self._node = rclpy.create_node(self._node_name)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = Thread(target=self._spin, name="pilot-ros-executor", daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        assert self._executor is not None
        while not self._shutdown_event.is_set():
            self._executor.spin_once(timeout_sec=0.1)

    async def handle_websocket(self, request: web.Request) -> web.StreamResponse:
        if self._node is None:
            raise web.HTTPInternalServerError(text="ROS node not initialized")
        params = TopicBridgeRequest.from_request(request)
        ws = web.WebSocketResponse(heartbeat=30.0)
        await ws.prepare(request)

        session = _TopicSession(node=self._node, params=params, loop=asyncio.get_running_loop())
        await session.start(ws)
        return ws

    def shutdown(self) -> None:
        """Stop ROS spinning and destroy the node."""

        self._shutdown_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._executor is not None and self._node is not None:
            self._executor.remove_node(self._node)
            self._executor.shutdown()
            self._executor = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()


class _TopicSession:
    """Manage a single topic bridge session."""

    def __init__(self, *, node: Node, params: TopicBridgeRequest, loop: asyncio.AbstractEventLoop) -> None:
        self._node = node
        self._params = params
        self._loop = loop
        self._subscription = None
        self._publisher = None
        self._queue: asyncio.Queue[Dict[str, Any]] = asyncio.Queue()
        self._message_cls = get_message(params.message_type)

    async def start(self, ws: web.WebSocketResponse) -> None:
        await ws.send_json({"event": "ready", "topic": self._params.topic})
        tasks = []
        if self._params.role in {"subscribe", "both"}:
            self._subscription = self._node.create_subscription(
                self._message_cls,
                self._params.topic,
                self._on_message,
                10,
            )
            tasks.append(asyncio.create_task(self._send_loop(ws)))
        if self._params.role in {"publish", "both"}:
            self._publisher = self._node.create_publisher(
                self._message_cls,
                self._params.topic,
                10,
            )
            tasks.append(asyncio.create_task(self._receive_loop(ws)))

        if tasks:
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task
            for task in done:
                with contextlib.suppress(asyncio.CancelledError):
                    task.result()
        else:
            await ws.close(code=4000, message="No bridge role selected")

        self._cleanup()

    async def _send_loop(self, ws: web.WebSocketResponse) -> None:
        try:
            while True:
                if ws.closed:
                    break
                try:
                    payload = await asyncio.wait_for(self._queue.get(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                if ws.closed:
                    break
                try:
                    await ws.send_json({"event": "message", "data": payload})
                except (ConnectionResetError, ConnectionError):
                    break
                except RuntimeError as exc:
                    if "websocket connection is closing" in str(exc).lower():
                        break
                    raise
        finally:
            if not ws.closed:
                with contextlib.suppress(Exception):
                    await ws.close()

    async def _receive_loop(self, ws: web.WebSocketResponse) -> None:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    await ws.send_json({"event": "error", "message": "Invalid JSON"})
                    continue
                if self._publisher is None:
                    await ws.send_json({"event": "error", "message": "Publisher unavailable"})
                    continue
                ros_message = self._message_cls()
                set_message_fields(ros_message, data)
                self._publisher.publish(ros_message)
            elif msg.type in {web.WSMsgType.CLOSE, web.WSMsgType.CLOSED, web.WSMsgType.ERROR}:
                break
        await ws.close()

    def _on_message(self, message: Any) -> None:
        payload = message_to_ordereddict(message)
        asyncio.run_coroutine_threadsafe(self._queue.put(payload), self._loop)

    def _cleanup(self) -> None:
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        if self._publisher is not None:
            self._node.destroy_publisher(self._publisher)
            self._publisher = None


__all__ = ["RosTopicBridge", "TopicBridgeRequest"]
