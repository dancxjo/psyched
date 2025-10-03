"""Asyncio websocket bridge that connects the cockpit UI to ROS 2."""
from __future__ import annotations

import asyncio
import contextlib
import logging
import signal
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from websockets.exceptions import ConnectionClosed
from websockets.server import WebSocketServerProtocol, serve

from .foot import FootTelemetryBridge
from .protocol import (
    InboundMessage,
    OutboundMessage,
    ProtocolError,
    make_error,
    make_message,
    make_ok,
    parse_inbound,
)


_LOGGER = logging.getLogger(__name__)


class CockpitError(RuntimeError):
    """Raised when websocket operations cannot be satisfied."""


@dataclass
class SubscriptionEntry:
    ref_count: int
    subscription: Subscription


@dataclass(eq=False)
class ClientConnection:
    websocket: WebSocketServerProtocol
    queue: asyncio.Queue[OutboundMessage]
    subscriptions: Set[str] = field(default_factory=set)
    sender_task: Optional[asyncio.Task[None]] = None

    def __hash__(self) -> int:  # pragma: no cover - identity hash
        return id(self)


class CockpitBridgeNode(Node):
    """ROS node that owns publishers, subscriptions, and websocket state."""

    def __init__(self) -> None:
        super().__init__("cockpit")
        self._conversation_pub = self.create_publisher(String, "/conversation", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._topic_subscriptions: Dict[str, SubscriptionEntry] = {}
        self._clients: Set[ClientConnection] = set()
        self._broadcast_queue: asyncio.Queue[OutboundMessage] = asyncio.Queue(maxsize=512)
        self._broadcast_task: Optional[asyncio.Task[None]] = None
        self._foot_bridge = FootTelemetryBridge(self, self.enqueue_broadcast)

    # -- client lifecycle -------------------------------------------------
    def register_client(self, websocket: WebSocketServerProtocol) -> ClientConnection:
        client = ClientConnection(websocket=websocket, queue=asyncio.Queue(maxsize=256))
        self._clients.add(client)
        self.get_logger().info(f"client connected ({id(client):#x})")
        return client

    async def unregister_client(self, client: ClientConnection) -> None:
        if client.sender_task:
            client.sender_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await client.sender_task
        for topic in list(client.subscriptions):
            with contextlib.suppress(CockpitError):
                self.unsubscribe_topic(topic)
        self._clients.discard(client)
        self.get_logger().info(f"client disconnected ({id(client):#x})")

    def start_client_sender(self, client: ClientConnection) -> asyncio.Task[None]:
        task = asyncio.create_task(self._client_sender(client))
        client.sender_task = task
        return task

    async def _client_sender(self, client: ClientConnection) -> None:
        try:
            while True:
                message = await client.queue.get()
                await client.websocket.send(message.to_json())
        except asyncio.CancelledError:
            raise
        except ConnectionClosed:
            pass
        finally:
            # Drain to unblock producers before exit
            with contextlib.suppress(asyncio.QueueEmpty):
                while True:
                    client.queue.get_nowait()

    # -- broadcast coordination -----------------------------------------
    def start_background_tasks(self) -> None:
        if self._broadcast_task is None:
            self._broadcast_task = asyncio.create_task(self._broadcast_loop())

    async def stop_background_tasks(self) -> None:
        if self._broadcast_task is not None:
            self._broadcast_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._broadcast_task
            self._broadcast_task = None

    def enqueue_broadcast(self, message: OutboundMessage) -> None:
        if not self._clients:
            return
        try:
            self._broadcast_queue.put_nowait(message)
        except asyncio.QueueFull:
            self.get_logger().warning(
                "dropping broadcast message due to slow consumers (%s)",
                message.topic or "<none>",
            )

    async def _broadcast_loop(self) -> None:
        try:
            while True:
                message = await self._broadcast_queue.get()
                if message.topic is None:
                    targets = list(self._clients)
                else:
                    targets = [c for c in self._clients if message.topic in c.subscriptions]
                for client in targets:
                    try:
                        client.queue.put_nowait(message)
                    except asyncio.QueueFull:
                        self.get_logger().warning(
                            "client queue full; dropping %s for %s",
                            message.topic or "<none>",
                            hex(id(client)),
                        )
                self._broadcast_queue.task_done()
        except asyncio.CancelledError:
            raise

    # -- subscription plumbing ------------------------------------------
    def subscribe_topic(self, topic: str) -> List[OutboundMessage]:
        if FootTelemetryBridge.handles_topic(topic):
            return self._foot_bridge.handle_subscribe(topic)

        entry = self._topic_subscriptions.get(topic)
        if entry is not None:
            entry.ref_count += 1
            return []

        subscription = self._create_subscription(topic)
        self._topic_subscriptions[topic] = SubscriptionEntry(ref_count=1, subscription=subscription)
        return []

    def unsubscribe_topic(self, topic: str) -> None:
        if FootTelemetryBridge.handles_topic(topic):
            self._foot_bridge.handle_unsubscribe(topic)
            return

        entry = self._topic_subscriptions.get(topic)
        if entry is None:
            raise CockpitError(f"unsupported topic '{topic}'")
        if entry.ref_count > 1:
            entry.ref_count -= 1
            return

        self.destroy_subscription(entry.subscription)
        del self._topic_subscriptions[topic]

    def _create_subscription(self, topic: str) -> Subscription:
        if topic == "/audio/transcript/final":
            return self.create_subscription(String, topic, self._on_transcript, 10)
        if topic == "/imu/data":
            return self.create_subscription(Imu, topic, self._on_imu, qos_profile_sensor_data)
        raise CockpitError(f"unsupported topic '{topic}'")

    def _on_transcript(self, msg: String) -> None:
        self.enqueue_broadcast(make_message("/audio/transcript/final", {"data": msg.data}))

    def _on_imu(self, msg: Imu) -> None:
        payload = {
            "header": {
                "frame_id": msg.header.frame_id,
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
            },
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
        }
        self.enqueue_broadcast(make_message("/imu/data", payload))

    # -- publishers ------------------------------------------------------
    def publish_message(self, inbound: InboundMessage) -> None:
        if inbound.topic == "/conversation":
            self._publish_conversation(inbound.msg or {})
        elif inbound.topic == "/cmd_vel":
            self._publish_cmd_vel(inbound.msg or {})
        else:
            raise CockpitError(f"unsupported topic '{inbound.topic}'")

    def _publish_conversation(self, payload: Dict[str, Any]) -> None:
        data = payload.get("data")
        if not isinstance(data, str):
            raise CockpitError("expected string field 'msg.data'")
        ros_msg = String()
        ros_msg.data = data
        self._conversation_pub.publish(ros_msg)
        self.enqueue_broadcast(make_message("/conversation", {"data": data}))

    def _publish_cmd_vel(self, payload: Dict[str, Any]) -> None:
        twist = self._parse_twist(payload)
        self._cmd_vel_pub.publish(twist)
        self._foot_bridge.record_command(twist)

    def _parse_twist(self, payload: Dict[str, Any]) -> Twist:
        linear = self._expect_object(payload, "linear")
        angular = self._expect_object(payload, "angular")
        twist = Twist()
        twist.linear.x = self._expect_number(linear, "x", "msg.linear.x")
        twist.linear.y = self._expect_number(linear, "y", "msg.linear.y")
        twist.linear.z = self._expect_number(linear, "z", "msg.linear.z")
        twist.angular.x = self._expect_number(angular, "x", "msg.angular.x")
        twist.angular.y = self._expect_number(angular, "y", "msg.angular.y")
        twist.angular.z = self._expect_number(angular, "z", "msg.angular.z")
        return twist

    @staticmethod
    def _expect_object(parent: Dict[str, Any], key: str) -> Dict[str, Any]:
        value = parent.get(key)
        if not isinstance(value, dict):
            raise CockpitError(f"expected field 'msg.{key}'")
        return value

    @staticmethod
    def _expect_number(parent: Dict[str, Any], key: str, label: str) -> float:
        value = parent.get(key)
        if not isinstance(value, (int, float)):
            raise CockpitError(f"expected field '{label}'")
        return float(value)

    # -- shutdown --------------------------------------------------------
    async def shutdown(self) -> None:
        await self.stop_background_tasks()
        for client in list(self._clients):
            await self.unregister_client(client)


class WebsocketServer:
    """Thin wrapper around :mod:`websockets` for cockpit clients."""

    def __init__(self, node: CockpitBridgeNode, host: str = "0.0.0.0", port: int = 8088) -> None:
        self._node = node
        self._host = host
        self._port = port
        self._server: Optional[asyncio.base_events.Server] = None

    async def start(self) -> None:
        self._server = await serve(self._handler, self._host, self._port, ping_interval=30, ping_timeout=30)
        _LOGGER.info("listening on ws://%s:%d/ws", self._host, self._port)

    async def stop(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None

    async def _handler(self, websocket: WebSocketServerProtocol, path: str) -> None:
        if path != "/ws":
            await websocket.close(code=1008, reason="unsupported path")
            return

        client = self._node.register_client(websocket)
        sender_task = self._node.start_client_sender(client)
        try:
            async for raw in websocket:
                if not isinstance(raw, str):
                    continue
                await self._process_message(client, raw)
        except ConnectionClosed:
            pass
        finally:
            sender_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await sender_task
            await self._node.unregister_client(client)

    async def _process_message(self, client: ClientConnection, raw: str) -> None:
        try:
            inbound = parse_inbound(raw)
        except ProtocolError as err:
            await client.websocket.send(make_error(str(err)).to_json())
            return

        if inbound.op == "sub":
            await self._handle_subscribe(client, inbound)
        elif inbound.op == "unsub":
            await self._handle_unsubscribe(client, inbound)
        elif inbound.op == "pub":
            await self._handle_publish(client, inbound)

    async def _handle_subscribe(self, client: ClientConnection, inbound: InboundMessage) -> None:
        try:
            initial = self._node.subscribe_topic(inbound.topic)
        except CockpitError as err:
            await client.websocket.send(make_error(str(err)).to_json())
            return

        client.subscriptions.add(inbound.topic)
        await client.websocket.send(make_ok().to_json())
        for message in initial:
            await client.queue.put(message)

    async def _handle_unsubscribe(self, client: ClientConnection, inbound: InboundMessage) -> None:
        if inbound.topic not in client.subscriptions and not FootTelemetryBridge.handles_topic(inbound.topic):
            await client.websocket.send(make_error(f"not subscribed to '{inbound.topic}'").to_json())
            return
        try:
            self._node.unsubscribe_topic(inbound.topic)
        except CockpitError as err:
            await client.websocket.send(make_error(str(err)).to_json())
            return
        client.subscriptions.discard(inbound.topic)
        await client.websocket.send(make_ok().to_json())

    async def _handle_publish(self, client: ClientConnection, inbound: InboundMessage) -> None:
        try:
            self._node.publish_message(inbound)
        except CockpitError as err:
            await client.websocket.send(make_error(str(err)).to_json())
            return
        await client.websocket.send(make_ok().to_json())


async def run_bridge(host: str = "0.0.0.0", port: int = 8088) -> None:
    """Entry point that spins ROS alongside the websocket server."""
    import threading

    rclpy.init()
    node = CockpitBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def request_shutdown() -> None:
        if not stop_event.is_set():
            node.get_logger().info("shutdown requested")
            stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(NotImplementedError):
            loop.add_signal_handler(sig, request_shutdown)

    node.start_background_tasks()
    
    # Spin ROS executor in a separate thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    server = WebsocketServer(node, host=host, port=port)
    await server.start()

    await stop_event.wait()

    await server.stop()
    
    # Shutdown ROS
    executor.shutdown()
    await node.shutdown()
    rclpy.shutdown()
    
    # Wait for ROS thread to finish
    ros_thread.join(timeout=2.0)


__all__ = ["run_bridge", "CockpitBridgeNode", "CockpitError"]
