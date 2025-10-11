"""Asyncio websocket bridge that connects the cockpit UI to ROS 2."""
from __future__ import annotations

import asyncio
import errno
import contextlib
import json
import logging
import mimetypes
import os
import signal
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set, Type

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from websockets.exceptions import ConnectionClosed
from websockets.server import WebSocketServerProtocol, serve

from .foot import FootTelemetryBridge
from .image import ImageEncoder
from .status import PilotStatusTracker
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

_STATUS_TOPIC = "/pilot/status"
_MODULE_TOPIC_MAP: Dict[str, str] = {
    "/foot/telemetry": "foot",
    "/cmd_vel": "foot",
    "/imu/data": "imu",
    "/image_raw": "eye",
    "/camera/depth/image_raw": "eye",
    "/conversation": "voice",
    "/audio/transcript/final": "ear",
}


class CockpitError(RuntimeError):
    """Raised when websocket operations cannot be satisfied."""


@dataclass(frozen=True)
class TopicRelaySpec:
    """Declarative description of a ROS topic bridged to the cockpit."""

    message_type: Type[Any]
    serializer: Callable[[Any], Dict[str, Any]]
    qos_profile: Any


def _make_transcript_payload(msg: String) -> Dict[str, Any]:
    return {"data": msg.data}


def _make_imu_payload(msg: Imu) -> Dict[str, Any]:
    return {
        "header": {
            "frame_id": msg.header.frame_id,
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
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


_SIMPLE_TOPIC_RELAYS: Dict[str, TopicRelaySpec] = {
    "/audio/transcript/final": TopicRelaySpec(String, _make_transcript_payload, 10),
    "/imu/data": TopicRelaySpec(Imu, _make_imu_payload, qos_profile_sensor_data),
}


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
        self._image_encoder = ImageEncoder(self.get_logger())
        self._image_topics = {"/image_raw", "/camera/depth/image_raw"}
        self._status = PilotStatusTracker()

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
        if topic == _STATUS_TOPIC:
            return [self._status_message()]
        if FootTelemetryBridge.handles_topic(topic):
            messages = self._foot_bridge.handle_subscribe(topic)
            self._register_module_subscription(topic)
            return messages

        entry = self._topic_subscriptions.get(topic)
        if entry is not None:
            entry.ref_count += 1
        else:
            relay = _SIMPLE_TOPIC_RELAYS.get(topic)
            if relay is not None:
                subscription = self.create_subscription(
                    relay.message_type,
                    topic,
                    lambda msg, spec=relay, name=topic: self._handle_simple_topic(name, spec, msg),
                    relay.qos_profile,
                )
            else:
                subscription = self._create_subscription(topic)
            self._topic_subscriptions[topic] = SubscriptionEntry(ref_count=1, subscription=subscription)
        self._register_module_subscription(topic)
        return []

    def unsubscribe_topic(self, topic: str) -> None:
        if topic == _STATUS_TOPIC:
            return
        if FootTelemetryBridge.handles_topic(topic):
            self._foot_bridge.handle_unsubscribe(topic)
            self._unregister_module_subscription(topic)
            return

        entry = self._topic_subscriptions.get(topic)
        if entry is None:
            raise CockpitError(f"unsupported topic '{topic}'")
        if entry.ref_count > 1:
            entry.ref_count -= 1
        else:
            self.destroy_subscription(entry.subscription)
            del self._topic_subscriptions[topic]
        self._unregister_module_subscription(topic)

    def _create_subscription(self, topic: str) -> Subscription:
        if topic in self._image_topics:
            return self.create_subscription(
                Image,
                topic,
                lambda msg, bound_topic=topic: self._on_image(bound_topic, msg),
                qos_profile_sensor_data,
            )
        raise CockpitError(f"unsupported topic '{topic}'")

    def _handle_simple_topic(self, topic: str, spec: TopicRelaySpec, msg: Any) -> None:
        try:
            payload = spec.serializer(msg)
        except Exception:
            self.get_logger().exception("failed to serialise payload for topic %s", topic)
            return
        if not isinstance(payload, dict):
            self.get_logger().warning(
                "serializer for topic %s returned %s; expected dict",
                topic,
                type(payload).__name__,
            )
            return
        self.enqueue_broadcast(make_message(topic, payload))

    def _on_image(self, topic: str, msg: Image) -> None:
        result = self._image_encoder.encode(topic, msg)
        if result is None:
            return
        self.enqueue_broadcast(make_message(topic, result.payload))

    # -- publishers ------------------------------------------------------
    def publish_message(self, inbound: InboundMessage) -> None:
        if inbound.topic == "/conversation":
            self._publish_conversation(inbound.msg or {})
        elif inbound.topic == "/cmd_vel":
            self._publish_cmd_vel(inbound.msg or {})
        elif inbound.topic == _STATUS_TOPIC:
            self._publish_status(inbound.msg or {})
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

    def _publish_status(self, payload: Dict[str, Any]) -> None:
        note = payload.get("note")
        timestamp = payload.get("timestamp")
        if note is not None and not isinstance(note, str):
            raise CockpitError("expected field 'msg.note'")
        if timestamp is not None and not isinstance(timestamp, str):
            raise CockpitError("expected field 'msg.timestamp'")
        self._status.update(note=note, timestamp=timestamp)
        self._broadcast_status()

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

    def _status_message(self) -> OutboundMessage:
        return make_message(_STATUS_TOPIC, self._status.snapshot().to_payload())

    def _broadcast_status(self) -> None:
        self.enqueue_broadcast(self._status_message())

    def _register_module_subscription(self, topic: str) -> None:
        module = _MODULE_TOPIC_MAP.get(topic)
        if self._status.register_module(module):
            self._broadcast_status()

    def _unregister_module_subscription(self, topic: str) -> None:
        module = _MODULE_TOPIC_MAP.get(topic)
        if self._status.unregister_module(module):
            self._broadcast_status()


class UnifiedServer:
    """Unified server that handles both WebSocket and HTTP requests on the same port."""

    def __init__(
        self,
        node: CockpitBridgeNode,
        www_dir: str,
        host: str = "0.0.0.0",
        port: int = 8088,
        hosts_dir: Optional[str] = None
    ) -> None:
        self._node = node
        self._www_dir = Path(www_dir).resolve()
        self._hosts_dir = Path(hosts_dir).resolve() if hosts_dir else None
        self._host = host
        self._port = port
        self._server: Optional[asyncio.base_events.Server] = None

    async def start(self) -> None:
        backoff = 0.5
        attempt = 0
        while True:
            try:
                self._server = await serve(
                    self._ws_handler,
                    self._host,
                    self._port,
                    process_request=self._http_handler,
                    ping_interval=30,
                    ping_timeout=30,
                    reuse_port=True,
                )
            except OSError as err:
                if err.errno != errno.EADDRINUSE or attempt >= 4:
                    raise
                attempt += 1
                _LOGGER.warning(
                    "unified server port %s:%d busy (%s); retrying in %.1fs",
                    self._host,
                    self._port,
                    err.strerror,
                    backoff,
                )
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 4.0)
            else:
                break
        _LOGGER.info("unified server listening on http://%s:%d (ws://%s:%d/ws)", self._host, self._port, self._host, self._port)

    async def stop(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None

    async def _http_handler(self, path: str, request_headers) -> Optional[tuple]:
        """Handle HTTP requests. Return None to let websockets handle it as a WebSocket upgrade."""
        # Check if this is a WebSocket upgrade request
        upgrade = request_headers.get("Upgrade", "").lower()
        if upgrade == "websocket":
            # Let websockets library handle WebSocket upgrade
            return None
        
        # Handle HTTP requests
        try:
            if path == "/api/modules":
                return await self._handle_modules_api()
            else:
                return await self._serve_file(path)
        except Exception as e:
            _LOGGER.exception("Error handling HTTP request for %s: %s", path, e)
            return (500, [], b'Internal Server Error')

    async def _handle_modules_api(self) -> tuple:
        """Handle the /api/modules endpoint."""
        try:
            modules = self._get_active_modules()
            response_data = json.dumps({'modules': modules}).encode('utf-8')
            return (
                200,
                [
                    ('Content-Type', 'application/json'),
                    ('Content-Length', str(len(response_data))),
                ],
                response_data
            )
        except Exception as e:
            _LOGGER.exception("Error in modules API: %s", e)
            return (500, [], b'Internal Server Error')

    def _get_active_modules(self) -> List[str]:
        """Get list of active modules from host config."""
        try:
            import socket
            hostname = socket.gethostname()
            
            if self._hosts_dir:
                config_dir = self._hosts_dir
            else:
                # Fall back to relative path from this file (for development)
                config_dir = Path(__file__).parent.parent.parent.parent.parent.parent / 'hosts'
            
            config_file = config_dir / f'{hostname}.json'
            
            if config_file.exists():
                with open(config_file, 'r') as f:
                    config = json.load(f)
                    host_modules = config.get('host', {}).get('modules', [])
                    module_configs = config.get('modules', {})
                    
                    active = []
                    for module in host_modules:
                        if module == 'pilot':
                            active.append(module)
                            continue
                        mod_config = module_configs.get(module, {})
                        if mod_config.get('launch'):
                            active.append(module)
                    
                    return active
        except Exception as e:
            _LOGGER.warning("Could not load host config: %s", e)
        
        return ['pilot', 'imu', 'foot', 'eye', 'ear']

    async def _serve_file(self, path: str) -> tuple:
        """Serve a static file."""
        if path == '/':
            path = '/index.html'
        
        path = path.lstrip('/')
        file_path = self._www_dir / path

        # Security check
        try:
            file_path = file_path.resolve()
            file_path.relative_to(self._www_dir)
        except (ValueError, OSError):
            return (403, [], b'Forbidden')

        if not file_path.exists() or not file_path.is_file():
            return (404, [], b'Not Found')

        try:
            with open(file_path, 'rb') as f:
                content = f.read()

            content_type, _ = mimetypes.guess_type(str(file_path))
            if content_type is None:
                content_type = 'application/octet-stream'

            return (
                200,
                [
                    ('Content-Type', content_type),
                    ('Content-Length', str(len(content))),
                ],
                content
            )
        except Exception as e:
            _LOGGER.exception("Error serving file %s: %s", file_path, e)
            return (500, [], b'Internal Server Error')

    async def _ws_handler(self, websocket: WebSocketServerProtocol, path: str) -> None:
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


async def run_bridge(
    host: str = "0.0.0.0",
    port: int = 8088,
    www_dir: Optional[str] = None,
    hosts_dir: Optional[str] = None
) -> None:
    """Entry point that spins ROS alongside the unified server (WebSocket + HTTP)."""
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

    # Find the www directory
    if www_dir:
        www_path = Path(www_dir)
    else:
        # Fall back to relative path from this file (for development)
        www_path = Path(__file__).parent.parent.parent.parent.parent / 'www'
    
    if not www_path.exists():
        node.get_logger().error("www directory not found at %s, cannot start server", www_path)
        executor.shutdown()
        await node.shutdown()
        rclpy.shutdown()
        return
    
    # Start unified server (handles both WebSocket and HTTP)
    server = UnifiedServer(node, str(www_path), host=host, port=port, hosts_dir=hosts_dir)
    await server.start()

    await stop_event.wait()

    # Stop server
    await server.stop()
    
    # Shutdown ROS
    executor.shutdown()
    await node.shutdown()
    rclpy.shutdown()
    
    # Wait for ROS thread to finish
    ros_thread.join(timeout=2.0)


__all__ = ["run_bridge", "CockpitBridgeNode", "CockpitError"]
