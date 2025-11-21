"""rclpy helpers that bridge ROS 2 interactions into the cockpit backend."""

from __future__ import annotations

import asyncio
import logging
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message, get_service

_LOGGER = logging.getLogger(__name__)


class RosClientError(RuntimeError):
    """Raised when ROS client operations fail."""


class ServiceCallError(RosClientError):
    """Raised when service invocations fail."""


class TopicStreamError(RosClientError):
    """Raised when topic streaming operations fail."""


class TopicPublishError(RosClientError):
    """Raised when publishing to a ROS topic fails."""


@dataclass(slots=True)
class TopicStreamMetadata:
    """Metadata describing a topic stream exposed to the frontend."""

    id: str
    module: str
    topic: str
    message_type: str
    role: str

    def to_dict(self) -> Dict[str, str]:
        return {
            "id": self.id,
            "module": self.module,
            "topic": self.topic,
            "message_type": self.message_type,
            "role": self.role,
        }


class TopicStream:
    """Wrap a ROS topic subscription/publisher pair for streaming clients."""

    def __init__(
        self,
        *,
        stream_id: str,
        module: str,
        topic: str,
        message_type: str,
        message_cls: Any,
        role: str,
        node,
        qos_profile: QoSProfile,
        loop: asyncio.AbstractEventLoop,
    ) -> None:
        self.id = stream_id
        self.module = module
        self.topic = topic
        self.message_type = message_type
        self.role = role
        self._node = node
        self._loop = loop
        self._message_cls = message_cls
        self._qos_profile = qos_profile
        # Use a bounded queue to provide best-effort delivery to the frontend.
        # When the queue is full we drop the oldest non-status event so slow
        # websocket clients don't cause unbounded memory growth or high
        # latency. The queue size is driven by the queue_length requested by
        # the caller (see RosClient.create_topic_stream).
        max_queue = getattr(qos_profile, "depth", 10)
        self._queue: asyncio.Queue[Dict[str, Any]] = asyncio.Queue(maxsize=max_queue)
        self._closed = False
        self._subscription = None
        self._publisher = None

        if role in {"subscribe", "both"}:
            self._subscription = node.create_subscription(
                message_cls,
                topic,
                self._handle_message,
                qos_profile,
            )

        if role in {"publish", "both"}:
            self._publisher = node.create_publisher(
                message_cls,
                topic,
                qos_profile,
            )

        # Signal readiness to consumers.
        loop.call_soon(self._queue_event, "status", {"state": "ready"})

    @property
    def metadata(self) -> TopicStreamMetadata:
        """Return serialisable metadata for the stream."""

        return TopicStreamMetadata(
            id=self.id,
            module=self.module,
            topic=self.topic,
            message_type=self.message_type,
            role=self.role,
        )

    @property
    def publish_enabled(self) -> bool:
        """Return ``True`` when the stream is configured for publishing."""

        return self._publisher is not None

    async def next_event(self) -> Dict[str, Any]:
        """Return the next queued event emitted by the stream."""

        return await self._queue.get()

    async def publish(self, payload: Mapping[str, Any]) -> None:
        """Publish *payload* to the configured ROS topic."""

        if self._publisher is None:
            raise TopicStreamError("Stream does not support publishing")
        if not isinstance(payload, Mapping):
            raise TopicStreamError("Published payload must be a mapping")

        message = self._message_cls()
        try:
            set_message_fields(message, dict(payload))
        except (AttributeError, TypeError, ValueError) as exc:
            raise TopicStreamError(f"Failed to populate message for topic {self.topic}: {exc}") from exc

        self._publisher.publish(message)

    async def close(self) -> None:
        """Tear down the stream and release ROS resources."""

        if self._closed:
            return
        self._closed = True

        def _destroy_resources() -> None:
            if self._subscription is not None:
                self._node.destroy_subscription(self._subscription)
            if self._publisher is not None:
                self._node.destroy_publisher(self._publisher)

        try:
            await asyncio.get_running_loop().run_in_executor(None, _destroy_resources)
        except RuntimeError:
            # When shutting down the loop, fall back to direct destruction.
            _destroy_resources()

        self._queue_event("status", {"state": "closed"})

    # Internal helpers -------------------------------------------------
    def _handle_message(self, message: Any) -> None:
        data = message_to_ordereddict(message)
        self._loop.call_soon_threadsafe(self._queue_event, "message", data)

    def _queue_event(self, event: str, data: Mapping[str, Any]) -> None:
        if self._closed:
            return
        payload = {"event": event, "data": dict(data)}
        try:
            self._queue.put_nowait(payload)
            return
        except asyncio.QueueFull:
            # Preserve status events (ready/closed) by dropping older
            # application messages preferentially. We attempt to remove one
            # existing non-status message from the queue to make room. If the
            # queue only contains status messages, drop the incoming payload.
            try:
                # Drain one item to make room. Use get_nowait which returns
                # the oldest queued item.
                oldest = self._queue.get_nowait()
            except asyncio.QueueEmpty:
                _LOGGER.warning("Queue full for stream %s and no removable message found; dropping incoming event", self.id)
                return
            else:
                # If the oldest is a status event, we put it back and decide
                # to drop the incoming payload (prefer status retention).
                if oldest.get("event") == "status":
                    try:
                        # Put status item back at the front by making a new
                        # queue with the same items. Simpler approach: put it
                        # back at the end and drop incoming payload.
                        self._queue.put_nowait(oldest)
                    except asyncio.QueueFull:
                        # If we cannot requeue, log and drop incoming.
                        _LOGGER.warning("Failed to requeue status event for stream %s; dropping incoming", self.id)
                    return
                # We successfully removed a non-status oldest message; try to
                # enqueue the incoming payload now. If this still fails, log
                # and drop.
                try:
                    self._queue.put_nowait(payload)
                except asyncio.QueueFull:
                    _LOGGER.warning("Failed to enqueue event for stream %s after dropping oldest; dropping incoming", self.id)


class RosClient:
    """Manage a dedicated rclpy node for cockpit backend interactions."""

    def __init__(self, *, node_name: str = "cockpit_backend") -> None:
        self._context = rclpy.Context()
        rclpy.init(args=None, context=self._context)
        self._node = rclpy.create_node(
            node_name,
            context=self._context,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self._executor = MultiThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin,
            name="cockpit-rclpy-executor",
            daemon=True,
        )
        self._executor_thread.start()
        self._streams: Dict[str, TopicStream] = {}
        self._lock = threading.Lock()
        self._shutdown = False

    @property
    def node(self):
        """Return the shared cockpit ROS node."""

        return self._node

    async def create_topic_stream(
        self,
        *,
        module: str,
        topic: str,
        message_type: str,
        role: str = "subscribe",
        queue_length: int = 10,
        qos: Optional[Mapping[str, Any]] = None,
        loop: Optional[asyncio.AbstractEventLoop] = None,
    ) -> TopicStream:
        """Create a ROS topic stream ready to forward events to the frontend."""

        if self._shutdown:
            raise TopicStreamError("ROS client has been shut down")
        if not topic or not isinstance(topic, str):
            raise TopicStreamError("topic must be a non-empty string")
        if role not in {"subscribe", "publish", "both"}:
            raise TopicStreamError("role must be 'subscribe', 'publish', or 'both'")
        try:
            message_cls = get_message(message_type)
        except (AttributeError, ValueError) as exc:
            raise TopicStreamError(f"Unknown message type: {message_type}") from exc

        # Prefer reliable publishers so command topics (e.g. /voice) reach reliable subscribers.
        reliability_default = (
            QoSReliabilityPolicy.RELIABLE if role in {"publish", "both"} else QoSReliabilityPolicy.BEST_EFFORT
        )
        qos_profile = self._build_qos_profile(
            queue_length=queue_length,
            overrides=qos,
            reliability_default=reliability_default,
        )
        stream_id = uuid.uuid4().hex
        event_loop = loop or asyncio.get_running_loop()
        stream = TopicStream(
            stream_id=stream_id,
            module=module,
            topic=topic,
            message_type=message_type,
            message_cls=message_cls,
            role=role,
            node=self._node,
            qos_profile=qos_profile,
            loop=event_loop,
        )
        with self._lock:
            self._streams[stream_id] = stream
        return stream

    async def call_service(
        self,
        *,
        service_name: str,
        service_type: str,
        arguments: Optional[Mapping[str, Any]] = None,
        timeout: float = 8.0,
    ) -> Mapping[str, Any]:
        """Invoke a ROS service and return the response payload."""

        if self._shutdown:
            raise ServiceCallError("ROS client has been shut down")

        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(
            None,
            self._call_service_sync,
            service_name,
            service_type,
            dict(arguments or {}),
            float(timeout),
        )

    async def publish_topic(
        self,
        *,
        module: str,
        topic: str,
        message_type: str,
        payload: Mapping[str, Any],
        qos: Optional[Mapping[str, Any]] = None,
    ) -> None:
        """Publish a single message to a ROS topic on behalf of a module."""

        if self._shutdown:
            raise TopicPublishError("ROS client has been shut down")
        if not topic or not isinstance(topic, str):
            raise TopicPublishError("topic must be a non-empty string")
        if not message_type or not isinstance(message_type, str):
            raise TopicPublishError("message_type must be a non-empty string")
        if payload is None or not isinstance(payload, Mapping):
            raise TopicPublishError("payload must be expressed as a JSON object")

        qos_mapping = dict(qos) if isinstance(qos, Mapping) else None
        _LOGGER.debug(
            "Publishing cockpit action payload to %s for module %s", topic, module
        )
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            self._publish_topic_sync,
            topic,
            message_type,
            dict(payload),
            qos_mapping,
        )

    async def close_stream(self, stream_id: str) -> None:
        """Close and drop a previously created topic stream."""

        stream = None
        with self._lock:
            stream = self._streams.pop(stream_id, None)
        if stream:
            await stream.close()

    async def shutdown(self) -> None:
        """Stop the rclpy executor and clean up resources."""

        if self._shutdown:
            return
        self._shutdown = True

        for stream_id in list(self._streams.keys()):
            await self.close_stream(stream_id)

        self._executor.remove_node(self._node)
        self._executor.shutdown()
        if self._executor_thread.is_alive():
            self._executor_thread.join(timeout=2.0)
        self._node.destroy_node()
        rclpy.shutdown(context=self._context)

    # Internal helpers -------------------------------------------------
    def _build_qos_profile(
        self,
        *,
        queue_length: int,
        overrides: Optional[Mapping[str, Any]],
        reliability_default: QoSReliabilityPolicy = QoSReliabilityPolicy.BEST_EFFORT,
    ) -> QoSProfile:
        depth = max(int(queue_length), 1)
        # Default to best-effort reliability so we can subscribe to sensor topics
        # that publish with SensorDataQoS without requiring per-stream overrides.
        profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=reliability_default,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        if overrides and isinstance(overrides, Mapping):
            reliability = overrides.get("reliability")
            if isinstance(reliability, str):
                profile.reliability = self._coerce_reliability(reliability)
            durability = overrides.get("durability")
            if isinstance(durability, str):
                profile.durability = self._coerce_durability(durability)
        return profile

    @staticmethod
    def _coerce_reliability(value: str) -> QoSReliabilityPolicy:
        normalized = value.strip().lower()
        if normalized in {"best_effort", "best-effort", "best"}:
            return QoSReliabilityPolicy.BEST_EFFORT
        return QoSReliabilityPolicy.RELIABLE

    @staticmethod
    def _coerce_durability(value: str) -> QoSDurabilityPolicy:
        normalized = value.strip().lower()
        if normalized in {"transient_local", "transient-local", "transient"}:
            return QoSDurabilityPolicy.TRANSIENT_LOCAL
        return QoSDurabilityPolicy.VOLATILE

    def _call_service_sync(
        self,
        service_name: str,
        service_type: str,
        arguments: Dict[str, Any],
        timeout: float,
    ) -> Mapping[str, Any]:
        try:
            service_cls = get_service(service_type)
        except (AttributeError, ValueError) as exc:
            raise ServiceCallError(f"Unknown service type: {service_type}") from exc

        client = self._node.create_client(service_cls, service_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise ServiceCallError(f"Service {service_name} not available after {timeout} seconds")

            request = service_cls.Request()
            try:
                set_message_fields(request, arguments)
            except (AttributeError, TypeError, ValueError) as exc:
                raise ServiceCallError(f"Failed to populate service request: {exc}") from exc

            future = client.call_async(request)
            start_time = time.monotonic()
            while not future.done():
                if (time.monotonic() - start_time) >= timeout:
                    raise ServiceCallError(f"Service call to {service_name} timed out after {timeout} seconds")
                time.sleep(0.01)

            response = future.result()
            return message_to_ordereddict(response)
        finally:
            self._node.destroy_client(client)

    def _publish_topic_sync(
        self,
        topic: str,
        message_type: str,
        payload: Dict[str, Any],
        qos: Optional[Mapping[str, Any]],
    ) -> None:
        try:
            message_cls = get_message(message_type)
        except (AttributeError, ValueError) as exc:
            raise TopicPublishError(f"Unknown message type: {message_type}") from exc

        qos_profile = self._build_qos_profile(
            queue_length=1,
            overrides=qos,
            reliability_default=QoSReliabilityPolicy.RELIABLE,
        )
        publisher = self._node.create_publisher(message_cls, topic, qos_profile)
        try:
            message = message_cls()
            try:
                set_message_fields(message, payload)
            except (AttributeError, TypeError, ValueError) as exc:
                raise TopicPublishError(f"Failed to populate message for topic {topic}: {exc}") from exc
            publisher.publish(message)
        finally:
            self._node.destroy_publisher(publisher)
