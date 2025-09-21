#!/usr/bin/env python3
"""Separate WebSocket node for Pilot.

Runs only the WebSocket server and publishes to ROS topics.
This isolates asyncio from the HTTP server to avoid cross-thread/event-loop issues.
"""

import asyncio
import threading
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except ImportError:
    websockets = None


class PilotWebSocketNode(Node):
    def __init__(self):
        super().__init__('pilot_websocket_node')

        self.declare_parameter('websocket_port', 8081)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('voice_topic', '/voice')
        self.declare_parameter('host', '0.0.0.0')

        def _get(name, default):
            try:
                v = self.get_parameter(name).value
                return v if v is not None else default
            except Exception:
                return default

        self.websocket_port = int(_get('websocket_port', 8081))
        self.cmd_vel_topic = str(_get('cmd_vel_topic', '/cmd_vel'))
        self.voice_topic = str(_get('voice_topic', '/voice'))
        self.host = str(_get('host', '0.0.0.0'))

        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.voice_publisher = self.create_publisher(String, self.voice_topic, 10)

        # asyncio context in dedicated thread
        self._loop = None
        self._server = None
        self._stop_event = None
        self.connected_clients = set()

        # Start asyncio server thread
        self._thread = threading.Thread(target=self._run_ws_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f'WebSocket node starting on ws://{self.host}:{self.websocket_port}')

    def _run_ws_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        self._stop_event = asyncio.Event()
        loop.create_task(self._start_server())
        try:
            loop.run_forever()
        finally:
            try:
                pending = asyncio.all_tasks(loop=loop)
                for t in pending:
                    t.cancel()
                loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            loop.close()

    async def _start_server(self):
        try:
            self._server = await websockets.serve(self._ws_handler, self.host, self.websocket_port)
            self.get_logger().info(f'WebSocket server started on {self.host}:{self.websocket_port}')
            await self._stop_event.wait()
        except Exception as e:
            self.get_logger().error(f'WebSocket server error: {e}')
        finally:
            await self._shutdown_server()

    async def _shutdown_server(self):
        try:
            for client in list(self.connected_clients):
                try:
                    await client.close()
                except Exception:
                    pass
            if self._server is not None:
                self._server.close()
                await self._server.wait_closed()
        except Exception as e:
            self.get_logger().warn(f'Error during WS shutdown: {e}')
        finally:
            self._server = None

    async def _ws_handler(self, websocket: WebSocketServerProtocol, path: Optional[str] = None):
        self.connected_clients.add(websocket)
        self.get_logger().info(f'Client connected: {websocket.remote_address}')
        try:
            status_msg = {
                'type': 'status',
                'cmd_vel_topic': self.cmd_vel_topic,
                'publisher_matched_count': self.cmd_vel_publisher.get_subscription_count() if hasattr(self.cmd_vel_publisher, 'get_subscription_count') else 0,
                'voice_topic': self.voice_topic,
                'voice_subscriber_count': self.voice_publisher.get_subscription_count() if hasattr(self.voice_publisher, 'get_subscription_count') else 0,
            }
            await websocket.send(json.dumps(status_msg))
        except Exception:
            pass
        try:
            async for message in websocket:
                await self._handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            self.get_logger().warn(f'WebSocket error: {e}')
        finally:
            self.connected_clients.discard(websocket)
            self.get_logger().info(f'Client disconnected: {websocket.remote_address}')

    async def _handle_message(self, websocket: WebSocketServerProtocol, message: str):
        try:
            data = json.loads(message)
            t = data.get('type')
            if t == 'cmd_vel':
                twist = Twist()
                lin = data.get('linear', {})
                ang = data.get('angular', {})
                twist.linear.x = float(lin.get('x', 0))
                twist.linear.y = float(lin.get('y', 0))
                twist.linear.z = float(lin.get('z', 0))
                twist.angular.x = float(ang.get('x', 0))
                twist.angular.y = float(ang.get('y', 0))
                twist.angular.z = float(ang.get('z', 0))
                self.cmd_vel_publisher.publish(twist)
                await websocket.send(json.dumps({'type': 'ack', 'cmd_vel': {'linear': lin, 'angular': ang}}))
            elif t == 'ping':
                try:
                    sub_count = self.cmd_vel_publisher.get_subscription_count() if hasattr(self.cmd_vel_publisher, 'get_subscription_count') else 0
                except Exception:
                    sub_count = 0
                await websocket.send(json.dumps({'type': 'pong', 'cmd_vel_topic': self.cmd_vel_topic, 'publisher_matched_count': sub_count, 'voice_topic': self.voice_topic, 'voice_subscriber_count': self.voice_publisher.get_subscription_count() if hasattr(self.voice_publisher, 'get_subscription_count') else 0}))
            elif t == 'voice':
                text = data.get('text')
                if isinstance(text, str) and text.strip():
                    msg = String()
                    msg.data = text.strip()
                    self.voice_publisher.publish(msg)
                    await websocket.send(json.dumps({'type': 'ack', 'voice': {'text': msg.data}}))
        except Exception as e:
            self.get_logger().warn(f'Error handling message: {e}')

    def destroy_node(self):
        # signal shutdown of server
        try:
            if self._loop and self._stop_event:
                self._loop.call_soon_threadsafe(self._stop_event.set)
        except Exception:
            pass
        # join thread
        try:
            if getattr(self, '_thread', None) and self._thread.is_alive():
                self._thread.join(timeout=2.0)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    if websockets is None:
        print('Error: websockets library required. Install with: pip install websockets')
        return 1
    rclpy.init(args=args)
    node = PilotWebSocketNode()
    try:
        # Even single-threaded is fine since asyncio handles its own loop
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
