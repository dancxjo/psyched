#!/usr/bin/env python3
"""Pilot web interface node with joystick control.

Provides a web interface with joystick control that sends cmd_vel commands via WebSocket.
The web server runs on 0.0.0.0 to allow external access, and publishes geometry_msgs/Twist
messages for robot movement control.
"""

import asyncio
import json
import os
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from math import atan2

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except ImportError:
    websockets = None

import http.server
import socketserver
from urllib.parse import urlparse


class PilotNode(Node):
    """ROS2 node that provides web interface with joystick control."""
    
    def __init__(self):
        super().__init__('pilot_node')
        
        # Parameters (robust to string/typed inputs from launch substitutions)
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('websocket_port', 8081)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('voice_topic', '/voice')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('enable_http', True)
        self.declare_parameter('enable_websocket', True)
        self.declare_parameter('imu_topic', '/imu')

        def _get_param_value(name):
            # rclpy returns a Python value at .value regardless of declared type
            try:
                return self.get_parameter(name).value
            except Exception:
                return None

        def _as_int(val, default):
            try:
                return int(val)
            except (TypeError, ValueError):
                return default

        def _as_str(val, default):
            try:
                s = str(val)
                return s if s else default
            except Exception:
                return default

        def _as_bool(val, default):
            try:
                if isinstance(val, bool):
                    return val
                if isinstance(val, (int, float)):
                    return bool(val)
                if isinstance(val, str):
                    v = val.strip().lower()
                    if v in ('1', 'true', 'yes', 'on'): return True
                    if v in ('0', 'false', 'no', 'off'): return False
                return default
            except Exception:
                return default

        self.web_port = _as_int(_get_param_value('web_port'), 8080)
        self.websocket_port = _as_int(_get_param_value('websocket_port'), 8081)
        self.cmd_vel_topic = _as_str(_get_param_value('cmd_vel_topic'), '/cmd_vel')
        self.voice_topic = _as_str(_get_param_value('voice_topic'), '/voice')
        self.imu_topic = _as_str(_get_param_value('imu_topic'), '/imu')
        self.host = _as_str(_get_param_value('host'), '0.0.0.0')
        self.enable_http = _as_bool(_get_param_value('enable_http'), True)
        self.enable_websocket = _as_bool(_get_param_value('enable_websocket'), True)
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.voice_publisher = self.create_publisher(String, self.voice_topic, 10)
        # Subscribers
        self._imu_last = None  # cache last IMU message dictionary
        try:
            self.imu_sub = self.create_subscription(Imu, self.imu_topic, self._on_imu, 10)
        except Exception as e:
            self.get_logger().warn(f'Failed to subscribe to IMU: {e}')

        # Web and WebSocket servers
        self.web_server = None  # http.server instance
        self.websocket_server = None  # websockets.server.Server (set inside loop)
        self.web_thread = None
        self.websocket_thread = None
        self._ws_loop = None  # asyncio loop used by WS server
        self._ws_stop_event = None  # asyncio.Event to signal WS shutdown
        
        # Connected WebSocket clients
        self.connected_clients = set()
        
        # Static files path
        self.static_path = self._find_static_path()
        
        # Start servers
        self.start_servers()

        # Periodic push of IMU to clients (20 Hz)
        self.create_timer(0.05, self._tick_broadcast)
        
        self.get_logger().info('Pilot node started:')
        self.get_logger().info(f'  Static path: {self.static_path}')
        self.get_logger().info(f'  Web interface: http://{self.host}:{self.web_port}')
        self.get_logger().info(f'  WebSocket: ws://{self.host}:{self.websocket_port}')
        self.get_logger().info(f'  Publishing to: {self.cmd_vel_topic}')
        self.get_logger().info(f'  Voice topic: {self.voice_topic}')
        self.get_logger().info(f'  IMU topic: {self.imu_topic}')

    def _on_imu(self, msg: Imu):
        # Extract linear acceleration
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        # Angular velocity
        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)
        # Orientation yaw from quaternion (if provided)
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)
        # Robust yaw (z) extraction
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        try:
            yaw = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        except Exception:
            yaw = 0.0
        self._imu_last = {
            'ax': ax, 'ay': ay, 'az': az,
            'gx': gx, 'gy': gy, 'gz': gz,
            'yaw': yaw,
        }

    async def _send_ws_json(self, websocket, payload):
        try:
            await websocket.send(json.dumps(payload))
        except Exception:
            pass

    def _tick_broadcast(self):
        # Broadcast IMU if we have clients and data
        if not self.connected_clients or not self._imu_last:
            return
        payload = { 'type': 'imu', **self._imu_last }
        # Send to all clients via the WS loop thread-safely
        if self._ws_loop is not None:
            for ws in list(self.connected_clients):
                try:
                    self._ws_loop.call_soon_threadsafe(lambda w=ws, p=payload: asyncio.create_task(self._send_ws_json(w, p)))
                except Exception:
                    pass
    
    def _find_static_path(self) -> Path:
        """Find the static files directory."""
        # Try relative to this file first
        current_dir = Path(__file__).parent
        static_path = current_dir / 'static'
        if static_path.exists():
            return static_path
        
        # Try in share directory (installed)
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = Path(get_package_share_directory('pilot'))
            static_path = share_dir / 'static'
            if static_path.exists():
                return static_path
        except Exception:
            pass
        
        # Fallback to current directory
        static_path = Path.cwd() / 'static'
        static_path.mkdir(exist_ok=True)
        return static_path
    
    def start_servers(self):
        """Start web and WebSocket servers in separate threads."""
        # Start web server
        if self.enable_http:
            self.web_thread = threading.Thread(target=self._run_web_server, daemon=True)
            self.web_thread.start()
        else:
            self.get_logger().info('HTTP server disabled by parameter enable_http=false')
        
        # Start WebSocket server
        if self.enable_websocket and websockets:
            self.websocket_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
            self.websocket_thread.start()
        elif not self.enable_websocket:
            self.get_logger().info('WebSocket server disabled by parameter enable_websocket=false')
        else:
            self.get_logger().warn('websockets library not available. Install with: pip install websockets')
    
    def _run_web_server(self):
        """Run the HTTP web server."""
        try:
            handler = self._create_http_handler()
            # Prefer a threading server to avoid blocking
            HTTPServer = getattr(http.server, 'ThreadingHTTPServer', http.server.HTTPServer)
            httpd = HTTPServer((self.host, self.web_port), handler)
            self.web_server = httpd
            self.get_logger().info(f'Web server started on {self.host}:{self.web_port}')
            try:
                httpd.serve_forever()
            finally:
                httpd.server_close()
        except Exception as e:
            self.get_logger().error(f'Web server error: {e}')
    
    def _create_http_handler(self):
        """Create HTTP request handler class."""
        static_path = self.static_path
        
        class PilotHTTPHandler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_path), **kwargs)
            
            def log_message(self, format, *args):
                pass  # Suppress HTTP logs
        
        return PilotHTTPHandler
    
    def _run_websocket_server(self):
        """Run the WebSocket server."""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._ws_loop = loop
            self._ws_stop_event = asyncio.Event()

            async def ws_main():
                self.get_logger().info(f'Starting WebSocket server on {self.host}:{self.websocket_port}')
                # Start server and keep a reference for shutdown
                self.websocket_server = await websockets.serve(self._websocket_handler, self.host, self.websocket_port)
                self.get_logger().info(f'WebSocket server started on {self.host}:{self.websocket_port}')
                # Wait until stop is signaled
                await self._ws_stop_event.wait()
                # Begin graceful shutdown
                self.get_logger().info('Stopping WebSocket server...')
                try:
                    # Close all clients first
                    await self._close_all_ws_clients()
                except Exception as e:
                    self.get_logger().warn(f'Error while closing WS clients: {e}')
                try:
                    if self.websocket_server is not None:
                        self.websocket_server.close()
                        await self.websocket_server.wait_closed()
                finally:
                    self.websocket_server = None

            # Start the server task and run the loop forever
            loop.create_task(ws_main())
            loop.run_forever()
        except Exception as e:
            self.get_logger().error(f'WebSocket server error: {e}')
    
    async def _websocket_handler(self, websocket: WebSocketServerProtocol, path: Optional[str] = None):
        """Handle WebSocket connections."""
        self.connected_clients.add(websocket)
        self.get_logger().info(f'Client connected: {websocket.remote_address}')
        # Send initial status message so UI can show connected state and topic
        try:
            status_msg = {
                'type': 'status',
                'cmd_vel_topic': self.cmd_vel_topic,
                'publisher_matched_count': self.cmd_vel_publisher.get_subscription_count() if hasattr(self.cmd_vel_publisher, 'get_subscription_count') else 0,
                'voice_topic': self.voice_topic,
                'voice_subscriber_count': self.voice_publisher.get_subscription_count() if hasattr(self.voice_publisher, 'get_subscription_count') else 0,
                'imu_topic': self.imu_topic,
            }
            await websocket.send(json.dumps(status_msg))
        except Exception as e:
            self.get_logger().warn(f'Failed to send initial status: {e}')
        
        try:
            async for message in websocket:
                await self._handle_websocket_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            self.get_logger().warn(f'WebSocket error: {e}')
        finally:
            self.connected_clients.discard(websocket)
            self.get_logger().info(f'Client disconnected: {websocket.remote_address}')
    
    async def _handle_websocket_message(self, websocket: WebSocketServerProtocol, message: str):
        """Handle incoming WebSocket messages."""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'cmd_vel':
                # Handle joystick command
                linear_x = float(data.get('linear', {}).get('x', 0.0))
                linear_y = float(data.get('linear', {}).get('y', 0.0))
                linear_z = float(data.get('linear', {}).get('z', 0.0))
                angular_x = float(data.get('angular', {}).get('x', 0.0))
                angular_y = float(data.get('angular', {}).get('y', 0.0))
                angular_z = float(data.get('angular', {}).get('z', 0.0))
                
                # Create and publish Twist message
                twist_msg = Twist()
                twist_msg.linear.x = linear_x
                twist_msg.linear.y = linear_y
                twist_msg.linear.z = linear_z
                twist_msg.angular.x = angular_x
                twist_msg.angular.y = angular_y
                twist_msg.angular.z = angular_z
                
                self.cmd_vel_publisher.publish(twist_msg)
                
                # Send acknowledgment
                response = {
                    'type': 'ack',
                    'cmd_vel': {
                        'linear': {'x': linear_x, 'y': linear_y, 'z': linear_z},
                        'angular': {'x': angular_x, 'y': angular_y, 'z': angular_z}
                    }
                }
                await websocket.send(json.dumps(response))
                
            elif msg_type == 'ping':
                # Handle ping with detailed status
                try:
                    sub_count = self.cmd_vel_publisher.get_subscription_count() if hasattr(self.cmd_vel_publisher, 'get_subscription_count') else 0
                except Exception:
                    sub_count = 0
                response = {
                    'type': 'pong',
                    'cmd_vel_topic': self.cmd_vel_topic,
                    'publisher_matched_count': sub_count,
                    'voice_topic': self.voice_topic,
                    'voice_subscriber_count': self.voice_publisher.get_subscription_count() if hasattr(self.voice_publisher, 'get_subscription_count') else 0,
                }
                await websocket.send(json.dumps(response))
            elif msg_type == 'voice':
                # Publish text to voice topic
                text = data.get('text')
                if isinstance(text, str) and text.strip():
                    try:
                        msg = String()
                        msg.data = text.strip()
                        self.voice_publisher.publish(msg)
                    except Exception as e:
                        self.get_logger().warn(f'Failed to publish voice text: {e}')
                    # Send acknowledgment
                    ack = {
                        'type': 'ack',
                        'voice': {
                            'text': text.strip()
                        }
                    }
                    await websocket.send(json.dumps(ack))
                
        except Exception as e:
            self.get_logger().warn(f'Error handling WebSocket message: {e}')
    
    async def _close_all_ws_clients(self):
        """Close all connected websocket clients (must be called in WS loop)."""
        # Make a copy to avoid modification during iteration
        clients = list(self.connected_clients)
        for client in clients:
            try:
                await client.close()
            except Exception:
                pass
        # Give clients a moment to close
        await asyncio.sleep(0)
    
    def destroy_node(self):
        """Clean shutdown."""
        # Stop WebSocket server and close clients from its own loop
        if self._ws_loop is not None:
            try:
                if self._ws_stop_event is not None:
                    # Signal stop on the WS loop thread
                    self._ws_loop.call_soon_threadsafe(self._ws_stop_event.set)
                # Also request the loop to stop after tasks complete
                self._ws_loop.call_soon_threadsafe(self._ws_loop.stop)
            except Exception:
                pass
            # Join the websocket thread to ensure cleanup
            if self.websocket_thread and self.websocket_thread.is_alive():
                try:
                    self.websocket_thread.join(timeout=2.0)
                except Exception:
                    pass
            self._ws_loop = None
            self._ws_stop_event = None
            self.websocket_server = None
        
        # Stop HTTP server gracefully
        if self.web_server is not None:
            try:
                # Trigger server shutdown from this thread
                self.web_server.shutdown()
            except Exception:
                pass
            self.web_server = None
        
        return super().destroy_node()


def main(args=None):
    """Main entry point."""
    if websockets is None:
        print('Error: websockets library required. Install with: pip install websockets')
        return 1
    
    rclpy.init(args=args)
    node = PilotNode()
    
    try:
        executor = MultiThreadedExecutor(num_threads=4)
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