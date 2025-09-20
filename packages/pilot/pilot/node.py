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
        
        # Parameters
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('websocket_port', 8081)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('host', '0.0.0.0')
        
        self.web_port = self.get_parameter('web_port').get_parameter_value().integer_value
        self.websocket_port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        
        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Web and WebSocket servers
        self.web_server = None
        self.websocket_server = None
        self.web_thread = None
        self.websocket_thread = None
        
        # Connected WebSocket clients
        self.connected_clients = set()
        
        # Static files path
        self.static_path = self._find_static_path()
        
        # Start servers
        self.start_servers()
        
        self.get_logger().info(f'Pilot node started:')
        self.get_logger().info(f'  Web interface: http://{self.host}:{self.web_port}')
        self.get_logger().info(f'  WebSocket: ws://{self.host}:{self.websocket_port}')
        self.get_logger().info(f'  Publishing to: {self.cmd_vel_topic}')
    
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
        self.web_thread = threading.Thread(target=self._run_web_server, daemon=True)
        self.web_thread.start()
        
        # Start WebSocket server
        if websockets:
            self.websocket_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
            self.websocket_thread.start()
        else:
            self.get_logger().warn('websockets library not available. Install with: pip install websockets')
    
    def _run_web_server(self):
        """Run the HTTP web server."""
        try:
            handler = self._create_http_handler()
            with socketserver.TCPServer((self.host, self.web_port), handler) as httpd:
                self.get_logger().info(f'Web server started on {self.host}:{self.web_port}')
                httpd.serve_forever()
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
            
            start_server = websockets.serve(
                self._websocket_handler,
                self.host,
                self.websocket_port
            )
            
            self.get_logger().info(f'WebSocket server started on {self.host}:{self.websocket_port}')
            loop.run_until_complete(start_server)
            loop.run_forever()
        except Exception as e:
            self.get_logger().error(f'WebSocket server error: {e}')
    
    async def _websocket_handler(self, websocket: WebSocketServerProtocol, path: str):
        """Handle WebSocket connections."""
        self.connected_clients.add(websocket)
        self.get_logger().info(f'Client connected: {websocket.remote_address}')
        
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
                # Handle ping
                response = {'type': 'pong'}
                await websocket.send(json.dumps(response))
                
        except Exception as e:
            self.get_logger().warn(f'Error handling WebSocket message: {e}')
    
    def destroy_node(self):
        """Clean shutdown."""
        # Close WebSocket connections
        if self.connected_clients:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            async def close_connections():
                for client in list(self.connected_clients):
                    await client.close()
            
            loop.run_until_complete(close_connections())
            loop.close()
        
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