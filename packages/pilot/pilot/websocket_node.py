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
from std_msgs.msg import String, Float32, Int16, UInt16, Empty
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from create_msgs.msg import ChargingState, Mode, Bumper, Cliff

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

        # Battery topics (from create_driver)
        # Note: topics are relative to create_driver node namespace
        self._battery = {
            'voltage': None,
            'current': None,
            'charge': None,
            'capacity': None,
            'charge_ratio': None,
            'temperature': None,
            'charging_state': None,
        }
        self._battery_lock = threading.Lock()
        # Subscriptions
        self.create_subscription(Float32, 'battery/voltage', self._on_batt_voltage, 10)
        self.create_subscription(Float32, 'battery/current', self._on_batt_current, 10)
        self.create_subscription(Float32, 'battery/charge', self._on_batt_charge, 10)
        self.create_subscription(Float32, 'battery/capacity', self._on_batt_capacity, 10)
        self.create_subscription(Float32, 'battery/charge_ratio', self._on_batt_ratio, 10)
        self.create_subscription(Int16, 'battery/temperature', self._on_batt_temp, 10)
        self.create_subscription(ChargingState, 'battery/charging_state', self._on_batt_state, 10)

        # Robot status snapshot
        self._robot = {
            'mode': None,
            'speed': None,
            'bumper': None,
            'cliff': None,
            'wheel_drop': None,
            'ir_omni': None,
            'diag_level': None,
            'diag_counts': None,
        }
        self._robot_lock = threading.Lock()
        # Robot-related subscriptions
        self.create_subscription(Mode, 'mode', self._on_mode, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Bumper, 'bumper', self._on_bumper, 10)
        self.create_subscription(Cliff, 'cliff', self._on_cliff, 10)
        self.create_subscription(Empty, 'wheeldrop', self._on_wheeldrop, 10)
        self.create_subscription(UInt16, 'ir_omni', self._on_ir_omni, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self._on_diag, 10)
        # Periodic broadcast of robot snapshot
        self._robot_timer = self.create_timer(1.0, self._broadcast_robot_status)

        # Host health cache and subscription
        self._host_health = None
        self._host_lock = threading.Lock()
        self.create_subscription(String, 'host/health', self._on_host_health, 10)

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
            # include latest battery snapshot if available
            with self._battery_lock:
                if any(v is not None for v in self._battery.values()):
                    status_msg['battery'] = self._format_battery()
                with self._robot_lock:
                    rs = self._format_robot_status()
                    if rs:
                        status_msg['robot_status'] = rs
                with self._host_lock:
                    if self._host_health is not None:
                        status_msg['host_health'] = self._host_health
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
                pong = {
                    'type': 'pong',
                    'cmd_vel_topic': self.cmd_vel_topic,
                    'publisher_matched_count': sub_count,
                    'voice_topic': self.voice_topic,
                    'voice_subscriber_count': self.voice_publisher.get_subscription_count() if hasattr(self.voice_publisher, 'get_subscription_count') else 0
                }
                with self._battery_lock:
                    if any(v is not None for v in self._battery.values()):
                        pong['battery'] = self._format_battery()
                    with self._robot_lock:
                        rs = self._format_robot_status()
                        if rs:
                            pong['robot_status'] = rs
                    with self._host_lock:
                        if self._host_health is not None:
                            pong['host_health'] = self._host_health
                await websocket.send(json.dumps(pong))
            elif t == 'voice':
                text = data.get('text')
                if isinstance(text, str) and text.strip():
                    msg = String()
                    msg.data = text.strip()
                    self.voice_publisher.publish(msg)
                    await websocket.send(json.dumps({'type': 'ack', 'voice': {'text': msg.data}}))
        except Exception as e:
            self.get_logger().warn(f'Error handling message: {e}')
        # no return payload expected from handler
        return None

    # Battery callbacks
    def _broadcast_battery(self):
        if not self.connected_clients:
            return
        msg = {'type': 'battery', **self._format_battery()}
        data = json.dumps(msg)
        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(data)
                except Exception:
                    pass
        # schedule in loop thread-safe
        if self._loop:
            asyncio.run_coroutine_threadsafe(_send_all(), self._loop)

    def _format_battery(self):
        # Compute percentage if possible
        percent = None
        try:
            if self._battery['charge_ratio'] is not None:
                percent = float(self._battery['charge_ratio']) * 100.0
            elif self._battery['charge'] is not None and self._battery['capacity'] is not None and self._battery['capacity']:
                percent = (float(self._battery['charge']) / float(self._battery['capacity'])) * 100.0
        except Exception:
            percent = None
        return {
            'voltage': self._battery['voltage'],
            'current': self._battery['current'],
            'charge': self._battery['charge'],
            'capacity': self._battery['capacity'],
            'charge_ratio': self._battery['charge_ratio'],
            'temperature': self._battery['temperature'],
            'charging_state': getattr(self._battery['charging_state'], 'state', self._battery['charging_state']) if self._battery['charging_state'] is not None else None,
            'percent': percent
        }

    def _on_batt_voltage(self, msg: Float32):
        with self._battery_lock:
            self._battery['voltage'] = float(msg.data)
        self._broadcast_battery()

    def _on_batt_current(self, msg: Float32):
        with self._battery_lock:
            self._battery['current'] = float(msg.data)
        self._broadcast_battery()

    def _on_batt_charge(self, msg: Float32):
        with self._battery_lock:
            self._battery['charge'] = float(msg.data)
        self._broadcast_battery()

    def _on_batt_capacity(self, msg: Float32):
        with self._battery_lock:
            self._battery['capacity'] = float(msg.data)
        self._broadcast_battery()

    def _on_batt_ratio(self, msg: Float32):
        with self._battery_lock:
            self._battery['charge_ratio'] = float(msg.data)
        self._broadcast_battery()

    def _on_batt_temp(self, msg: Int16):
        with self._battery_lock:
            self._battery['temperature'] = int(msg.data)
        self._broadcast_battery()

    def _on_batt_state(self, msg: ChargingState):
        with self._battery_lock:
            self._battery['charging_state'] = msg
        self._broadcast_battery()

    # Robot callbacks and formatting
    def _format_robot_status(self):
        d = self._robot
        if all(v is None for v in d.values()):
            return None
        return {
            'mode': d['mode'],
            'speed': d['speed'],
            'bumper': d['bumper'],
            'cliff': d['cliff'],
            'wheel_drop': d['wheel_drop'],
            'ir_omni': d['ir_omni'],
            'diag_level': d['diag_level'],
            'diag_counts': d['diag_counts'],
        }

    def _on_mode(self, msg: Mode):
        with self._robot_lock:
            try:
                self._robot['mode'] = int(msg.mode)
            except Exception:
                self._robot['mode'] = None

    def _on_odom(self, msg: Odometry):
        try:
            vx = float(msg.twist.twist.linear.x)
            vy = float(msg.twist.twist.linear.y)
            speed = (vx * vx + vy * vy) ** 0.5
        except Exception:
            speed = None
        with self._robot_lock:
            self._robot['speed'] = speed

    def _on_bumper(self, msg: Bumper):
        any_pressed = bool(getattr(msg, 'is_left_pressed', False) or getattr(msg, 'is_right_pressed', False))
        with self._robot_lock:
            self._robot['bumper'] = any_pressed

    def _on_cliff(self, msg: Cliff):
        any_cliff = bool(
            getattr(msg, 'is_cliff_left', False) or getattr(msg, 'is_cliff_front_left', False) or getattr(msg, 'is_cliff_right', False) or getattr(msg, 'is_cliff_front_right', False)
        )
        with self._robot_lock:
            self._robot['cliff'] = any_cliff

    def _on_wheeldrop(self, _msg: Empty):
        with self._robot_lock:
            self._robot['wheel_drop'] = True

    def _on_ir_omni(self, msg: UInt16):
        with self._robot_lock:
            try:
                self._robot['ir_omni'] = int(msg.data)
            except Exception:
                self._robot['ir_omni'] = None

    def _on_diag(self, msg: DiagnosticArray):
        counts = {0: 0, 1: 0, 2: 0}
        max_lvl = 0
        for st in getattr(msg, 'status', []):
            try:
                lvl = int(getattr(st, 'level', 0))
            except Exception:
                lvl = 0
            counts[lvl] = counts.get(lvl, 0) + 1
            if lvl > max_lvl:
                max_lvl = lvl
        with self._robot_lock:
            self._robot['diag_level'] = max_lvl
            self._robot['diag_counts'] = counts

    def _broadcast_robot_status(self):
        if not self.connected_clients:
            return
        with self._robot_lock:
            payload = self._format_robot_status()
        if not payload:
            return
        data = json.dumps({'type': 'robot_status', **payload})
        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(data)
                except Exception:
                    pass
        if self._loop:
            asyncio.run_coroutine_threadsafe(_send_all(), self._loop)

    # Host health
    def _on_host_health(self, msg: String):
        try:
            obj = json.loads(msg.data)
        except Exception:
            obj = {'raw': msg.data}
        with self._host_lock:
            self._host_health = obj
        if not self.connected_clients:
            return
        data = json.dumps({'type': 'host_health', **obj})
        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(data)
                except Exception:
                    pass
        if self._loop:
            asyncio.run_coroutine_threadsafe(_send_all(), self._loop)
        # Robot callbacks
        def _on_mode(self, msg: Mode):
            with self._robot_lock:
                try:
                    self._robot['mode'] = int(msg.mode)
                except Exception:
                    self._robot['mode'] = None

        def _on_odom(self, msg: Odometry):
            try:
                vx = float(msg.twist.twist.linear.x)
                vy = float(msg.twist.twist.linear.y)
                speed = (vx * vx + vy * vy) ** 0.5
            except Exception:
                speed = None
            with self._robot_lock:
                self._robot['speed'] = speed

        def _on_bumper(self, msg: Bumper):
            any_pressed = bool(getattr(msg, 'is_left_pressed', False) or getattr(msg, 'is_right_pressed', False))
            with self._robot_lock:
                self._robot['bumper'] = any_pressed

        def _on_cliff(self, msg: Cliff):
            any_cliff = bool(
                getattr(msg, 'is_cliff_left', False) or getattr(msg, 'is_cliff_front_left', False) or getattr(msg, 'is_cliff_right', False) or getattr(msg, 'is_cliff_front_right', False)
            )
            with self._robot_lock:
                self._robot['cliff'] = any_cliff

        def _on_wheeldrop(self, _msg: Empty):
            with self._robot_lock:
                self._robot['wheel_drop'] = True

        def _on_ir_omni(self, msg: UInt16):
            with self._robot_lock:
                try:
                    self._robot['ir_omni'] = int(msg.data)
                except Exception:
                    self._robot['ir_omni'] = None

        def _on_diag(self, msg: DiagnosticArray):
            counts = {0: 0, 1: 0, 2: 0}
            max_lvl = 0
            for st in getattr(msg, 'status', []):
                try:
                    lvl = int(getattr(st, 'level', 0))
                except Exception:
                    lvl = 0
                counts[lvl] = counts.get(lvl, 0) + 1
                if lvl > max_lvl:
                    max_lvl = lvl
            with self._robot_lock:
                self._robot['diag_level'] = max_lvl
                self._robot['diag_counts'] = counts

        def _broadcast_robot_status(self):
            if not self.connected_clients:
                return
            with self._robot_lock:
                payload = self._format_robot_status()
            if not payload:
                return
            data = json.dumps({'type': 'robot_status', **payload})
            async def _send_all():
                for client in list(self.connected_clients):
                    try:
                        await client.send(data)
                    except Exception:
                        pass
            if self._loop:
                asyncio.run_coroutine_threadsafe(_send_all(), self._loop)

        # Host health
        def _on_host_health(self, msg: String):
            try:
                obj = json.loads(msg.data)
            except Exception:
                obj = {'raw': msg.data}
            with self._host_lock:
                self._host_health = obj
            if not self.connected_clients:
                return
            data = json.dumps({'type': 'host_health', **obj})
            async def _send_all():
                for client in list(self.connected_clients):
                    try:
                        await client.send(data)
                    except Exception:
                        pass
            if self._loop:
                asyncio.run_coroutine_threadsafe(_send_all(), self._loop)

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
