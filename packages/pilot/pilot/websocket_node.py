#!/usr/bin/env python3
"""Separate WebSocket node for Pilot.

Runs only the WebSocket server and publishes to ROS topics.
This isolates asyncio from the HTTP server to avoid cross-thread/event-loop issues.
"""

import asyncio
import subprocess
import shutil
import os
from pathlib import Path
import threading
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int16, UInt16, UInt32, Empty
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from create_msgs.msg import ChargingState, Mode, Bumper, Cliff
from sensor_msgs.msg import Imu
from math import atan2
from psyched_msgs.msg import Message as ConvMessage

# Optional AudioInfo import for microphone metadata
try:
    from audio_common_msgs.msg import AudioInfo  # type: ignore
except Exception:  # pragma: no cover
    AudioInfo = None  # type: ignore

try:
    import websockets
except ImportError:
    websockets = None


class PilotWebSocketNode(Node):
    def __init__(self):
        super().__init__('pilot_websocket_node')

        self.declare_parameter('websocket_port', 8081)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('voice_topic', '/voice')
        self.declare_parameter('voice_pause_topic', '/voice/interrupt')
        self.declare_parameter('voice_resume_topic', '/voice/resume')
        self.declare_parameter('voice_clear_topic', '/voice/clear')
        self.declare_parameter('voice_volume_topic', '/voice/volume')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('host_health_topic', 'auto')
        self.declare_parameter('imu_topic', '/imu/mpu6050')
        self.declare_parameter('gps_fix_topic', '/gps/fix')
        self.declare_parameter('conversation_topic', '/conversation')

        # Resolve parameters safely
        def _param(name, default):
            try:
                val = self.get_parameter(name).value
                return default if val is None else val
            except Exception:
                return default
        self.websocket_port = int(_param('websocket_port', 8081))
        self.cmd_vel_topic = str(_param('cmd_vel_topic', '/cmd_vel'))
        self.voice_topic = str(_param('voice_topic', '/voice'))
        self.voice_pause_topic = str(_param('voice_pause_topic', '/voice/interrupt'))
        self.voice_resume_topic = str(_param('voice_resume_topic', '/voice/resume'))
        self.voice_clear_topic = str(_param('voice_clear_topic', '/voice/clear'))
        self.voice_volume_topic = str(_param('voice_volume_topic', '/voice/volume'))
        self.host = str(_param('host', '0.0.0.0'))
        self.host_health_topic = str(_param('host_health_topic', 'auto'))
        self.imu_topic = str(_param('imu_topic', '/imu/mpu6050'))
        self.gps_fix_topic = str(_param('gps_fix_topic', '/gps/fix'))
        self.conversation_topic = str(_param('conversation_topic', '/conversation'))

        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.voice_publisher = self.create_publisher(String, self.voice_topic, 10)
        self.voice_pause_pub = self.create_publisher(Empty, self.voice_pause_topic, 10)
        self.voice_resume_pub = self.create_publisher(Empty, self.voice_resume_topic, 10)
        self.voice_clear_pub = self.create_publisher(Empty, self.voice_clear_topic, 10)
        self.voice_volume_pub = self.create_publisher(Float32, self.voice_volume_topic, 10)

        # IMU subscription/cache
        self._imu_last = None
        try:
            self.create_subscription(Imu, self.imu_topic, self._on_imu, 10)
        except Exception as e:
            self.get_logger().warn(f'Failed to subscribe to IMU: {e}')

        # GPS fix subscription/cache
        self._gps_fix = None  # dict with lat, lon, alt, status
        try:
            self.create_subscription(NavSatFix, self.gps_fix_topic, self._on_gps_fix, 10)
        except Exception as e:
            self.get_logger().warn(f'Failed to subscribe to GPS fix: {e}')

        # Conversation subscription
        try:
            self.create_subscription(ConvMessage, self.conversation_topic, self._on_conversation, 10)
        except Exception as e:
            self.get_logger().warn(f'Failed to subscribe to conversation: {e}')

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
        # Determine host health topic
        htopic = self.host_health_topic
        self._host_short = None
        if not htopic or htopic.lower() == 'auto':
            try:
                import socket
                short = socket.gethostname().split('.')[0]
            except Exception:
                import os as _os
                short = _os.uname().nodename.split('.')[0] if hasattr(_os, 'uname') else 'host'
            htopic = f'hosts/{short}/health'
            self._host_short = short
        else:
            # best-effort host short from env/hostname
            try:
                import socket
                self._host_short = socket.gethostname().split('.')[0]
            except Exception:
                self._host_short = 'host'
        self.create_subscription(String, htopic, self._on_host_health, 10)

        # Audio status (voice speaking and VAD speech duration)
        self._audio = {
            'autophony_ms': 0,
            'speech_ms': 0,
            'mic': {'sample_rate': None, 'channels': None},
        }
        try:
            self.create_subscription(UInt32, '/audio/autophony_duration', self._on_autophony, 10)
        except Exception:
            pass
        try:
            self.create_subscription(UInt32, '/audio/speech_duration', self._on_speech_dur, 10)
        except Exception:
            pass
        if AudioInfo is not None:
            try:
                self.create_subscription(AudioInfo, '/audio/pcm/info', self._on_audio_info, 10)
            except Exception:
                pass

        # Systemd services discovery
        self._systemd_units = self._discover_systemd_units()

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

    async def _ws_handler(self, websocket, path: Optional[str] = None):
        self.connected_clients.add(websocket)
        self.get_logger().info(f'Client connected: {websocket.remote_address}')
        try:
            status_msg = {
                'type': 'status',
                'cmd_vel_topic': self.cmd_vel_topic,
                'publisher_matched_count': self.cmd_vel_publisher.get_subscription_count() if hasattr(self.cmd_vel_publisher, 'get_subscription_count') else 0,
                'voice_topic': self.voice_topic,
                'voice_subscriber_count': self.voice_publisher.get_subscription_count() if hasattr(self.voice_publisher, 'get_subscription_count') else 0,
                'voice_pause_topic': self.voice_pause_topic,
                'voice_resume_topic': self.voice_resume_topic,
                'voice_clear_topic': self.voice_clear_topic,
                'voice_volume_topic': self.voice_volume_topic,
                'imu_topic': self.imu_topic,
                'conversation_topic': self.conversation_topic,
            }
            status_msg['audio'] = self._audio
            # include latest GPS fix if available
            if self._gps_fix is not None:
                status_msg['gps_fix'] = self._gps_fix
            # include systemd services snapshot
            try:
                services = self._list_systemd_status()
                if services:
                    status_msg['systemd_services'] = services
            except Exception as e:
                self.get_logger().warn(f'Failed to get systemd status: {e}')
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

    async def _handle_message(self, websocket, message: str):
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
                    'voice_subscriber_count': self.voice_publisher.get_subscription_count() if hasattr(self.voice_publisher, 'get_subscription_count') else 0,
                    'voice_pause_topic': self.voice_pause_topic,
                    'voice_resume_topic': self.voice_resume_topic,
                    'voice_clear_topic': self.voice_clear_topic,
                    'voice_volume_topic': self.voice_volume_topic,
                    'imu_topic': self.imu_topic,
                    'conversation_topic': self.conversation_topic,
                }
                if self._gps_fix is not None:
                    pong['gps_fix'] = self._gps_fix
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
                # include audio status snapshot
                pong['audio'] = self._audio
                # include services snapshot on pong to keep UI fresh (lightweight)
                try:
                    services = self._list_systemd_status()
                    if services:
                        pong['systemd_services'] = services
                except Exception:
                    pass
                await websocket.send(json.dumps(pong))
            elif t == 'voice':
                text = data.get('text')
                if isinstance(text, str) and text.strip():
                    msg = String()
                    msg.data = text.strip()
                    self.voice_publisher.publish(msg)
                    await websocket.send(json.dumps({'type': 'ack', 'voice': {'text': msg.data}}))
            elif t == 'voice_control':
                # { type: 'voice_control', action: 'pause'|'resume'|'clear' }
                action = (data.get('action') or '').lower()
                if action == 'pause':
                    self.voice_pause_pub.publish(Empty())
                    await websocket.send(json.dumps({'type': 'ack', 'voice_control': 'pause'}))
                elif action == 'resume':
                    self.voice_resume_pub.publish(Empty())
                    await websocket.send(json.dumps({'type': 'ack', 'voice_control': 'resume'}))
                elif action == 'clear':
                    self.voice_clear_pub.publish(Empty())
                    await websocket.send(json.dumps({'type': 'ack', 'voice_control': 'clear'}))
                else:
                    await websocket.send(json.dumps({'type': 'error', 'error': f'unknown voice action: {action}'}))
            elif t == 'voice_volume':
                # { type: 'voice_volume', value: <0.0..1.5> }
                try:
                    val = float(data.get('value'))
                    # Clamp to a reasonable range
                    if not (val == val) or val < 0:
                        raise ValueError('invalid')
                    if val > 2.0:
                        val = 2.0
                    self.voice_volume_pub.publish(Float32(data=val))
                    # Also attempt to set system ALSA mixer volume when available.
                    # Mapping: values <= 1.0 map to 0-100%; values > 1.0 keep ALSA at 100% (extra gain handled by voice node)
                    try:
                        pct = int(max(0.0, min(1.0, val)) * 100.0)
                        ok = self._set_alsa_volume(pct)
                        if not ok:
                            self.get_logger().info('ALSA volume unchanged (amixer missing or control not found)')
                    except Exception as e:
                        self.get_logger().warn(f'Failed to adjust ALSA volume: {e}')
                    await websocket.send(json.dumps({'type': 'ack', 'voice_volume': val}))
                except Exception:
                    await websocket.send(json.dumps({'type': 'error', 'error': 'invalid volume value'}))
            elif t == 'systemd':
                action = (data.get('action') or 'list').lower()
                if action == 'list':
                    await self._send_systemd_list(websocket)
                elif action in ('start', 'stop', 'enable', 'disable', 'restart', 'status'):
                    unit = data.get('unit')
                    if not unit or not isinstance(unit, str):
                        await websocket.send(json.dumps({'type': 'systemd', 'error': 'missing unit'}))
                    elif unit not in self._systemd_units:
                        await websocket.send(json.dumps({'type': 'systemd', 'error': f'unknown unit: {unit}'}))
                    else:
                        result = self._systemd_perform(action, unit)
                        # always follow up with fresh status for this unit
                        status = self._query_systemd_unit(unit)
                        payload = {'type': 'systemd', 'unit': unit, 'status': status}
                        if result.get('error'):
                            payload['error'] = result['error']
                        await websocket.send(json.dumps(payload))
                elif action == 'detail':
                    unit = data.get('unit')
                    lines = data.get('lines')
                    try:
                        n = int(lines) if lines is not None else 200
                        if n <= 0 or n > 1000:
                            n = 200
                    except Exception:
                        n = 200
                    if not unit or not isinstance(unit, str):
                        await websocket.send(json.dumps({'type': 'systemd', 'error': 'missing unit'}))
                    elif unit not in self._systemd_units:
                        await websocket.send(json.dumps({'type': 'systemd', 'error': f'unknown unit: {unit}'}))
                    else:
                        detail = self._systemd_detail(unit, n)
                        await websocket.send(json.dumps({'type': 'systemd', 'unit': unit, 'detail': detail}))
        except Exception as e:
            self.get_logger().warn(f'Error handling message: {e}')
        # no return payload expected from handler
        return None

    # IMU forwarding
    def _on_imu(self, msg: Imu):
        try:
            ax = float(msg.linear_acceleration.x)
            ay = float(msg.linear_acceleration.y)
            az = float(msg.linear_acceleration.z)
            gx = float(msg.angular_velocity.x)
            gy = float(msg.angular_velocity.y)
            gz = float(msg.angular_velocity.z)
            qx = float(msg.orientation.x)
            qy = float(msg.orientation.y)
            qz = float(msg.orientation.z)
            qw = float(msg.orientation.w)
            yaw = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        except Exception:
            ax=ay=az=gx=gy=gz=yaw=0.0
        self._imu_last = {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz, 'yaw': yaw}
        # push to clients immediately (throttle implicitly by ROS rate)
        if not self.connected_clients or self._loop is None:
            return
        data = json.dumps({'type': 'imu', **self._imu_last})
        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(data)
                except Exception:
                    pass
        asyncio.run_coroutine_threadsafe(_send_all(), self._loop)

    # Conversation forwarding
    def _on_conversation(self, msg: ConvMessage):
        try:
            role = str(getattr(msg, 'role', '') or '')
            content = str(getattr(msg, 'content', '') or '')
        except Exception:
            return
        if not content:
            return
        payload = {'type': 'conversation', 'role': role, 'content': content}
        if not self.connected_clients or self._loop is None:
            return
        data = json.dumps(payload)
        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(data)
                except Exception:
                    pass
        asyncio.run_coroutine_threadsafe(_send_all(), self._loop)

    # GPS forwarding
    def _on_gps_fix(self, msg: NavSatFix):
        try:
            lat = float(msg.latitude)
            lon = float(msg.longitude)
            alt = float(msg.altitude)
            status = int(getattr(msg.status, 'status', 0)) if getattr(msg, 'status', None) is not None else 0
        except Exception:
            return
        self._gps_fix = {'lat': lat, 'lon': lon, 'alt': alt, 'status': status}
        if not self.connected_clients or self._loop is None:
            return
        data = json.dumps({'type': 'gps_fix', **self._gps_fix})
        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(data)
                except Exception:
                    pass
        asyncio.run_coroutine_threadsafe(_send_all(), self._loop)

    # Audio callbacks/forwarding
    def _broadcast_audio(self):
        if not self.connected_clients or self._loop is None:
            return
        data = json.dumps({'type': 'audio_status', **self._audio})
        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(data)
                except Exception:
                    pass
        asyncio.run_coroutine_threadsafe(_send_all(), self._loop)

    def _on_autophony(self, msg: UInt32):
        try:
            self._audio['autophony_ms'] = int(getattr(msg, 'data', 0))
        except Exception:
            self._audio['autophony_ms'] = 0
        self._broadcast_audio()

    def _on_speech_dur(self, msg: UInt32):
        try:
            self._audio['speech_ms'] = int(getattr(msg, 'data', 0))
        except Exception:
            self._audio['speech_ms'] = 0
        self._broadcast_audio()

    def _on_audio_info(self, msg):  # AudioInfo when available
        try:
            sr = int(getattr(msg, 'sample_rate', 0))
            ch = int(getattr(msg, 'channels', 0))
            self._audio['mic'] = {'sample_rate': sr or None, 'channels': ch or None}
        except Exception:
            self._audio['mic'] = {'sample_rate': None, 'channels': None}
        # Send a dedicated message for info to avoid overwriting durations unnecessarily
        if not self.connected_clients or self._loop is None:
            return
        data = json.dumps({'type': 'audio_info', **self._audio.get('mic', {})})
        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(data)
                except Exception:
                    pass
        asyncio.run_coroutine_threadsafe(_send_all(), self._loop)

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

    # -----------------------------
    # Systemd helpers
    # -----------------------------
    def _discover_systemd_units(self):
        try:
            # services present under hosts/<host_short>/systemd
            host_short = self._host_short or 'host'
            repo_root = Path(__file__).resolve().parents[3]  # .../packages/pilot/pilot -> repo
            services_dir = repo_root / 'hosts' / host_short / 'systemd'
            if not services_dir.exists():
                return []
            units = []
            for p in services_dir.glob('psyched-*.service'):
                units.append(p.name)
            return sorted(units)
        except Exception as e:
            self.get_logger().warn(f'Failed to discover systemd units: {e}')
            return []

    def _run_cmd(self, args, require_root: bool = False):
        env = os.environ.copy()
        cmd = list(args)
        # Non-interactive sudo if requested
        if require_root:
            cmd = ['sudo', '-n'] + cmd
        try:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            return {'code': res.returncode, 'out': res.stdout.strip(), 'err': res.stderr.strip()}
        except Exception as e:
            return {'code': -1, 'out': '', 'err': str(e)}

    def _query_systemd_unit(self, unit: str):
        # Query active state
        active = self._run_cmd(['systemctl', 'is-active', unit])
        enabled = self._run_cmd(['systemctl', 'is-enabled', unit])
        desc = self._run_cmd(['systemctl', 'show', unit, '--property=Description', '--value'])
        return {
            'name': unit,
            'active': active['out'] if active['code'] == 0 else 'unknown',
            'enabled': enabled['out'] if enabled['code'] == 0 else 'unknown',
            'description': desc['out'] if desc['code'] == 0 else ''
        }

    def _list_systemd_status(self):
        return [self._query_systemd_unit(u) for u in self._systemd_units]

    async def _send_systemd_list(self, websocket):
        try:
            services = self._list_systemd_status()
        except Exception:
            services = []
        await websocket.send(json.dumps({'type': 'systemd', 'services': services}))

    def _systemd_perform(self, action: str, unit: str):
        # Map actions to systemctl verbs
        verb = {
            'start': 'start',
            'stop': 'stop',
            'enable': 'enable',
            'disable': 'disable',
            'restart': 'restart',
            'status': 'status',
        }.get(action)
        if not verb:
            return {'error': f'unsupported action: {action}'}
        # Try without sudo first (status/start may work if user has rights), else sudo -n
        r = self._run_cmd(['systemctl', verb, unit])
        if r['code'] != 0:
            # Try non-interactive sudo
            r2 = self._run_cmd(['systemctl', verb, unit], require_root=True)
            if r2['code'] != 0:
                # include error but do not block UI; suggest NOPASSWD configuration
                return {'error': r2['err'] or r2['out'] or f'systemctl {verb} failed with code {r2["code"]}'}
        return {}

    def _systemd_detail(self, unit: str, lines: int = 200):
        # Full status text
        st = self._run_cmd(['systemctl', '--no-pager', '--full', 'status', unit])
        if st['code'] != 0:
            st2 = self._run_cmd(['systemctl', '--no-pager', '--full', 'status', unit], require_root=True)
            st_txt = st2['out'] or st2['err']
        else:
            st_txt = st['out'] or st['err']
        # Journal recent logs
        jl = self._run_cmd(['journalctl', '-u', unit, '-n', str(lines), '--no-pager', '--output=short-iso'])
        if jl['code'] != 0:
            jl2 = self._run_cmd(['journalctl', '-u', unit, '-n', str(lines), '--no-pager', '--output=short-iso'], require_root=True)
            jl_txt = jl2['out'] or jl2['err']
        else:
            jl_txt = jl['out'] or jl['err']
        return {
            'status': st_txt,
            'journal': jl_txt,
            'lines': lines
        }

    # -----------------------------
    # Audio helpers (system ALSA volume)
    # -----------------------------
    def _set_alsa_volume(self, level_percent: int) -> bool:
        """Set system output volume via ALSA amixer.

        Tries common mixer controls in order until one succeeds.
        Returns True if a control was adjusted successfully.
        """
        try:
            level = int(max(0, min(100, level_percent)))
        except Exception:
            level = 100

        amixer = shutil.which('amixer')
        if not amixer:
            return False

        controls = ['Master', 'Speaker', 'PCM', 'Headphone', 'Digital', 'Playback']
        # Try without device, then explicit default, then pulse bridge
        device_args = [[], ['-D', 'default'], ['-D', 'pulse']]
        for ctl in controls:
            for dargs in device_args:
                res = self._run_cmd([amixer, *dargs, '-q', 'sset', ctl, f'{level}%'])
                if res.get('code', 1) == 0:
                    dev = ' '.join(dargs) if dargs else '(default)'
                    self.get_logger().info(f"ALSA volume set: {ctl} {dev} -> {level}%")
                    return True
        # If none succeeded, log available controls to aid debugging
        try:
            scontrols = self._run_cmd([amixer, 'scontrols'])
            if scontrols.get('code', 1) == 0 and scontrols.get('out'):
                self.get_logger().info(f"amixer available controls: {scontrols['out']}")
        except Exception:
            pass
        return False


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
