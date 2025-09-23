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
from sensor_msgs.msg import Image
try:
    from nav_msgs.msg import OccupancyGrid
except Exception:
    OccupancyGrid = None
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import base64
import zlib

# Optional image tooling
try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover
    CvBridge = None

try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None

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
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('gps_fix_topic', '/gps/fix')
        self.declare_parameter('conversation_topic', '/conversation')
        self.declare_parameter('map_topic', '/map')

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
        self.imu_topic = str(_param('imu_topic', '/imu'))
        self.gps_fix_topic = str(_param('gps_fix_topic', '/gps/fix'))
        self.conversation_topic = str(_param('conversation_topic', '/conversation'))
        self.map_topic = str(_param('map_topic', '/map'))

        # Background recording/map save processes registry
        self._bg_recordings = {}  # name -> Popen
        self._bg_lock = threading.Lock()

        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.voice_publisher = self.create_publisher(String, self.voice_topic, 10)
        self.voice_pause_pub = self.create_publisher(Empty, self.voice_pause_topic, 10)
        self.voice_resume_pub = self.create_publisher(Empty, self.voice_resume_topic, 10)
        self.voice_clear_pub = self.create_publisher(Empty, self.voice_clear_topic, 10)
        self.voice_volume_pub = self.create_publisher(Float32, self.voice_volume_topic, 10)
        # Module control publisher (generic JSON payload for modules to consume)
        try:
            self.module_control_pub = self.create_publisher(String, 'pilot/module_control', 10)
        except Exception:
            self.module_control_pub = None

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

        # Module controls discovery
        self._module_unit_map = {}
        self._modules = self._discover_modules()

        # asyncio context in dedicated thread
        self._loop = None
        self._server = None
        self._stop_event = None
        self.connected_clients = set()
        # Systemd watch registry and task
        self._systemd_watch = {}
        self._systemd_watch_task = None

        # Start asyncio server thread
        self._thread = threading.Thread(target=self._run_ws_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f'WebSocket node starting on ws://{self.host}:{self.websocket_port}')

        # Image bridge (optional)
        self._bridge = CvBridge() if CvBridge is not None else None
        # Subscriptions for camera and depth (best-effort)
        try:
            if self._bridge is not None and cv2 is not None:
                # Use small depth and BEST_EFFORT reliability for high-rate image streams
                try:
                    qos_img = QoSProfile(depth=1)
                    qos_img.reliability = ReliabilityPolicy.BEST_EFFORT
                    qos_img.history = HistoryPolicy.KEEP_LAST
                    # color image
                    self.create_subscription(Image, '/image_raw', self._on_image, qos_img)
                    # depth image
                    self.create_subscription(Image, '/depth/image_raw', self._on_depth, qos_img)
                except Exception:
                    # Fallback to default integer depth if QoSProfile isn't accepted
                    self.create_subscription(Image, '/image_raw', self._on_image, 10)
                    self.create_subscription(Image, '/depth/image_raw', self._on_depth, 10)
            else:
                if CvBridge is None:
                    self.get_logger().info('cv_bridge not available; image streaming disabled')
                else:
                    self.get_logger().info('OpenCV not available; image streaming disabled')
        except Exception as e:
            self.get_logger().warn(f'Failed to create image subscriptions: {e}')

        # Map subscription (OccupancyGrid) — optional
        try:
            if OccupancyGrid is not None:
                # Maps are relatively low-rate; keep a small queue and best-effort reliability
                try:
                    qos_map = QoSProfile(depth=1)
                    qos_map.reliability = ReliabilityPolicy.BEST_EFFORT
                    qos_map.history = HistoryPolicy.KEEP_LAST
                    self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, qos_map)
                except Exception:
                    self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, 1)
            else:
                self.get_logger().info('nav_msgs/OccupancyGrid not available; map streaming disabled')
        except Exception as e:
            self.get_logger().warn(f'Failed to create map subscription: {e}')

        # Periodic snapshot broadcaster (push stacked data to clients)
        # Ensures UI receives all current measurements in a single, ordered payload
        # even when individual sensors update at different rates.
        # Runs at 20 Hz to match UI expectations.
        try:
            self._snapshot_timer = self.create_timer(1.0 / 20.0, self._broadcast_snapshot)
        except Exception:
            self._snapshot_timer = None

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
        # Initialize per-client systemd watch map
        try:
            self._systemd_watch[websocket] = {}
        except Exception:
            pass
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
            # include module configurations
            try:
                status_msg['modules'] = self._modules
            except Exception as e:
                self.get_logger().warn(f'Failed to get modules: {e}')
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
            # Cleanup any watches for this client
            try:
                self._systemd_watch.pop(websocket, None)
            except Exception:
                pass
            # Stop watcher task if no watchers remain
            try:
                self._maybe_stop_watch_task()
            except Exception:
                pass
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
                # include modules snapshot
                try:
                    pong['modules'] = self._modules
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
            elif t == 'module_control':
                # { type: 'module_control', kind: 'action'|'change', module: <name>, id: <control_id>, action?: <str>, value?: <any> }
                try:
                    kind = str((data.get('kind') or 'change')).lower()
                    module = str(data.get('module') or '')
                    cid = str(data.get('id') or '')
                    action = data.get('action')
                    value = data.get('value')
                    if not module or not cid or kind not in ('action', 'change'):
                        raise ValueError('invalid module control payload')
                    # Build JSON payload for ROS consumer modules
                    payload = {
                        'kind': kind,
                        'module': module,
                        'id': cid,
                        'action': action,
                        'value': value,
                        'ts': asyncio.get_event_loop().time() if asyncio.get_event_loop() else 0.0,
                    }
                    # Publish to generic topic
                    if self.module_control_pub is not None:
                        try:
                            self.module_control_pub.publish(String(data=json.dumps(payload)))
                        except Exception:
                            pass
                    # Ack back to client
                    await websocket.send(json.dumps({'type': 'module_control_ack', **payload}))
                except Exception as e:
                    await websocket.send(json.dumps({'type': 'error', 'error': f'module_control: {e}'}))
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
                        module_name = self._module_unit_map.get(unit)
                        if module_name:
                            status['module'] = module_name
                        # also include a detail snapshot for immediate UI update
                        try:
                            detail = self._systemd_detail(unit, int(data.get('lines') or 200))
                        except Exception:
                            detail = None
                        payload = {'type': 'systemd', 'unit': unit, 'status': status}
                        if module_name:
                            payload['module'] = module_name
                        if detail:
                            payload['detail'] = detail
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
                        payload = {'type': 'systemd', 'unit': unit, 'detail': detail}
                        module_name = self._module_unit_map.get(unit)
                        if module_name:
                            payload['module'] = module_name
                        await websocket.send(json.dumps(payload))
                elif action in ('watch', 'unwatch', 'watch_off'):
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
                        if action == 'watch':
                            # Register watch and send immediate snapshot
                            try:
                                self._systemd_watch.setdefault(websocket, {})[unit] = {'lines': n}
                                detail = self._systemd_detail(unit, n)
                                payload = {'type': 'systemd', 'unit': unit, 'detail': detail}
                                module_name = self._module_unit_map.get(unit)
                                if module_name:
                                    payload['module'] = module_name
                                await websocket.send(json.dumps(payload))
                                self._ensure_watch_task()
                            except Exception as e:
                                await websocket.send(json.dumps({'type': 'systemd', 'error': f'watch failed: {e}'}))
                        else:
                            try:
                                ws_map = self._systemd_watch.get(websocket, {})
                                if unit in ws_map:
                                    del ws_map[unit]
                                if not ws_map:
                                    self._systemd_watch.pop(websocket, None)
                                self._maybe_stop_watch_task()
                                payload = {'type': 'systemd', 'unit': unit, 'unwatched': True}
                                module_name = self._module_unit_map.get(unit)
                                if module_name:
                                    payload['module'] = module_name
                                await websocket.send(json.dumps(payload))
                            except Exception as e:
                                await websocket.send(json.dumps({'type': 'systemd', 'error': f'unwatch failed: {e}'}))
            elif t == 'save_map':
                # Trigger save_map helper script asynchronously
                name = data.get('name') or 'nav_map'
                try:
                    res = self._run_background_script(['modules/nav/save_map.sh', name])
                    await websocket.send(json.dumps({'type': 'save_map_ack', 'name': name, 'pid': res}))
                except Exception as e:
                    await websocket.send(json.dumps({'type': 'error', 'error': f'save_map failed: {e}'}))
            elif t == 'record_bag':
                # Start ros2 bag recording in background; optional name arg
                name = data.get('name') or 'nav_record'
                try:
                    pid = self._start_recording(name)
                    await websocket.send(json.dumps({'type': 'record_started', 'name': name, 'pid': pid}))
                except Exception as e:
                    await websocket.send(json.dumps({'type': 'error', 'error': f'record_bag failed: {e}'}))
            elif t == 'record_stop' or t == 'record_bag_stop':
                name = data.get('name') or None
                try:
                    stopped = self._stop_recording(name)
                    await websocket.send(json.dumps({'type': 'record_stopped', 'stopped': stopped}))
                except Exception as e:
                    await websocket.send(json.dumps({'type': 'error', 'error': f'stop_record failed: {e}'}))
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

    # -----------------------------
    # Image handling (color + depth)
    # -----------------------------
    def _encode_and_send_image(self, topic: str, b64data: str):
        if not self.connected_clients or self._loop is None:
            return
        payload = json.dumps({'type': 'image', 'topic': topic, 'data': b64data})

        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(payload)
                except Exception:
                    pass

        try:
            asyncio.run_coroutine_threadsafe(_send_all(), self._loop)
        except Exception:
            pass

    def _on_image(self, msg: Image):
        # Convert ROS Image to JPEG base64 and broadcast
        if self._bridge is None or cv2 is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        try:
            ret, buf = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ret:
                return
            b64 = 'data:image/jpeg;base64,' + (buf.tobytes().encode('base64') if False else __import__('base64').b64encode(buf.tobytes()).decode('ascii'))
            self._encode_and_send_image('/image_raw', b64)
        except Exception:
            return

    def _on_depth(self, msg: Image):
        # Convert depth to a visualizable color image, JPEG encode and broadcast
        if self._bridge is None or cv2 is None or np is None:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return
        try:
            # Normalize to 0-255
            d = np.nan_to_num(depth, nan=0.0)
            amin = float(np.min(d)) if d.size else 0.0
            amax = float(np.max(d)) if d.size else 1.0
            if amax - amin <= 1e-6:
                amax = amin + 1.0
            norm = (d - amin) / (amax - amin)
            d8 = (np.clip(norm, 0.0, 1.0) * 255.0).astype(np.uint8)
            colored = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
            ret, buf = cv2.imencode('.jpg', colored, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ret:
                return
            b64 = 'data:image/jpeg;base64,' + __import__('base64').b64encode(buf.tobytes()).decode('ascii')
            self._encode_and_send_image('/depth/image_raw', b64)
        except Exception:
            return

    def _on_map(self, msg: OccupancyGrid):
        """Handle OccupancyGrid messages and broadcast as an image or raw payload.

        If OpenCV + numpy are available, render an image and send as the existing
        'image' message (topic '/map') so the frontend can reuse the image handler.
        Otherwise, send a compact raw base64 payload with width/height for the
        frontend to decode and render.
        """
        try:
            w = int(msg.info.width)
            h = int(msg.info.height)
            data = list(msg.data) if hasattr(msg, 'data') else []
        except Exception:
            return

        # Prefer image encoding when OpenCV is present
        if cv2 is not None and np is not None:
            try:
                arr = np.array(data, dtype=np.int8).reshape((h, w))
                # Create a BGR image: unknown=127, free=255 (white), occ=0 (black)
                img = np.full((h, w, 3), 127, dtype=np.uint8)
                free_mask = (arr == 0)
                occ_mask = (arr > 0)
                img[free_mask] = [255, 255, 255]
                img[occ_mask] = [0, 0, 0]
                # Flip vertically to match display conventions
                img = np.flipud(img)
                ret, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ret:
                    b64 = 'data:image/jpeg;base64,' + base64.b64encode(buf.tobytes()).decode('ascii')
                    # Reuse existing image broadcaster with topic '/map'
                    self._encode_and_send_image('/map', b64)
                    return
            except Exception as e:
                self.get_logger().warn(f'Failed to encode map image: {e}')

        # Fallback: send raw occupancy bytes as base64 for client-side rendering
        try:
            # Pack as signed bytes (-1..100) into single byte values by offsetting 128
            raw = bytes([(x + 128) & 0xFF for x in data])
            b64 = base64.b64encode(raw).decode('ascii')
            payload = json.dumps({
                'type': 'map_raw',
                'topic': self.map_topic,
                'width': w,
                'height': h,
                'resolution': float(msg.info.resolution) if hasattr(msg.info, 'resolution') else None,
                'data': b64,
            })

            async def _send_all():
                for client in list(self.connected_clients):
                    try:
                        await client.send(payload)
                    except Exception:
                        pass

            if self._loop:
                asyncio.run_coroutine_threadsafe(_send_all(), self._loop)
        except Exception as e:
            self.get_logger().warn(f'Failed to send raw map payload: {e}')

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

    # -----------------------------
    # Snapshot broadcaster (stacked messages)
    # -----------------------------
    def _build_snapshot(self):
        """Return a dict with the ordered snapshot of sensor/module state."""
        snap = {'type': 'snapshot'}

        # IMU (last)
        snap['imu'] = self._imu_last if self._imu_last is not None else None

        # GPS
        snap['gps_fix'] = self._gps_fix if self._gps_fix is not None else None

        # Battery
        with self._battery_lock:
            if any(v is not None for v in self._battery.values()):
                snap['battery'] = self._format_battery()
            else:
                snap['battery'] = None

        # Robot snapshot
        with self._robot_lock:
            snap['robot_status'] = self._format_robot_status()

        # Host health
        with self._host_lock:
            snap['host_health'] = self._host_health

        # Audio
        snap['audio'] = self._audio

        # Modules and systemd services (small lists)
        try:
            snap['modules'] = self._modules
        except Exception:
            snap['modules'] = None
        try:
            services = self._list_systemd_status()
            snap['systemd_services'] = services
        except Exception:
            snap['systemd_services'] = None

        return snap

    def _broadcast_snapshot(self):
        # Build deterministic snapshot and send to all connected clients
        if not self.connected_clients or self._loop is None:
            return
        snap = self._build_snapshot()
        # Ensure ordering of keys for predictable parsing on the frontend
        # (Python 3.7+ preserves insertion order)
        try:
            data = json.dumps(snap)
        except Exception:
            return

        async def _send_all():
            for client in list(self.connected_clients):
                try:
                    await client.send(data)
                except Exception:
                    pass

        try:
            asyncio.run_coroutine_threadsafe(_send_all(), self._loop)
        except Exception:
            pass

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
    def _discover_modules(self):
        """Discover modules and load their pilot control configurations."""
        modules: dict[str, dict] = {}
        self._module_unit_map = {}
        try:
            host_short = self._host_short or 'host'
            repo_dir_env = os.environ.get('REPO_DIR')
            if repo_dir_env and Path(repo_dir_env).exists():
                repo_root = Path(repo_dir_env)
            else:
                repo_root = Path(__file__).resolve()
                for _ in range(10):
                    if (repo_root / 'hosts').exists() and (repo_root / 'modules').exists():
                        break
                    repo_root = repo_root.parent
            modules_dir = repo_root / 'hosts' / host_short / 'modules'

            if not modules_dir.exists():
                self.get_logger().warn(f'No modules directory found at {modules_dir}')
                return modules

            def _register_units(module_name: str, entry: dict) -> None:
                units = entry.get('systemd_units') or []
                for unit in units:
                    if isinstance(unit, str) and unit:
                        self._module_unit_map[unit] = module_name

            def _make_entry(module_name: str, config: dict | None) -> dict:
                cfg = config or {}
                display = cfg.get('display') or cfg.get('name')
                if not display:
                    display = module_name.replace('_', ' ').replace('-', ' ').title()
                description = cfg.get('description') or f'{display} module'
                controls = cfg.get('controls') if isinstance(cfg.get('controls'), list) else []
                raw_units = []
                units_field = cfg.get('systemd_units')
                if isinstance(units_field, list):
                    raw_units.extend(units_field)
                elif isinstance(units_field, str):
                    raw_units.append(units_field)
                single_unit = cfg.get('systemd_unit')
                if isinstance(single_unit, str):
                    raw_units.append(single_unit)
                deduped: list[str] = []
                seen = set()
                for unit in raw_units:
                    if not isinstance(unit, str):
                        continue
                    clean = unit.strip()
                    if not clean or clean in seen:
                        continue
                    seen.add(clean)
                    deduped.append(clean)
                if not deduped:
                    deduped.append(f'psyched-{module_name}.service')
                entry = {
                    'name': display,
                    'description': description,
                    'controls': controls,
                    'systemd_units': deduped,
                    'systemd_unit': deduped[0],
                    'slug': cfg.get('slug') or cfg.get('module') or module_name,
                    'module': module_name,
                }
                return entry

            for module_link in sorted(modules_dir.iterdir(), key=lambda p: p.name):
                if not module_link.is_symlink() and not module_link.is_dir():
                    continue

                module_name = module_link.name
                try:
                    module_path = module_link.resolve(strict=False)
                except Exception as e:
                    self.get_logger().warn(f'Failed to resolve module link {module_name}: {e}')
                    entry = _make_entry(module_name, None)
                    modules[module_name] = entry
                    _register_units(module_name, entry)
                    continue

                if not module_path.exists():
                    self.get_logger().warn(f'Module path missing for {module_name}: {module_path}')
                    entry = _make_entry(module_name, None)
                    modules[module_name] = entry
                    _register_units(module_name, entry)
                    continue

                controls_file = module_path / 'pilot_controls.json'
                entry = None
                if controls_file.exists():
                    try:
                        with open(controls_file, 'r', encoding='utf-8') as f:
                            controls_config = json.loads(f.read())
                        controls_config = self._expand_env_vars(controls_config)
                        entry = _make_entry(module_name, controls_config)
                        self.get_logger().info(f'Loaded pilot controls for module: {module_name}')
                    except Exception as e:
                        self.get_logger().warn(f'Failed to load pilot controls for {module_name}: {e}')

                if entry is None:
                    entry = _make_entry(module_name, None)

                modules[module_name] = entry
                _register_units(module_name, entry)

            return modules
        except Exception as e:
            self.get_logger().warn(f'Failed to discover modules: {e}')
            return modules
            
    def _expand_env_vars(self, config):
        """Recursively expand environment variables in configuration values."""
        import re
        
        def expand_value(value):
            if isinstance(value, str):
                # Replace ${VAR:-default} patterns
                pattern = r'\$\{([^}:]+)(?::-(.*?))?\}'
                def replacer(match):
                    var_name = match.group(1)
                    default_value = match.group(2) or ''
                    return os.environ.get(var_name, default_value)
                return re.sub(pattern, replacer, value)
            elif isinstance(value, dict):
                return {k: expand_value(v) for k, v in value.items()}
            elif isinstance(value, list):
                return [expand_value(item) for item in value]
            else:
                return value
                
        return expand_value(config)

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

    def _run_background_script(self, args):
        """Run a shell helper script in background returning the pid (or raise).

        Args is a list like ['modules/nav/save_map.sh', 'name'] (relative repo path).
        """
        repo_root = Path(__file__).resolve().parents[3]
        script = repo_root.joinpath(*args)
        # If args list provided includes path components, script currently resolves wrong; accept full path string join
        try:
            # If first element is a path to script relative to repo, normalize
            if isinstance(args, (list, tuple)):
                spath = repo_root.joinpath(args[0]) if not Path(args[0]).is_absolute() else Path(args[0])
                cmd = [str(spath)] + list(args[1:])
            else:
                cmd = [str(args)]
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            with self._bg_lock:
                self._bg_recordings[p.pid] = p
            self.get_logger().info(f'Started background script: {cmd} (pid {p.pid})')
            return p.pid
        except Exception as e:
            raise

    def _start_recording(self, name='nav_record'):
        """Start ros2 bag recording via helper script and return pid."""
        repo_root = Path(__file__).resolve().parents[3]
        script = repo_root / 'modules' / 'nav' / 'record_bag.sh'
        if not script.exists():
            raise FileNotFoundError(str(script))
        try:
            p = subprocess.Popen([str(script), name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            with self._bg_lock:
                self._bg_recordings[p.pid] = p
            self.get_logger().info(f'Started ros2 bag recording: {name} (pid {p.pid})')
            return p.pid
        except Exception as e:
            self.get_logger().warn(f'Failed to start recording: {e}')
            raise

    def _stop_recording(self, name=None):
        """Stop background recording(s). If name is None, stop all recordings started by this process."""
        stopped = []
        with self._bg_lock:
            for pid, proc in list(self._bg_recordings.items()):
                try:
                    proc.terminate()
                    proc.wait(timeout=2)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
                stopped.append(pid)
                del self._bg_recordings[pid]
        return stopped

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
        # Return full service info including status and journal logs for each unit
        services = []
        for u in self._systemd_units:
            base = self._query_systemd_unit(u)
            module_name = self._module_unit_map.get(u)
            if module_name:
                base['module'] = module_name
            detail = self._systemd_detail(u, lines=200)
            # Merge detail fields into base dict
            base.update(detail)
            services.append(base)
        return services

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
    # Systemd watch helpers (periodic log/status updates)
    # -----------------------------
    def _ensure_watch_task(self):
        try:
            active = any(bool(v) for v in self._systemd_watch.values())
        except Exception:
            active = False
        if not active or self._loop is None:
            return
        try:
            task = self._systemd_watch_task
            done = getattr(task, 'done', lambda: True)() if task is not None else True
            cancelled = getattr(task, 'cancelled', lambda: True)() if task is not None else True
        except Exception:
            done = True
            cancelled = True
        if task is None or done or cancelled:
            try:
                self._systemd_watch_task = asyncio.run_coroutine_threadsafe(self._watch_loop(), self._loop)
            except Exception:
                self._systemd_watch_task = None

    def _maybe_stop_watch_task(self):
        try:
            active = any(bool(v) for v in self._systemd_watch.values())
        except Exception:
            active = False
        if not active and self._systemd_watch_task is not None:
            try:
                self._systemd_watch_task.cancel()
            except Exception:
                pass
            self._systemd_watch_task = None

    async def _watch_loop(self):
        try:
            while True:
                await asyncio.sleep(3.0)
                # Build snapshot to avoid mutation during iteration
                snapshot = []
                for ws, units in list(self._systemd_watch.items()):
                    if not units:
                        continue
                    for unit, meta in list(units.items()):
                        lines = int(meta.get('lines', 200) or 200)
                        snapshot.append((ws, unit, lines))
                if not snapshot:
                    continue
                async def fetch_detail(unit, lines):
                    return await asyncio.to_thread(self._systemd_detail, unit, lines)
                tasks = [fetch_detail(unit, lines) for (_ws, unit, lines) in snapshot]
                results = await asyncio.gather(*tasks, return_exceptions=True)
                for idx, (ws, unit, _lines) in enumerate(snapshot):
                    if idx >= len(results):
                        continue
                    detail = results[idx]
                    if isinstance(detail, Exception):
                        continue
                    try:
                        payload = {'type': 'systemd', 'unit': unit, 'detail': detail}
                        module_name = self._module_unit_map.get(unit)
                        if module_name:
                            payload['module'] = module_name
                        await ws.send(json.dumps(payload))
                    except Exception:
                        pass
        except asyncio.CancelledError:
            return
        except Exception as e:
            self.get_logger().warn(f'systemd watch loop error: {e}')

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
