#!/usr/bin/env python3
"""Publish host health metrics periodically on /host/health.

Fields (JSON in std_msgs/String):
- cpu_percent (float)
- load_avg_1, load_avg_5, load_avg_15 (float)
- mem_used_percent (float)
- mem_total_mb, mem_used_mb (float)
- temp_c (float, optional)
- uptime_sec (float)
- disk_used_percent_root (float)
"""
import json
import os
import time
import socket
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg

try:
    import psutil  # type: ignore
except Exception:
    psutil = None

class HostHealthNode(Node):
    def __init__(self):
        super().__init__('host_health')
        self.declare_parameter('publish_topic', 'auto')
        self.declare_parameter('period_sec', 2.0)
        topic_param = self.get_parameter('publish_topic').get_parameter_value().string_value
        if not topic_param or str(topic_param).lower() == 'auto':
            try:
                short = socket.gethostname().split('.')[0]
            except Exception:
                short = os.uname().nodename.split('.')[0] if hasattr(os, 'uname') else 'host'
            self.topic = f'hosts/{short}/health'
        else:
            self.topic = topic_param
        self.period = self.get_parameter('period_sec').get_parameter_value().double_value or 2.0
        self.pub = self.create_publisher(StringMsg, self.topic, 10)
        self.timer = self.create_timer(self.period, self._tick)
        self.boot_time = psutil.boot_time() if psutil else time.time()
        self.get_logger().info(f'Host health publishing on /{self.topic} every {self.period}s')

    def _read_temp(self) -> Optional[float]:
        # Try psutil sensors_temperatures
        if psutil and hasattr(psutil, 'sensors_temperatures'):
            try:
                temps = psutil.sensors_temperatures()
                # Pick the first reasonable temperature
                for name, entries in temps.items():
                    for e in entries:
                        if e.current is not None:
                            return float(e.current)
            except Exception:
                pass
        # Fallbacks: read from common linux paths (best-effort)
        paths = [
            '/sys/class/thermal/thermal_zone0/temp',
            '/sys/class/hwmon/hwmon0/temp1_input'
        ]
        for p in paths:
            try:
                with open(p, 'r') as f:
                    v = f.read().strip()
                    if v:
                        val = float(v) / (1000.0 if float(v) > 200 else 1.0)
                        return val
            except Exception:
                continue
        return None

    def _tick(self):
        now = time.time()
        data = {}
        try:
            if psutil:
                data['cpu_percent'] = float(psutil.cpu_percent(interval=None))
                la1, la5, la15 = os.getloadavg()
                data['load_avg_1'] = float(la1)
                data['load_avg_5'] = float(la5)
                data['load_avg_15'] = float(la15)
                vm = psutil.virtual_memory()
                data['mem_used_percent'] = float(vm.percent)
                data['mem_total_mb'] = round(vm.total / (1024*1024), 1)
                data['mem_used_mb'] = round((vm.total - vm.available) / (1024*1024), 1)
                disk = psutil.disk_usage('/')
                data['disk_used_percent_root'] = float(disk.percent)
                temp = self._read_temp()
                if temp is not None:
                    data['temp_c'] = float(temp)
                data['uptime_sec'] = float(now - self.boot_time)
            else:
                # Minimal fallback
                la1, la5, la15 = os.getloadavg()
                data.update({
                    'cpu_percent': None,
                    'load_avg_1': float(la1),
                    'load_avg_5': float(la5),
                    'load_avg_15': float(la15),
                    'mem_used_percent': None,
                    'mem_total_mb': None,
                    'mem_used_mb': None,
                    'disk_used_percent_root': None,
                    'temp_c': self._read_temp(),
                    'uptime_sec': None,
                })
        except Exception as e:
            self.get_logger().warn(f'Error collecting host health: {e}')
        try:
            msg = StringMsg()
            msg.data = json.dumps(data)
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Error publishing host health: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = HostHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
