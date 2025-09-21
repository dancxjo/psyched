import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import Header

try:
    from gps3.agps3threaded import AGPS3mechanism
    GPS3_AVAILABLE = True
except Exception:
    GPS3_AVAILABLE = False


class UbloxGpsNode(Node):
    def __init__(self):
        super().__init__('ublox_gps_node')
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_rate', 5.0)  # Hz
        self.declare_parameter('device', '/dev/gps0')  # informational; gpsd handles it

        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', qos)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', qos)

        self.frame_id = frame_id
        self.period = 1.0 / max(0.1, publish_rate)

        self._last_report = None
        self._gps_thread: Optional[threading.Thread] = None
        self._gps = None

        if not GPS3_AVAILABLE:
            self.get_logger().error('gps3 module not available. Install with: pip install gps3')
        else:
            self._start_gpsd_client()

        self.timer = self.create_timer(self.period, self._on_timer)

    def _start_gpsd_client(self):
        self._gps = AGPS3mechanism()
        self._gps.stream_data()
        self._gps.run_thread()
        self.get_logger().info('Started gpsd client thread (gps3)')

    def _on_timer(self):
        if not GPS3_AVAILABLE or self._gps is None:
            return
        data = self._gps.data_stream
        # gps3 exposes properties like lat, lon, alt, speed, track, time, mode, climb, etc.
        try:
            if getattr(data, 'lat', None) is None or getattr(data, 'lon', None) is None:
                return
            msg = NavSatFix()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id
            msg.header = header

            # NavSatStatus: service bitmask is unknown; status based on mode
            status = NavSatStatus()
            mode = getattr(data, 'mode', 1)
            if mode >= 2:
                status.status = NavSatStatus.STATUS_FIX
            else:
                status.status = NavSatStatus.STATUS_NO_FIX
            status.service = 0  # unknown service
            msg.status = status

            msg.latitude = float(data.lat)
            msg.longitude = float(data.lon)
            try:
                msg.altitude = float(data.alt)
            except Exception:
                msg.altitude = 0.0

            # Position covariance unknown; set to unknown per REP 145
            msg.position_covariance = [0.0] * 9
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.fix_pub.publish(msg)

            # Publish time reference if available
            tre = TimeReference()
            tre.header = header
            if getattr(data, 'time', None):
                try:
                    tre.time_ref = header.stamp
                    tre.source = 'gpsd'
                    self.time_ref_pub.publish(tre)
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().debug(f'gps publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UbloxGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

