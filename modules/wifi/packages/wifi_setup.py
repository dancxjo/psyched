import os
import rclpy
from rclpy.node import Node

DNSMASQ_CONF = os.path.join(os.path.dirname(__file__), '..', 'dnsmasq.conf')

DEFAULT_DNSMASQ = '''interface=wlan0
dhcp-range=192.168.12.10,192.168.12.100,12h
domain-needed
bogus-priv
'''

class WifiStatusPublisher(Node):
    def __init__(self):
        super().__init__('wifi_status_publisher')
        self.publisher_ = self.create_publisher(
            rclpy.qos.QoSProfile(depth=10),
            'wifi_status',
            rclpy.qos.QoSProfile(depth=10)
        )
        self.report_status()

    def report_status(self):
        status = self.check_dnsmasq_conf()
        msg = f"Wi-Fi setup: {status}"
        self.get_logger().info(msg)
        # Publish as a string (std_msgs/String)
        from std_msgs.msg import String
        self.publisher_.publish(String(data=msg))

    def check_dnsmasq_conf(self):
        if os.path.exists(DNSMASQ_CONF):
            return f"dnsmasq.conf found at {DNSMASQ_CONF}"
        else:
            try:
                with open(DNSMASQ_CONF, 'w') as f:
                    f.write(DEFAULT_DNSMASQ)
                return f"dnsmasq.conf created at {DNSMASQ_CONF}"
            except Exception as e:
                return f"Failed to create dnsmasq.conf: {e}"

def main(args=None):
    rclpy.init(args=args)
    node = WifiStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
