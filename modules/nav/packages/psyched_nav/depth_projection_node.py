"""ROS 2 node that projects depth images into LaserScan messages."""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

from psyched_nav.depth_projection import DepthToScan

class DepthProjectionNode(Node):
    """Subscribe to depth images and publish synthesized LaserScan data."""

    def __init__(self):
        super().__init__('depth_projection')
        self.bridge = CvBridge()
        self.d2s = DepthToScan(fx=525, fy=525, cx=160, cy=120, min_y=100, max_y=140)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.overlay_pub = self.create_publisher(Image, 'scan_overlay', 10)
        self.create_subscription(Image, 'kinect/depth', self.depth_callback, 10)
        self.create_subscription(Image, 'kinect/rgb', self.rgb_callback, 10)
        self.last_rgb = None

    def depth_callback(self, msg):
        """Convert incoming depth images into a pseudo laser scan."""

        depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        scan_ranges, line_mask = self.d2s.project_depth_to_scan(depth_img)
        scan_msg = self.d2s.make_laserscan_msg(scan_ranges, frame_id="kinect_link")
        scan_msg.header.stamp = msg.header.stamp
        self.scan_pub.publish(scan_msg)
        if self.last_rgb is not None:
            rgb_img = self.bridge.imgmsg_to_cv2(self.last_rgb, desired_encoding='bgr8')
            overlay = self.d2s.overlay_line_on_image(rgb_img, line_mask)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)

    def rgb_callback(self, msg):
        """Cache the most recent RGB frame for overlay generation."""

        self.last_rgb = msg

def main(args=None):
    rclpy.init(args=args)
    node = DepthProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
