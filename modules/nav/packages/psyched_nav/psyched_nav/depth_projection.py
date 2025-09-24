"""
depth_projection.py
"""
import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

class DepthToScan:
    def __init__(self, fx, fy, cx, cy, min_y, max_y, min_range=0.2, max_range=5.0):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.min_y = min_y
        self.max_y = max_y
        self.min_range = min_range
        self.max_range = max_range

    def project_depth_to_scan(self, depth_img):
        h, w = depth_img.shape
        scan_ranges = []
        line_mask = np.zeros_like(depth_img, dtype=np.uint8)
        for x in range(w):
            col = depth_img[self.min_y:self.max_y, x]
            valid = (col > self.min_range) & (col < self.max_range)
            if np.any(valid):
                idx = np.argmin(col[valid])
                y = np.arange(self.min_y, self.max_y)[valid][idx]
                r = col[valid][idx]
                scan_ranges.append(float(r))
                line_mask[y, x] = 255
            else:
                scan_ranges.append(float('inf'))
        return scan_ranges, line_mask

    def make_laserscan_msg(self, scan_ranges, frame_id, angle_min=-1.57, angle_max=1.57):
        scan = LaserScan()
        scan.header.frame_id = frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = (angle_max - angle_min) / len(scan_ranges)
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = scan_ranges
        return scan

    def overlay_line_on_image(self, rgb_img, line_mask, color=(0, 255, 0)):
        overlay = rgb_img.copy()
        overlay[line_mask > 0] = color
        return overlay

    def kinect_tf(self, x, y, z, parent_frame="base_link", child_frame="kinect_link"):
        tf = TransformStamped()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        return tf
"""
depth_projection.py

Converts Kinect depth images to pseudo-LaserScan data for AMCL mapping.
Highlights nearest obstacle line for vision LLM annotation.
"""
import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

class DepthToScan:
    def __init__(self, fx, fy, cx, cy, min_y, max_y, min_range=0.2, max_range=5.0):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.min_y = min_y
        self.max_y = max_y
        self.min_range = min_range
        self.max_range = max_range

    def project_depth_to_scan(self, depth_img):
        h, w = depth_img.shape
        scan_ranges = []
        line_mask = np.zeros_like(depth_img, dtype=np.uint8)
        for x in range(w):
            col = depth_img[self.min_y:self.max_y, x]
            valid = (col > self.min_range) & (col < self.max_range)
            if np.any(valid):
                idx = np.argmin(col[valid])
                y = np.arange(self.min_y, self.max_y)[valid][idx]
                r = col[valid][idx]
                scan_ranges.append(float(r))
                line_mask[y, x] = 255
            else:
                scan_ranges.append(float('inf'))
        return scan_ranges, line_mask

    def make_laserscan_msg(self, scan_ranges, frame_id, angle_min=-1.57, angle_max=1.57):
        scan = LaserScan()
        scan.header.frame_id = frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = (angle_max - angle_min) / len(scan_ranges)
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = scan_ranges
        return scan

    def overlay_line_on_image(self, rgb_img, line_mask, color=(0, 255, 0)):
        overlay = rgb_img.copy()
        overlay[line_mask > 0] = color
        return overlay

    def kinect_tf(self, x, y, z, parent_frame="base_link", child_frame="kinect_link"):
        tf = TransformStamped()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        return tf
