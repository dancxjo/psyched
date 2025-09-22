import numpy as np
import cv2
from nav.depth_projection import DepthToScan

def test_project_depth_to_scan():
    # Simulate depth image: 240x320, nearest obstacle at y=120
    depth_img = np.full((240,320), 2.0, dtype=np.float32)
    depth_img[120,:] = 0.5
    d2s = DepthToScan(fx=525, fy=525, cx=160, cy=120, min_y=100, max_y=140)
    scan_ranges, line_mask = d2s.project_depth_to_scan(depth_img)
    assert len(scan_ranges) == 320
    assert np.count_nonzero(line_mask) == 320

def test_overlay_line_on_image():
    rgb_img = np.zeros((240,320,3), dtype=np.uint8)
    line_mask = np.zeros((240,320), dtype=np.uint8)
    line_mask[120,:] = 255
    d2s = DepthToScan(525,525,160,120,100,140)
    overlay = d2s.overlay_line_on_image(rgb_img, line_mask)
    assert np.all(overlay[120,:] == [0,255,0])

def test_make_laserscan_msg():
    d2s = DepthToScan(525,525,160,120,100,140)
    scan_ranges = [1.0]*320
    msg = d2s.make_laserscan_msg(scan_ranges, frame_id="kinect_link")
    assert msg.header.frame_id == "kinect_link"
    assert len(msg.ranges) == 320
