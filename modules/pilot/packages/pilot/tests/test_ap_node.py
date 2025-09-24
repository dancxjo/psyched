import time

import rclpy

from pilot.ap_node import APNode


def test_apnode_dry_run_creates_object():
    """Ensure APNode can be created with dry_run=True and does not raise."""
    rclpy.init()
    node = APNode()
    # Set parameters for safe dry-run
    node.set_parameters([rclpy.parameter.Parameter('dry_run', rclpy.Parameter.Type.BOOL, True)])
    node.set_parameters([rclpy.parameter.Parameter('enable_ap', rclpy.Parameter.Type.BOOL, True)])
    try:
        # Call start_ap directly in dry-run mode
        node.start_ap()
        # Allow a short moment for any timers
        time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
