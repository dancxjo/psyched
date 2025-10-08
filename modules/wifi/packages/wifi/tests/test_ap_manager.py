import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter

from wifi.ap_manager import WifiAccessPointNode


def test_ap_manager_dry_run_creates_state(tmp_path):
    """Ensure WifiAccessPointNode handles dry-run startup without side effects."""
    rclpy.init()
    node = WifiAccessPointNode()
    node.set_parameters([
        Parameter('dry_run', Parameter.Type.BOOL, True),
        Parameter('enable_ap', Parameter.Type.BOOL, True),
        Parameter('state_dir', Parameter.Type.STRING, str(tmp_path)),
        Parameter('interface', Parameter.Type.STRING, 'wlan-test'),
        Parameter('enable_nat', Parameter.Type.BOOL, True),
        Parameter('internet_interface', Parameter.Type.STRING, 'eth0'),
        Parameter('passphrase', Parameter.Type.STRING, 'validpass'),
    ])
    try:
        node.start_ap()
        time.sleep(0.1)
        assert tmp_path.exists()
    finally:
        node.destroy_node()
        rclpy.shutdown()
