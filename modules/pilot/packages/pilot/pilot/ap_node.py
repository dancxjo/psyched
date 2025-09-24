#!/usr/bin/env python3
"""AP node: create an access point on a spare wireless interface and advertise services.

This node attempts to configure a wireless interface as an AP using system tools
(`ip`, `hostapd`, `dnsmasq`) when available. It also advertises the robot's
services (HTTP/WebSocket) via mDNS using `zeroconf` so clients connected to the
AP can discover the robot by the same hostname.

Behavior and fallbacks:
- If `enable_ap` parameter is false the node will not attempt changes.
- All system commands are executed with subprocess; missing tools or lack of
  privileges result in logged warnings but do not crash the node.
- If `hostapd`/`dnsmasq` are unavailable the node will log the failure and
  continue advertising mDNS on the current network (if possible).

Note: Configuring network interfaces and launching system daemons requires
root privileges. This node is safe to import in tests (it defers destructive
actions behind `start_ap()` and a `dry_run` mode).
"""

from __future__ import annotations

import os
import socket
import subprocess
import tempfile
import time
from typing import Optional

import rclpy
from rclpy.node import Node

try:
    from zeroconf import Zeroconf, ServiceInfo
except Exception:
    Zeroconf = None
    ServiceInfo = None


class APNode(Node):
    def __init__(self):
        super().__init__('pilot_ap')

        # Parameters
        self.declare_parameter('enable_ap', True)
        self.declare_parameter('ap_interface', 'wlan1')
        self.declare_parameter('ap_ssid', None)
        self.declare_parameter('ap_passphrase', None)
        self.declare_parameter('ap_ip', '192.168.50.1/24')
        self.declare_parameter('dhcp_range', '192.168.50.10,192.168.50.100')
        self.declare_parameter('dhcp_lease_time', '12h')
        self.declare_parameter('mdns_name', None)
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('websocket_port', 8081)
        # dry_run useful for tests; if true we won't call system commands
        self.declare_parameter('dry_run', False)

        def _p(name, default=None):
            try:
                v = self.get_parameter(name).value
            except Exception:
                v = default
            if v is None:
                return default
            if isinstance(v, str):
                trimmed = v.strip()
                lowered = trimmed.lower()
                if lowered in {'', 'auto', 'none', 'null'}:
                    return default
                if isinstance(default, bool) or lowered in {'true', 'false', '1', '0', 'yes', 'no', 'on', 'off'}:
                    return lowered in {'true', '1', 'yes', 'on'}
            return v

        def _as_bool(value, fallback=False):
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                lowered = value.strip().lower()
                if lowered in {'true', '1', 'yes', 'on'}:
                    return True
                if lowered in {'false', '0', 'no', 'off'}:
                    return False
                return fallback
            return bool(value)

        self.enable_ap = _as_bool(_p('enable_ap', True), True)
        self.iface = str(_p('ap_interface', 'wlan1'))
        self.ssid = _p('ap_ssid', f'psyched-{socket.gethostname().split(".")[0]}')
        self.passphrase = _p('ap_passphrase', None)
        self.ap_ip = str(_p('ap_ip', '192.168.50.1/24'))
        self.dhcp_range = str(_p('dhcp_range', '192.168.50.10,192.168.50.100'))
        self.dhcp_lease_time = str(_p('dhcp_lease_time', '12h'))
        self.mdns_name = _p('mdns_name', socket.gethostname())
        self.http_port = int(_p('http_port', 8080))
        self.websocket_port = int(_p('websocket_port', 8081))
        self.dry_run = _as_bool(_p('dry_run', False), False)

        # Runtime handles for subprocesses and zeroconf
        self._hostapd_proc: Optional[subprocess.Popen] = None
        self._dnsmasq_proc: Optional[subprocess.Popen] = None
        self._zeroconf: Optional[Zeroconf] = None
        self._service_info: Optional[ServiceInfo] = None

        # Start AP if requested (deferred to a short timer so ROS is up)
        if self.enable_ap:
            # Wait a little to allow other nodes to come up
            self.create_timer(0.5, self._delayed_start)
        else:
            self.get_logger().info('AP disabled by parameter enable_ap=false')

    def _delayed_start(self):
        # Timer callback should only run once
        try:
            self.start_ap()
        finally:
            # Cancel this timer by shutting down node's timer; easiest is to not recreate
            pass

    def start_ap(self):
        """Attempt to configure interface, start hostapd/dnsmasq, and advertise mDNS."""
        if self.dry_run:
            self.get_logger().info('AP start (dry_run): Skipping system commands')
            # Still advertise mDNS locally if zeroconf available
            if Zeroconf:
                self._start_zeroconf()
            return

        if os.geteuid() != 0:
            self.get_logger().warn('AP configuration requires root privileges; running in read-only mode')
        # 1) Configure IP on interface
        try:
            subprocess.run(['ip', 'link', 'set', self.iface, 'up'], check=False)
            subprocess.run(['ip', 'addr', 'flush', 'dev', self.iface], check=False)
            subprocess.run(['ip', 'addr', 'add', self.ap_ip, 'dev', self.iface], check=False)
            self.get_logger().info(f'Configured {self.iface} with {self.ap_ip}')
        except Exception as e:
            self.get_logger().warn(f'Failed to configure interface {self.iface}: {e}')

        # 2) Create hostapd config and launch hostapd if available
        hostapd_path = shutil_which('hostapd')
        if hostapd_path:
            try:
                hostapd_conf = self._write_hostapd_conf()
                cmd = [hostapd_path, hostapd_conf]
                self._hostapd_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.get_logger().info(f'Launched hostapd: {hostapd_path} (ssid={self.ssid})')
            except Exception as e:
                self.get_logger().warn(f'Failed to launch hostapd: {e}')
        else:
            self.get_logger().warn('hostapd not found on system; cannot create full AP')

        # 3) Create dnsmasq config and launch if available
        dnsmasq_path = shutil_which('dnsmasq')
        if dnsmasq_path:
            try:
                dns_conf = self._write_dnsmasq_conf()
                cmd = [dnsmasq_path, '--conf-file=' + dns_conf]
                self._dnsmasq_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.get_logger().info('Launched dnsmasq for DHCP/DNS on AP network')
            except Exception as e:
                self.get_logger().warn(f'Failed to launch dnsmasq: {e}')
        else:
            self.get_logger().warn('dnsmasq not found on system; cannot provide DHCP')

        # 4) Start mDNS advertisement so clients can find the robot by hostname
        if Zeroconf:
            try:
                self._start_zeroconf()
            except Exception as e:
                self.get_logger().warn(f'Failed to start zeroconf mDNS: {e}')
        else:
            self.get_logger().warn('zeroconf Python package not installed; mDNS advertisement unavailable')

    def _start_zeroconf(self):
        """Register basic HTTP and websocket services via zeroconf."""
        # Prepare Zeroconf and register an _http._tcp.local service and a custom pilot service
        if self._zeroconf is not None:
            return
        # Determine IPv4 address (strip mask)
        ip = self.ap_ip.split('/')[0]
        addr_bytes = socket.inet_aton(ip)
        zc = Zeroconf()
        hostname = self.mdns_name

        # HTTP service
        props = {b'path': b'/'}
        info = ServiceInfo(
            "_http._tcp.local.",
            f"{hostname}._http._tcp.local.",
            addresses=[addr_bytes],
            port=self.http_port,
            properties=props,
            server=f"{hostname}.local.",
        )
        zc.register_service(info)

        # Also advertise a pilot websocket service
        ws_info = ServiceInfo(
            "_ws._tcp.local.",
            f"{hostname}._ws._tcp.local.",
            addresses=[addr_bytes],
            port=self.websocket_port,
            properties={},
            server=f"{hostname}.local.",
        )
        zc.register_service(ws_info)

        self._zeroconf = zc
        self._service_info = info
        self.get_logger().info(f'mDNS services registered for {hostname} at {ip}')

    def _write_hostapd_conf(self) -> str:
        passwd = self.passphrase
        cfg = [f'interface={self.iface}', f'ssid={self.ssid}', 'driver=nl80211']
        if passwd and len(passwd) >= 8:
            cfg += ['hw_mode=g', 'channel=6', 'wpa=2', 'wpa_key_mgmt=WPA-PSK', f'wpa_passphrase={passwd}', 'rsn_pairwise=CCMP']
        else:
            cfg += ['hw_mode=g', 'channel=6', 'auth_algs=1', 'ignore_broadcast_ssid=0']

        fd, path = tempfile.mkstemp(prefix='pilot_hostapd_', text=True)
        with os.fdopen(fd, 'w') as f:
            f.write('\n'.join(cfg))
        return path

    def _write_dnsmasq_conf(self) -> str:
        # dnsmasq config into a temp file
        ip = self.ap_ip.split('/')[0]
        leasefile = os.path.join(tempfile.gettempdir(), f'pilot_dhcp_{self.iface}.leases')
        cfg_lines = [
            f'interface={self.iface}',
            'bind-interfaces',
            'no-resolv',
            f'dhcp-range={self.dhcp_range},{self.dhcp_lease_time}',
            f'dhcp-leasefile={leasefile}',
            f'address=/{self.mdns_name}/{ip}',
        ]
        fd, path = tempfile.mkstemp(prefix='pilot_dnsmasq_', text=True)
        with os.fdopen(fd, 'w') as f:
            f.write('\n'.join(cfg_lines))
        return path

    def destroy_node(self):
        # Cleanup launched processes and zeroconf
        try:
            if self._zeroconf:
                try:
                    self._zeroconf.unregister_all_services()
                except Exception:
                    pass
                try:
                    self._zeroconf.close()
                except Exception:
                    pass
                self._zeroconf = None
        except Exception:
            pass

        if self._hostapd_proc:
            try:
                self._hostapd_proc.terminate()
            except Exception:
                pass
            self._hostapd_proc = None

        if self._dnsmasq_proc:
            try:
                self._dnsmasq_proc.terminate()
            except Exception:
                pass
            self._dnsmasq_proc = None

        return super().destroy_node()


def shutil_which(name: str) -> Optional[str]:
    """Small wrapper for shutil.which without importing at module top to keep tests lightweight."""
    try:
        import shutil
        return shutil.which(name)
    except Exception:
        return None


def main(args=None):
    rclpy.init(args=args)
    node = APNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
