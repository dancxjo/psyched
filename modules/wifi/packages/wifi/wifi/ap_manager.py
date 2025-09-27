#!/usr/bin/env python3
"""Wi-Fi access point manager node.

This ROS 2 node configures a wireless interface as a managed (non-adhoc) access
point, launches DHCP/DNS services, advertises the host via mDNS, and optionally
configures internet sharing through NAT. It publishes a status string so other
components can surface Wi-Fi health, and supports a dry-run mode so tests can
exercise it without requiring root privileges.
"""

from __future__ import annotations

import json
import os
import socket
import subprocess
import tempfile
from pathlib import Path
from typing import Any, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:  # Optional dependency – installed via module actions but guarded for tests
    from zeroconf import ServiceInfo, Zeroconf
except Exception:  # pragma: no cover - exercised if zeroconf missing
    ServiceInfo = None
    Zeroconf = None


def _which(name: str) -> Optional[str]:
    """Lightweight wrapper around shutil.which lazily importing shutil."""
    try:
        import shutil

        return shutil.which(name)
    except Exception:  # pragma: no cover - shutil import should always succeed
        return None


class WifiAccessPointNode(Node):
    """Configure and maintain a Wi-Fi access point with DHCP/mDNS and optional NAT."""

    def __init__(self) -> None:
        super().__init__('wifi_ap_manager')

        # Parameter declarations – ROS parameters allow overriding at launch
        self.declare_parameter('enable_ap', True)
        self.declare_parameter('interface', 'wlan0')
        self.declare_parameter('ssid', None)
        self.declare_parameter('passphrase', None)
        self.declare_parameter('channel', 6)
        self.declare_parameter('country_code', 'US')
        self.declare_parameter('ap_ip', '192.168.50.1/24')
        self.declare_parameter('dhcp_range', '192.168.50.10,192.168.50.100')
        self.declare_parameter('dhcp_lease_time', '12h')
        self.declare_parameter('mdns_name', None)
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('websocket_port', 8081)
        self.declare_parameter('enable_nat', False)
        self.declare_parameter('internet_interface', 'eth0')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('state_dir', None)

        def _param(name: str, default=None):
            try:
                value = self.get_parameter(name).value
                return default if value is None else value
            except Exception:
                return default

        hostname = socket.gethostname().split('.')[0]
        self.enable_ap = bool(_param('enable_ap', True))
        self.iface = str(_param('interface', 'wlan0'))
        self.ssid = str(_param('ssid', f'psyched-{hostname}'))
        self.passphrase = _param('passphrase', None)
        self.channel = int(_param('channel', 6))
        self.country_code = str(_param('country_code', 'US'))
        self.ap_ip = str(_param('ap_ip', '192.168.50.1/24'))
        self.dhcp_range = str(_param('dhcp_range', '192.168.50.10,192.168.50.100'))
        self.dhcp_lease_time = str(_param('dhcp_lease_time', '12h'))
        self.mdns_name = str(_param('mdns_name', hostname))
        self.http_port = int(_param('http_port', 8080))
        self.websocket_port = int(_param('websocket_port', 8081))
        self.enable_nat = bool(_param('enable_nat', False))
        self.internet_iface = str(_param('internet_interface', 'eth0'))
        self.dry_run = bool(_param('dry_run', False))

        requested_state_dir = _param('state_dir', None)
        if requested_state_dir:
            state_dir = Path(str(requested_state_dir))
        else:
            # Default to /run/psyched-wifi/<iface> when available, fall back to /tmp
            default_base = Path('/run/psyched-wifi')
            state_dir = default_base / self.iface
            if not state_dir.exists():
                try:
                    state_dir.mkdir(parents=True, exist_ok=True)
                except PermissionError:
                    tmp_base = Path(tempfile.gettempdir()) / f'psyched_wifi_{self.iface}'
                    tmp_base.mkdir(parents=True, exist_ok=True)
                    state_dir = tmp_base
        state_dir.mkdir(parents=True, exist_ok=True)
        self.state_dir = state_dir

        # Runtime handles
        self._hostapd_proc: Optional[subprocess.Popen] = None
        self._dnsmasq_proc: Optional[subprocess.Popen] = None
        self._zeroconf: Optional[Zeroconf] = None
        self._zeroconf_infos: List[Any] = []
        self._nat_rules: List[List[str]] = []
        self._ip_forward_original: Optional[str] = None
        self._iface_state_file = self.state_dir / 'iface_state.json'
        self._nat_state_file = self.state_dir / 'nat_rules.json'
        self._hostapd_conf_path = self.state_dir / 'hostapd.conf'
        self._dnsmasq_conf_path = self.state_dir / 'dnsmasq.conf'
        self._dnsmasq_lease_path = self.state_dir / 'dnsmasq.leases'
        self._start_timer = None
        self._status_pub = self.create_publisher(String, 'wifi_status', 10)

        self.get_logger().info(
            f'Wi-Fi AP manager configured for interface={self.iface} ssid={self.ssid} '
            f'ip={self.ap_ip} nat={self.enable_nat}'
        )
        self._publish_status('Wi-Fi AP manager initialised; waiting to start')

        if self.enable_ap:
            self._start_timer = self.create_timer(0.5, self._on_start)
        else:
            self.get_logger().info('AP disabled by parameter enable_ap=false')
            self._publish_status('Wi-Fi AP disabled by configuration')

    def _on_start(self) -> None:
        if self._start_timer:
            self.destroy_timer(self._start_timer)
            self._start_timer = None
        self.start_ap()

    def start_ap(self) -> None:
        """Configure interface, start hostapd/dnsmasq, and advertise services."""
        if self.dry_run:
            self.get_logger().info('AP start (dry_run): skipping system configuration')
            self._publish_status('Dry-run: Wi-Fi AP setup skipped')
            self._start_zeroconf()
            return

        if os.geteuid() != 0:
            self.get_logger().warn('AP configuration requires root privileges; run the service as root.')

        self._publish_status('Starting Wi-Fi access point setup')
        self._snapshot_interface_state()
        self._configure_interface()

        hostapd_ok = self._launch_hostapd()
        dnsmasq_ok = self._launch_dnsmasq()
        nat_ok = True
        if self.enable_nat:
            nat_ok = self._enable_nat()
        self._start_zeroconf()

        if hostapd_ok and dnsmasq_ok:
            parts = [f'SSID {self.ssid}', f'IP {self.ap_ip}']
            if self.passphrase and len(str(self.passphrase)) >= 8:
                parts.append('WPA2 secured')
            else:
                parts.append('open network')
            if self.enable_nat:
                parts.append(f'NAT via {self.internet_iface}' + (' ready' if nat_ok else ' failed'))
            status_msg = 'Wi-Fi AP ready: ' + ', '.join(parts)
        else:
            degraded = []
            if not hostapd_ok:
                degraded.append('hostapd missing')
            if not dnsmasq_ok:
                degraded.append('dnsmasq missing')
            if self.enable_nat and not nat_ok:
                degraded.append('NAT failed')
            status_msg = 'Wi-Fi AP degraded: ' + '; '.join(degraded) if degraded else 'Wi-Fi AP setup incomplete'
        self._publish_status(status_msg)

    def _snapshot_interface_state(self) -> None:
        if self.dry_run:
            return
        try:
            result = subprocess.run(
                ['ip', '-j', 'addr', 'show', self.iface],
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode == 0 and result.stdout:
                self._iface_state_file.write_text(result.stdout)
        except Exception as exc:  # pragma: no cover - depends on system utilities
            self.get_logger().warn(f'Failed to snapshot interface state: {exc}')

    def _configure_interface(self) -> None:
        if self.dry_run:
            return
        for cmd in (
            ['ip', 'link', 'set', self.iface, 'down'],
            ['ip', 'addr', 'flush', 'dev', self.iface],
            ['ip', 'addr', 'add', self.ap_ip, 'dev', self.iface],
            ['ip', 'link', 'set', self.iface, 'up'],
        ):
            self._run(cmd, check=False)
        self.get_logger().info(f'Configured {self.iface} with {self.ap_ip}')

    def _launch_hostapd(self) -> bool:
        if self.dry_run:
            return True
        hostapd = _which('hostapd')
        if not hostapd:
            self.get_logger().error('hostapd not found on PATH; cannot provide Wi-Fi AP')
            return False

        cfg_lines = [
            f'interface={self.iface}',
            f'ssid={self.ssid}',
            'driver=nl80211',
            'hw_mode=g',
            f'channel={self.channel}',
            f'country_code={self.country_code}',
            'ieee80211n=1',
            'wmm_enabled=1',
            'ignore_broadcast_ssid=0',
        ]
        if self.passphrase and len(str(self.passphrase)) >= 8:
            cfg_lines += [
                'auth_algs=1',
                'wpa=2',
                'wpa_key_mgmt=WPA-PSK',
                f'wpa_passphrase={self.passphrase}',
                'rsn_pairwise=CCMP',
            ]
        else:
            cfg_lines += ['auth_algs=1', 'wpa=0']

        self._hostapd_conf_path.write_text('\n'.join(cfg_lines) + '\n')
        try:
            self._hostapd_proc = subprocess.Popen(
                [hostapd, '-d', str(self._hostapd_conf_path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.get_logger().info(f'hostapd launched for SSID "{self.ssid}"')
            return True
        except Exception as exc:  # pragma: no cover - depends on system binaries
            self.get_logger().error(f'Failed to start hostapd: {exc}')
            return False

    def _launch_dnsmasq(self) -> bool:
        if self.dry_run:
            return True
        dnsmasq = _which('dnsmasq')
        if not dnsmasq:
            self.get_logger().error('dnsmasq not found on PATH; DHCP will be unavailable')
            return False

        ip = self.ap_ip.split('/')[0]
        cfg_lines = [
            f'interface={self.iface}',
            'bind-interfaces',
            'no-resolv',
            'domain-needed',
            'bogus-priv',
            f'dhcp-range={self.dhcp_range},{self.dhcp_lease_time}',
            f'dhcp-option=3,{ip}',
            f'dhcp-option=6,{ip}',
            f'dhcp-leasefile={self._dnsmasq_lease_path}',
            f'address=/{self.mdns_name}/{ip}',
        ]
        self._dnsmasq_conf_path.write_text('\n'.join(cfg_lines) + '\n')
        try:
            self._dnsmasq_proc = subprocess.Popen(
                [dnsmasq, '--keep-in-foreground', '--conf-file', str(self._dnsmasq_conf_path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.get_logger().info(f'dnsmasq launched on {self.iface} to provide DHCP/DNS')
            return True
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Failed to start dnsmasq: {exc}')
            return False

    def _enable_nat(self) -> bool:
        if self.dry_run:
            return True
        self._ip_forward_original = self._get_sysctl('net.ipv4.ip_forward')
        self._set_sysctl('net.ipv4.ip_forward', '1')
        rules = [
            ['iptables', '-t', 'nat', '-A', 'POSTROUTING', '-o', self.internet_iface, '-j', 'MASQUERADE'],
            ['iptables', '-A', 'FORWARD', '-i', self.internet_iface, '-o', self.iface, '-m', 'state', '--state', 'RELATED,ESTABLISHED', '-j', 'ACCEPT'],
            ['iptables', '-A', 'FORWARD', '-i', self.iface, '-o', self.internet_iface, '-j', 'ACCEPT'],
        ]
        applied = []
        for rule in rules:
            if self._run(rule, check=False) == 0:
                applied.append(rule)
            else:
                self.get_logger().warn(f'Failed to apply NAT rule: {" ".join(rule)}')
        self._nat_rules = applied
        state = {'rules': applied, 'ip_forward': self._ip_forward_original}
        try:
            self._nat_state_file.write_text(json.dumps(state))
        except Exception:
            pass
        if applied:
            self.get_logger().info(f'Enabled NAT to share {self.internet_iface} with Wi-Fi clients')
            return True
        self.get_logger().warn('No NAT rules were applied; internet sharing disabled')
        return False

    def _disable_nat(self) -> None:
        if self.dry_run:
            return
        stored_ip_forward = None
        rules = self._nat_rules
        if (not rules) and self._nat_state_file.exists():
            try:
                data = json.loads(self._nat_state_file.read_text())
            except Exception:  # pragma: no cover - unexpected parse failure
                data = None
            if isinstance(data, dict):
                stored_ip_forward = data.get('ip_forward')
                rules = data.get('rules', [])
            elif isinstance(data, list):
                rules = data
        if self._ip_forward_original is None and stored_ip_forward is not None:
            self._ip_forward_original = stored_ip_forward
        for rule in reversed(rules or []):
            delete_rule = list(rule)
            if '-A' in delete_rule:
                delete_rule[delete_rule.index('-A')] = '-D'
            self._run(delete_rule, check=False)
        if self._ip_forward_original is not None:
            self._set_sysctl('net.ipv4.ip_forward', self._ip_forward_original)
        elif self.enable_nat:
            self._set_sysctl('net.ipv4.ip_forward', '0')

    def _start_zeroconf(self) -> None:
        if self.dry_run:
            return
        if not Zeroconf:
            self.get_logger().warn('zeroconf package missing; mDNS advertisement disabled')
            return
        if self._zeroconf:
            return
        ip = self.ap_ip.split('/')[0]
        addr = socket.inet_aton(ip)
        try:
            zc = Zeroconf()
        except Exception as exc:  # pragma: no cover - depends on zeroconf install
            self.get_logger().warn(f'Failed to initialise zeroconf: {exc}')
            return
        hostname = self.mdns_name
        infos: List[Any] = []
        try:
            http_info = ServiceInfo(
                '_http._tcp.local.',
                f'{hostname}._http._tcp.local.',
                addresses=[addr],
                port=self.http_port,
                properties={b'path': b'/'},
                server=f'{hostname}.local.',
            )
            zc.register_service(http_info)
            infos.append(http_info)
            ws_info = ServiceInfo(
                '_ws._tcp.local.',
                f'{hostname}._ws._tcp.local.',
                addresses=[addr],
                port=self.websocket_port,
                properties={},
                server=f'{hostname}.local.',
            )
            zc.register_service(ws_info)
            infos.append(ws_info)
            self._zeroconf = zc
            self._zeroconf_infos = infos
            self.get_logger().info(f'mDNS registered for {hostname} at {ip}')
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Failed to register zeroconf services: {exc}')
            try:
                zc.close()
            except Exception:
                pass

    def destroy_node(self) -> bool:
        self._cleanup()
        return super().destroy_node()

    def _cleanup(self) -> None:
        if self._zeroconf:
            try:
                for info in self._zeroconf_infos:
                    try:
                        self._zeroconf.unregister_service(info)
                    except Exception:
                        pass
                self._zeroconf.close()
            except Exception:
                pass
            self._zeroconf = None
            self._zeroconf_infos = []

        for proc in (self._dnsmasq_proc, self._hostapd_proc):
            if proc:
                try:
                    proc.terminate()
                    proc.wait(timeout=5)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
        self._dnsmasq_proc = None
        self._hostapd_proc = None

        self._disable_nat()
        self._restore_interface_state()
        self._publish_status('Wi-Fi AP stopped and network state restored')

    def _restore_interface_state(self) -> None:
        if self.dry_run:
            return
        if not self._iface_state_file.exists():
            self._run(['ip', 'addr', 'flush', 'dev', self.iface], check=False)
            self._run(['ip', 'link', 'set', self.iface, 'down'], check=False)
            return
        try:
            data = json.loads(self._iface_state_file.read_text())
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Failed to parse interface state: {exc}')
            data = []
        self._run(['ip', 'addr', 'flush', 'dev', self.iface], check=False)
        oper_up = False
        for entry in data:
            if entry.get('operstate') == 'UP':
                oper_up = True
            for addr in entry.get('addr_info', []):
                if addr.get('family') != 'inet':
                    continue
                local = addr.get('local')
                prefix = addr.get('prefixlen')
                if local and prefix is not None:
                    self._run(['ip', 'addr', 'add', f'{local}/{prefix}', 'dev', self.iface], check=False)
        self._run(['ip', 'link', 'set', self.iface, 'up' if oper_up else 'down'], check=False)

    def _run(self, cmd: List[str], check: bool = True) -> int:
        try:
            self.get_logger().debug(f'Running command: {" ".join(cmd)}')
            result = subprocess.run(cmd, check=check)
            return result.returncode
        except subprocess.CalledProcessError as exc:
            self.get_logger().warn(f'Command failed ({exc.returncode}): {" ".join(exc.cmd)}')
            return exc.returncode
        except FileNotFoundError:
            self.get_logger().error(f'Command not found: {cmd[0]}')
            return 127
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Command {cmd} raised {exc}')
            return 1

    def _get_sysctl(self, key: str) -> Optional[str]:
        try:
            result = subprocess.run(['sysctl', '-n', key], capture_output=True, text=True, check=False)
            if result.returncode == 0:
                return result.stdout.strip()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Failed to read sysctl {key}: {exc}')
        return None

    def _set_sysctl(self, key: str, value: str) -> None:
        try:
            self._run(['sysctl', f'{key}={value}'], check=False)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Failed to set sysctl {key}: {exc}')

    def _publish_status(self, message: str) -> None:
        msg = String()
        msg.data = message
        self._status_pub.publish(msg)
        self.get_logger().debug(f'Status update: {message}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WifiAccessPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover - CLI entry point
    main()
