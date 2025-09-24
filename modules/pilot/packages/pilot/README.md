# Pilot WebSocket Node

This package provides the Pilot WebSocket bridge that receives UI commands and publishes them onto ROS topics.

## Wireless access point helper

The pilot module can optionally configure a spare wireless interface as an access
point so that tablets or other robots can join an ad-hoc DDS network without any
external infrastructure. The helper node wraps `hostapd`, `dnsmasq`, and Python
`zeroconf` to provide Wi-Fi, DHCP, and mDNS discovery respectively.

- Enable or disable the helper via `PILOT_ENABLE_AP` (defaults to `true`).
- Point `PILOT_AP_INTERFACE` at the wireless interface reserved for hotspot
  duties (for example `wlan1`).
- Adjust SSID, passphrase, subnet, DHCP range, and advertised mDNS hostname via
  the corresponding `PILOT_AP_*` variables in `hosts/<host>/config/pilot.env`.
- Set `PILOT_AP_DRY_RUN=true` during development to exercise the node without
  modifying host networking state.

## Volume control

- Web UI volume changes are forwarded as `std_msgs/Float32` on the topic `/voice/volume` for the voice node to adjust synthesis/playback gain.
- Additionally, the Pilot node attempts to set the system output volume via ALSA `amixer` when present. This adjusts typical mixer controls (`Master`, `Speaker`, `PCM`, `Headphone`, `Digital`, `Playback`). It tries device variants `default` and `pulse` if necessary.
- Values are interpreted as:
  - `0.0` to `1.0` map to `0%` to `100%` in ALSA.
  - Values above `1.0` will still set ALSA to `100%`; extra gain is handled inside the voice node.

### Dependencies

- `alsa-utils` for the `amixer` command. Install on Debian/Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y alsa-utils
```

### Notes

- If your system uses PulseAudio/PipeWire, the `amixer -D pulse` path is also attempted. If none of the controls are found, the node will log available controls from `amixer scontrols` to aid debugging.
- You can override playback device for TTS playback by setting `ALSA_PCM` or `PULSE_SERVER` environment variables in the voice node environment.
