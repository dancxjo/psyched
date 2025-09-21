# Pilot WebSocket Node

This package provides the Pilot WebSocket bridge that receives UI commands and publishes them onto ROS topics.

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
