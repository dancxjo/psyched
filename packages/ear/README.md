# Ear

Microphone PCM publisher for ROS 2. Captures raw S16_LE audio from ALSA via `arecord` and publishes `audio_common_msgs/AudioData` on a configurable topic (default `/audio/pcm`).

## Parameters
- `topic` (string): topic name for `audio_common_msgs/AudioData` (default `/audio/pcm`)
- `device` (string): ALSA capture device (default `default`)
- `rate` (int): sample rate in Hz (default `16000`)
- `channels` (int): channels (1=mono, 2=stereo) (default `1`)
- `chunk` (int): bytes per message read from arecord (default `2048`)

## Launch
```bash
ros2 launch ear ear.launch.py topic:=/audio/pcm device:=default rate:=16000 channels:=1 chunk:=2048
```

## Notes
- Requires `alsa-utils` for the `arecord` binary.
- Message type is `audio_common_msgs/msg/AudioData` (byte array of raw PCM S16_LE).
