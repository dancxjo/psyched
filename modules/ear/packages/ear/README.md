# Ear

PyAudio capture, WebRTC VAD, and streaming transcription for ROS 2.

The ear package exposes four executables that form the audio intake pipeline:

- `ear_node` – streams raw PCM16 audio from a PyAudio device to `/audio/raw`.
- `silence_node` – monitors `/audio/raw` and publishes a silence-duration gauge
  on `/audio/silence_ms`.
- `vad_node` – resamples `/audio/raw` to 16 kHz, applies WebRTC VAD, and emits
  voiced frames on `/audio/speech_segment` while tracking active speech
  duration on `/audio/speech_duration`.
- `transcriber_node` – batches `/audio/speech_segment` messages and invokes a
  Whisper-compatible backend (prefers [`faster-whisper`](https://github.com/SYSTRAN/faster-whisper)) to publish
  recognized utterances on `/audio/transcription`.

## Parameters

### `ear_node`
- `device_id` (int, default `0`): PyAudio device index.
- `sample_rate` (int, default `44100`): Capture rate in Hz.
- `channels` (int, default `1`): Channel count.
- `chunk_size` (int, default `1024`): Frames per callback.

### `silence_node`
- `silence_threshold` (float, default `500.0`): RMS threshold used to reset the
  silence gauge.

### `vad_node`
- No runtime parameters; consumes `/audio/raw` and emits `/audio/speech_segment`
  at 16 kHz PCM16.

### `transcriber_node`
- `segment_topic` (string, default `/audio/speech_segment`): Source for voiced
  audio.
- `transcript_topic` (string, default `/audio/transcription`): Destination for
  `psyched_msgs/msg/Transcript` outputs.
- `speaker` (string, default `user`): Speaker label to annotate each
  transcription.
- `segment_sample_rate` (int, default `16000`): Sample rate of the incoming
  segments.
- `model` (string, default `base`): Whisper model variant to load.
- `device` (string, default `cpu`): Execution device passed to the backend.
- `compute_type` (string, default `int8`): Precision hint forwarded to
  `faster-whisper`.
- `language` (string, optional): Force a language code instead of automatic
  detection.
- `beam_size` (int, default `5`): Beam search width for decoding.

## Launch

```bash
ros2 launch ear ear.launch.py \
    ear_device_id:=0 \
    ear_sample_rate:=44100 \
    ear_channels:=1 \
    ear_chunk_size:=1024 \
    silence_threshold:=500.0
```

Add the transcriber to an existing launch description or run it manually:

```bash
ros2 run ear transcriber_node --ros-args -p model:=small -p device:=cpu
```

## Notes

- The transcriber selects `faster-whisper` when available and falls back to the
  reference `whisper` package. Install at least one of them to enable
  transcription.
- All topics use little-endian PCM16 audio, matching the VAD output format.
- `psyched_msgs/msg/Transcript` includes the recognized text, speaker label, and
  a coarse confidence score derived from Whisper log probabilities.
