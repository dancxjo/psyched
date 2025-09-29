# Ear

PyAudio capture, WebRTC VAD, and streaming transcription for ROS 2.

The ear package exposes a small set of executables that form the audio intake
pipeline:

- `ear_node` – streams raw PCM16 audio from a PyAudio device to `/audio/raw`.
- `silence_node` – monitors `/audio/raw` and publishes a silence-duration gauge
  on `/audio/silence_ms`.
- `vad_node` – resamples `/audio/raw` to 16 kHz, applies WebRTC VAD, and emits
  tagged frames on `/audio/vad_frames` using `psyched_msgs/msg/VadFrame`.
- `segmenter_node` – subscribes to `/audio/vad_frames`, rebuilds contiguous
  speech segments on `/audio/speech_segment`, and streams an in-progress buffer
  on `/audio/speech_segment_accumulating` while maintaining the
  `/audio/speech_duration` gauge.
- `segment_accumulator_node` – listens for completed segments and stitches them
  into a longer rolling context on `/audio/speech_accumulating` for the remote
  ASR tiers.
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
- `vad_mode` (int, default `3`): WebRTC aggressiveness level (0–3).
- `target_sample_rate` (int, default `16000`): Resampling rate used for VAD.
- `frame_duration_ms` (int, default `30`): Frame length forwarded to the
  segmenter.
- `input_topic` (string, default `/audio/raw`): Source of raw PCM audio.
- `frame_topic` (string, default `/audio/vad_frames`): Destination for tagged
  `psyched_msgs/msg/VadFrame` messages.

### `segmenter_node`
- `frame_topic` (string, default `/audio/vad_frames`): VAD frame source.
- `segment_topic` (string, default `/audio/speech_segment`): Completed segment
  output.
- `accumulating_topic` (string, default `/audio/speech_segment_accumulating`):
  Rolling buffer output while speech is active.
- `duration_topic` (string, default `/audio/speech_duration`): Speech duration
  gauge.

### `segment_accumulator_node`
- `segment_topic` (string, default `/audio/speech_segment`): Upstream segment
  source.
- `accum_topic` (string, default `/audio/speech_accumulating`): Rolling context
  output for long-form ASR.
- `reset_timeout` (float, default `12.0`): Maximum silence between segments
  before the buffer is cleared.
- `max_segments` (int, default `8`): Hard limit on the number of segments to
  retain before resetting.

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
