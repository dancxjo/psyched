# Ear module guidelines

- Provide comprehensive type annotations and doctrings for public APIs.
- Run `PYTHONPATH=modules/ear/packages/ear:$PYTHONPATH pytest modules/ear/packages/ear/tests` after modifying Python code in this module.
- Prefer backend-agnostic abstractions so additional transcription backends can plug in easily.
- Keep shell scripts POSIX-friendly with `#!/usr/bin/env bash` and `set -euo pipefail`.
- Use `ros2 launch ear ear.launch.py` (via `launch_unit.sh`) to run the full audio pipeline; individual nodes (`ear_audio_capture`, `ear_vad`, `ear_silence`, `ear_transcriber`) can be started with `ros2 run` for debugging.
- Ensure `python3-webrtcvad` is installed on the host (automatically provisioned via `psh mod setup ear`) so the VAD node can start successfully.
- Node-level tests under `tests/test_nodes.py` depend on `rclpy`; install ROS 2 dependencies or skip them explicitly when the runtime is unavailable.
