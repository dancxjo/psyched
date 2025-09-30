# Psyched – Agent Guide

This handbook is your quick-start reference when contributing to the
**psyched** workspace. It consolidates prior guides and reflects the
current architecture.

---

## Instruction Precedence
- System → developer → user → this guide → in-source comments.
- Before editing, check for a more specific `AGENTS.md` deeper in the tree.
- Keep this guide updated whenever you discover workflow quirks, fragile scripts,
  or best practices.

---

## Repository Orientation
- `modules/` – Canonical home for all modules. Each module defines:
  - `module.toml` (actions, dependencies, systemd entries).
  - `packages/` (ROS 2 packages, colcon-built).
  - `launch/*.launch.py` (ROS 2 launch files).
- `hosts/` – Host-specific configuration.
  - `config/<module>.toml` → overrides launch arguments.
  - `systemd/*.service` → unit files managed by `psh systemd`.
- `tools/` – Provisioning helpers (ROS/Docker installers, env shims).
- `compose/` – Docker Compose stacks (e.g. `speech-stack.compose.yml`).
- `src/` – Workspace symlinks created during builds (remove before provisioning).
- `packages/` (legacy) – Only used for older packages; new work belongs in `modules/*/packages/`.

---

## System Architecture

### Cerebellum (Motherbrain)
- Hardware: Raspberry Pi 4/5 mounted on iRobot Create 1 base.
- Runs ROS 2 Jazzy/Kilted.
- Manages sensors/actuators and real-time loops.
- Modules: `ear`, `voice`, `chat`, `pilot`, `foot`, `imu`, `eye`, `gps`, `wifi`, `will`.

### Forebrain
- Hardware: headless laptop with GPU, also mounted onboard.
- Runs ASR/LLM/TTS containers (no ROS 2).
- Provisioning installs Docker, CUDA toolkit, and speech assets.
- Invoked with `psh speech up`.
- Offloads heavy compute from cerebellum.
- Planned integration with **Neo4j** (graph memory) and **Qdrant** (vector memory).

---

## Working Style
- Restate tasks and outline a plan before modifying files.
- Use BDD/TDD where practical: capture expectations before implementing.
- Additive commits; keep messages concise and meaningful.
- Keep dependencies cached when possible.
- Fix warnings as you go.
- When editing ROS launch files, wrap `LaunchConfiguration` values in
  `ParameterValue` with the appropriate type so nodes receive correctly-typed
  parameters instead of strings.

---

## Coding Standards
- Python 3.12; type annotations preferred.
- ROS 2 packages use `ament_python`.
- Avoid `try/except` import guards; handle optional deps at call sites.
- Add docstrings and usage examples for public APIs.
- For Deno/TypeScript (`psh`), follow existing CLI patterns.

---

## Build & Launch

### Provisioning
```bash
rm -rf ./src
psh provision cerebellum
psh install
psh systemd enable
````

### Running Core Stack

```bash
# On forebrain
psh speech up   # ASR/TTS/LLM containers

# On cerebellum
psh launch ear
psh launch voice
psh launch chat
psh launch pilot
```

### Pilot UI

Visit `http://<cerebellum-host>:8080`.

- When adding or updating Pilot controls, use the shared `.control-surface`,
  `.metric-grid`, and `.metric-card` CSS helpers to keep styling consistent
  across panels.

---

## CLI Reference (`psh`)

* `psh install` → system dependencies (apt, pip/uv).
* `psh provision <host>` → apply host configs.
* `psh models` → manage Ollama models.
* `psh speech up|down` → manage Docker ASR/TTS/LLM stack.
* `psh launch <module>` → run module launch file.
* `psh systemd enable|disable` → manage units.
* `psh build` → `colcon build` with symlink install.
* `psh clean` → cleanup build artifacts/systemd leftovers.
* `psh test` → run module/unit tests.

---

## Testing & Validation

* Prefer targeted tests (`pytest`, `colcon test`) over full builds.
* Module smoke tests:

  ```bash
  psh launch ear   # check transcription
  psh launch voice # check TTS output
  psh launch pilot # web UI health
  ```
* Pilot backend unit tests rely on FastAPI and related dependencies. Set
  `PYTHONPATH=modules/pilot/packages/pilot` before running `pytest` so the
  `pilot` package resolves without installation.
* Deno tests: run with `DENO_TLS_CA_STORE=system` and explicit permissions.
* Pilot backend tests require `fastapi`, `httpx`, `uvicorn`.

---

## Common Issues

* **Network dependencies**:

  * Ollama models (ollama.com) may fail behind firewall.
  * Piper TTS models (huggingface.co) require access.
  * ROS 2 installation may fail if GitHub API blocked.
  * Development containers may not include the Deno runtime by default; install Deno locally to run `psh` CLI tests.
* **Hardware deps**:

  * `alsa-utils` (for `arecord`), `webrtcvad`, `faster-whisper` for `ear`.
  * `libusb-1.0-dev` for `eye`.
  * Kinect and Create 1 hardware required for full bring-up.
* **Workarounds**:

  * Develop modules in isolation when offline.
  * Use local symlink installs (`colcon build --symlink-install`) to avoid duplicate packages.
  * `tools/with_ros_env.sh` sources ROS 2 before colcon/ros2 invocations.
  * Rust microservices such as `forebrain-llm` sit outside the root Cargo workspace; copy the crate to `/tmp` (or set
    `CARGO_TARGET_DIR`) before running `cargo test` so the workspace manifest does not block the build.
  * Remote ASR tiers may emit placeholder transcripts like `samples=<n> sum=<m>` when the fast/mid pipelines are misconfigured;
    let the ear module fall back to onboard Whisper when that happens.
  * The speech stack now loads whisper.cpp models from `asr-service/models/` (e.g. `ggml-tiny.en.bin`). Run
    `tools/download_speech_models.sh` and ensure the compose volume mounts that directory before bringing the stack up.

---

## Git Etiquette

* Stay on default branch unless told otherwise.
* Don’t amend; add new commits.
* Keep tree clean before commits: format, lint, ensure tests pass.

---

## Module Notes

* `ear`: PyAudio microphone capture, VAD, Whisper transcription. Tiered
  transcribers now live in `transcriber_*_node.py` and share helpers under
  `transcription_*.py`; prefer extending those utilities instead of creating
  bespoke threading logic.
* `voice`: espeak-ng or Coqui/Piper via WebSocket, with playback.
* `chat`: connects transcription → Ollama LLM → voice. Exposes a `pilot_base_url`
  parameter that should point at the running Pilot backend so the chat system
  prompt can include the text-only control surface summary. The Forebrain
  websocket LLM is currently paused; the node always calls Ollama over HTTP via
  `ollama_host`.
* `pilot`: LCARS-style web frontend + health reporter.
* `nav`: depth-to-scan pipeline + AMCL, vision LLM prompts.
* Others (`eye`, `foot`, `imu`, `gps`, `wifi`, `will`) follow similar structure.

---

## Status

* ✅ ROS 2 stack runs on Pi (cerebellum).
* ✅ GPU laptop (forebrain) runs heavy AI services.
* ✅ `psh` orchestrates installs, builds, launches, and services.
* 🔄 Graph + vector memory integration in progress.

## Provisioning Notes
* Speech provisioning leaves a sentinel file at `setup/.speech_setup_complete`.
  When adding new required models or steps bump the version written in
  `psh/speech_setup.ts` so hosts rerun the updated setup.
* Use `psh models` to manage Ollama models; ensure required models are listed in
  `psh/speech_setup.ts`.
