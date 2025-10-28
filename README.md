# Psyched

Psyched orchestrates Pete's modular robot stack: a ROS 2 workspace running on the robot ("motherbrain"), remote cognition services ("forebrain"), and a cockpit UI for operators. The repository ships the ROS 2 packages, cognitive agents, and the Deno-based `psh` CLI that provisions and coordinates everything.

## Stack Overview
- **Modular ROS 2 workspace** - each capability lives in `modules/<name>` with a `module.toml` manifest, lifecycle scripts, and ROS packages under `packages/`.
- **Shared cognition services** - speech, LLM, and memory backends run as containerised services in `services/<name>` and can be hosted on dedicated machines.
- **Unified orchestration** - the `psh` CLI manages host provisioning, workspace builds, runtime lifecycle, Cockpit UI assets, and systemd integration using declarative host manifests.

## Architecture
| Layer | Responsibilities | Typical host |
| --- | --- | --- |
| `motherbrain` | Runs ROS 2 nodes for sensors, actuation, cockpit bridge, and local cognition modules. | Pete's SBC / Raspberry Pi |
| `forebrain` | Hosts ASR, TTS, LLM, Neo4j, and Qdrant services consumed by the robot. | GPU workstation or cloud VM |
| Cockpit | Browser-based UI served by the `cockpit` module and WebSocket bridge to ROS topics. | Any operator device |
| Control plane | `psh` CLI, bootstrap scripts, and tests used from a developer workstation. | Developer laptop |

## Repository Tour
```
config/                # Shared configuration (ROS domain ID, etc.)
docs/                  # Operational guides (host manifests, systemd, docker workflows)
env/psyched_env.sh     # Shell helpers for sourcing ROS/colcon workspaces
hosts/*.toml           # Host manifests enumerating installers, services, and modules
modules/<name>/        # ROS 2 modules + Cockpit panels
packages/psyched_msgs/ # Shared ROS message definitions
services/<name>/       # Container stacks for ASR, TTS, LLM, memory, ROS dev shell
tests/                 # Shell and Python regression tests for bootstrap scripts and helpers
tools/bootstrap/       # Host bootstrap helper scripts invoked by ./setup
tools/psh/             # Deno CLI entrypoint, library code, and tests
work/                  # Colcon workspace populated by psh (src/build/install/log)
```

## Core Modules
| Module | What it provides |
| --- | --- |
| `cockpit` | Teleoperation dashboard, ROS <-> WebSocket bridge, and static cockpit frontend. |
| `conversant` | Turn-taking dialog manager with optional Ollama-backed concern handling. |
| `ear` | Audio capture, WebRTC VAD, and streaming transcription via the ASR service. |
| `eye` | Kinect RGB-D integration using patched `kinect_ros2` and libfreenect build helpers. |
| `faces` | Face detection/embedding publisher and message definitions for perception pipelines. |
| `foot` | iRobot Create 1 drive-base interface using create_robot and libcreate overlays. |
| `gps` | GPSD provisioning scripts and `ublox_gps` bridge for navigation telemetry. |
| `hypothalamus` | DHT environmental sensing over I2C using Adafruit CircuitPython drivers. |
| `imu` | MPU6050 inertial sensing with Madgwick filtering and ROS 2 control integration. |
| `memory` | ROS nodes and services that persist/recall events via Qdrant vectors and Neo4j graphs. |
| `nav` | Navigation2 overlays, launch recipes, and cockpit panel wiring. |
| `pilot` | Cognitive integrator that composes sensor impressions, LLM prompts, and memory actions. |
| `viscera` | Host health monitoring, mood heuristics, and cockpit telemetry feeds. |
| `voice` | WebSocket TTS publisher targeting the Coqui service backend. |

Each module exposes lifecycle scripts (`launch_unit.sh`, `shutdown_unit.sh`), optional Cockpit panels, and a manifest describing apt/pip/git dependencies.

## Platform Services
| Service | Purpose | Notes |
| --- | --- | --- |
| `services/asr` | Rust WebSocket server around Whisper GGML models. | `setup.sh` downloads checkpoints; listens on `ws://*:5003/asr`. |
| `services/tts` | Coqui TTS WebSocket wrapper for streaming synthesis. | `setup.sh` pulls Piper voice assets; exposes `ws://*:5002/tts`. |
| `services/llm` | GPU-enabled Ollama runtime and model bootstrap. | `setup.sh` ensures requested model cache. |
| `services/graphs` | Neo4j database for relational memory. | Ports 7474 (HTTP) and 7687 (Bolt). |
| `services/vectors` | Qdrant vector database for memory retrieval. | Ports 6333 (HTTP) and 6334 (gRPC). |
| `services/ros2` | Optional ROS 2 development container with workspace mounts. | Provides an interactive shell via `psh svc shell`. |

## Tooling Highlights
- `psh` (`tools/psh/main.ts`) is a Deno CLI that:
  - provisions hosts defined in `hosts/*.toml` (`psh host setup`),
  - synchronises module assets (`psh mod setup`, `psh mod up|down`),
  - builds the colcon workspace (`psh build`, `psh clean`),
  - manages services (`psh svc setup|up|down|shell`), and
  - renders Cockpit control metadata.
- Bootstrap scripts in `tools/bootstrap/` and `tools/provision/` install ROS, Docker, CUDA, Deno, and add profile snippets alongside reboot guards.
- `env/psyched_env.sh` exposes `psyched::activate` helpers to source `/opt/ros/<distro>` and the local workspace consistently.
- Regression tests in `tests/` validate bootstrap logic, environment helpers, and module launch arguments; run them after modifying scripts.

## Getting Started

### Requirements
- Ubuntu 22.04+ host with `sudo`, systemd, and a functional network connection.
- Docker installed for cognition services (the `./setup` script installs/updates it).
- Adequate disk space for ROS dependencies, Whisper/Coqui/Ollama models, and colcon builds.

### Bootstrap the repository
```bash
git clone https://github.com/dancxjo/psyched.git
cd psyched
./setup
```
`./setup` installs core packages, Deno, the `psh` wrapper, configures mDNS, and primes the CLI cache. Open a new shell (or `source ~/.bashrc`) afterwards so the environment picks up the new PATH entries.

### Provision hosts and modules
1. Edit `hosts/*.toml` if you need to tailor installers, services, or environment variables.
2. Run `psh host setup motherbrain` (robot) and `psh host setup forebrain` (inference) to apply installers and service prerequisites.
3. Prepare module assets on each host with `psh mod setup <module>`. The cockpit module must be set up anywhere you serve the UI.
4. Build the ROS workspace: `psh build` (or `psh build --packages-select <pkg>` for targeted builds).
5. Launch workloads:
   - Modules: `psh up cockpit foot ear` (stop with `psh down cockpit`).
   - Services: `psh svc setup tts`, `psh svc up tts`, `psh svc up llm`, etc.
6. Visit the cockpit via `http://<motherbrain-host>:8088/` once the `cockpit` module reports ready.

### Local development workflow
- Activate the environment: `source env/psyched_env.sh` then `psyched::activate` (append `--workspace-only` or `--ros-only` as needed).
- Build ROS packages manually if desired: `colcon build --packages-select cockpit`.
- Regenerate cockpit frontend assets through the module setup scripts; the frontend is bundled inside `modules/cockpit/frontend`.
- Use `psh clean` to rebuild the `work/` tree while preserving connectivity-critical modules.
- When testing new modules, keep them isolated - avoid cross-module imports outside the shared `packages/psyched_msgs`.

### Tests and quality gates
- CLI and tooling: `cd tools/psh && deno fmt && deno lint && deno task test`.
- ROS nodes: `colcon test --packages-select <pkg>` after building.
- Shell logic: `./tests/*.sh` and `shellcheck modules/**/launch_*.sh modules/**/shutdown_*.sh setup`.
- Services: run their `setup.sh` to validate model downloads before `psh svc up`.

## Configuration and docs
- `config/ros_domain_id` sets the ROS 2 domain ID shared across hosts.
- `docs/host-manifests.md` explains the manifest schema; `docs/psh-sys.md` covers systemd integration; `docs/docker.md` captures container workflows.
- `AGENTS.md` documents collaborative development guardrails and expectations for agents or humans touching the stack.

## License
Released under the [MIT License](LICENSE).
