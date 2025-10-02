# psyched

Psyched is Pete Rizzlington’s modular robotics stack: a ROS 2 workspace, a collection of hardware-specific modules, and a web-based “pilot” console that lets you drive a robot from any browser. The repo doubles as a reference for building your own robot—swap out the hardware-specific bits, keep the orchestration.

## Table of contents

- [Project overview](#project-overview)
- [Quick start](#quick-start)
- [Repository layout](#repository-layout)
- [Modules](#modules)
	- [Pilot](#pilot)
	- [IMU](#imu)
	- [Foot](#foot)
- [Services](#services)
	- [TTS](#tts)
	- [Language](#language)
	- [Graphs](#graphs)
	- [Vectors](#vectors)
	- [ASR](#asr)
- [Development workflows](#development-workflows)
	- [Rust + ROS backend](#rust--ros-backend)
	- [Pilot frontend](#pilot-frontend)
	- [Using the `psh` CLI](#using-the-psh-cli)
- [Testing & validation](#testing--validation)
- [Troubleshooting](#troubleshooting)
- [Roadmap](#roadmap)
- [License](#license)

## Project overview

Psyched is organized around three ideas:

1. **Composable modules.** Each hardware or capability lives in `modules/<name>` with declarative metadata (`module.toml`) and lifecycle scripts (`launch_*`, `shutdown_*`). Modules can surface UI controls inside the pilot console without modifying the core frontend.
2. **Containerised services.** Cross-cutting capabilities (speech stacks, perception pipelines, etc.) live in `services/<name>` alongside a `service.toml` manifest and Docker Compose stack. They boot via `psh svc ...` and share helper assets under `tools/`.
3. **A ROS 2 + Rust bridge.** The `pilot` crate (linked into `work/src/pilot`) exposes a websocket API (`ws://<host>:8088/ws`) that forwards cockpit messages into ROS topics using [`rclrs`](https://github.com/sequenceplanner/rclrs).
4. **A modern pilot UI.** The `modules/pilot/frontend` package uses [Deno](https://deno.land/) and [Fresh](https://fresh.deno.dev/) with Preact hooks. It consumes the cockpit websocket via a reusable client in `lib/cockpit.ts`.

Supporting utilities live under `tools/` and the `psh` CLI: a Rust binary that provisions hosts, orchestrates modules, and maintains the Deno symlinks required by the pilot.

## Quick start

### 1. Clone & bootstrap

```bash
git clone https://github.com/dancxjo/psyched.git
cd psyched
./setup  # installs system dependencies, builds psh, and drops you into psh setup
```

`./setup` installs build toolchains (Rust, Python, ROS dependencies), configures mDNS, compiles the `psh` helper, and runs `psh setup` to execute the host-specific bootstrap scripts defined in `hosts/*.toml`.

### 2. Provision additional machines (optional)

```bash
psh host setup motherbrain
psh host setup forebrain
```

Each host file lists shell scripts under `tools/bootstrap/` (or `tools/provision/`) to install ROS 2, CUDA, Docker, and Deno.

Host configs can also declare module directives so provisioning automatically installs and launches the right services. For example:

```toml
# hosts/motherbrain.toml
[[modules]]
name = "imu"
launch = true
env = { ROS_DOMAIN_ID = "25" }

[[modules]]
name = "foot"
launch = true

[[services]]
name = "tts"
setup = true
up = true
```

During `psh host setup`, each `modules` entry runs `psh mod setup <name>` (unless `setup = false`) and optionally launches the module when `launch = true`. The optional `env` map supplies environment variables to those lifecycle steps. Likewise, `services` entries trigger `psh svc setup <name>` and start the Compose stack when `up = true`.

### 3. Bring modules online

```bash
psh mod setup pilot   # prepare symlinks and dependencies
psh mod up pilot      # start the cockpit backend + Fresh frontend
```

Add other modules with `psh mod setup <name>` followed by `psh mod up <name>`. Shutdown with `psh mod down <name>`.

## Repository layout

```
├── modules/           # Module definitions (pilot, foot, imu, …)
├── modules/pilot/packages/pilot/  # Rust ROS bridge providing the cockpit backend (symlinked into work/src/pilot)
├── services/          # Containerised microservices (tts, language, graphs, vectors, …)
│   └── <name>/
│       ├── service.toml        # Manifest consumed by psh svc
│       └── docker-compose.yml  # Compose stack plus supporting assets
├── psh/               # Rust CLI for provisioning + module orchestration
├── psyched-msgs/      # Placeholder crate for shared ROS message helpers
├── tools/             # Host bootstrap & provisioning scripts
├── hosts/             # Host configuration TOML files
├── work/              # Colcon workspace (src/, build/, install/, log/) populated by tools/clean_workspace
├── target/            # Cargo build outputs
└── setup              # Top-level bootstrap script (see above)
```

## Modules

### Pilot

The pilot module provides a browser-based cockpit with two parts:

- **Backend:** `modules/pilot/packages/pilot/src/bin/cockpit.rs` exposes a websocket at `ws://0.0.0.0:8088/ws`. Incoming messages are mapped to ROS topics (currently `/conversation`; more topics coming soon). Outbound broadcasts like `/audio/transcript/final` are streamed to every connected browser.
- **Frontend:** `modules/pilot/frontend` is a Fresh app. `lib/cockpit.ts` contains the websocket client (`CockpitClient`) and a `useCockpitTopic` hook that any component can reuse.

Bring it up with:

```bash
psh mod setup pilot
psh mod up pilot
```

During development you can run the frontend directly:

```bash
cd modules/pilot/frontend
deno task dev
```

### IMU

Hardware module for an MPU6050 IMU. The module pulls in [`ros2_mpu6050_driver`](https://github.com/hiwad-aziz/ros2_mpu6050_driver) and exposes pilot UI components under `modules/imu/pilot/`. Launch it with `psh mod up imu` once the hardware is attached.

### Foot

Integrates the iRobot Create 1 drive base. The module checks out upstream ROS packages (`create_robot`, `libcreate`) and provides cockpit controls in `modules/foot/pilot/`. Launch via `psh mod up foot`.

Modules can optionally contribute Fresh components or routes by placing files inside `modules/<name>/pilot/{components,routes,islands,static}`. `psh mod setup <name>` manages symlinks into the pilot frontend.

## Services

Services complement modules by packaging long-running capabilities (speech, perception, data pipelines) as Docker Compose stacks under `services/<name>`. Each service has a `service.toml` manifest that `psh svc ...` understands. The manifest points at a Compose file plus any setup scripts required to fetch assets such as model checkpoints.

Use the `psh svc` subcommands to interact with services:

- `psh svc setup <service>` – run any declarative setup scripts (for example, download models)
- `psh svc up <service>` / `psh svc down <service>` – start or stop the Compose stack
- `psh svc list` – inspect available services and their status

### TTS

`services/tts` hosts a streaming text-to-speech websocket based on 🐸Coqui TTS. The Compose stack builds a slim Python image (`services/tts/docker/tts-websocket.Dockerfile`) that runs `tools/tts_websocket/websocket_server.py` and exposes port `5002`.

Model assets live under `services/tts/models` and are mounted into the container at `/models`. Run `psh svc setup tts` (or execute `services/tts/setup.sh` manually) to pull the default English voice. Once prepared, start the service with `psh svc up tts` and connect clients to `ws://<host>:5002/tts`.

### Language

`services/language` provides a GPU-enabled [Ollama](https://ollama.com/) runtime. The Compose stack binds Ollama's data directory from `/usr/share/ollama/.ollama` so models persist across container restarts and exposes the standard API on port `11434`.

Before starting the stack, ensure `/usr/share/ollama/.ollama` exists on the host (create it and grant write access to the Docker daemon user if needed). Bring the service online with `psh svc up language`; stop it via `psh svc down language`. GPU access requires Docker's NVIDIA runtime—verify that `nvidia-smi` works on the host and the Docker daemon has `default-runtime=nvidia` or an equivalent device mapping configured. After the first run, load desired models inside the container using `ollama run <model>` or the HTTP API.

### Graphs

`services/graphs` runs a standalone [Neo4j](https://neo4j.com/) graph database. Ports `7474` (HTTP UI) and `7687` (Bolt) are exposed, and named volumes back `/data`, `/logs`, `/import`, and `/plugins` to persist graph state between restarts. Start and stop it with `psh svc up graphs` / `psh svc down graphs`. The `NEO4J_AUTH` environment variable defaults to `neo4j/password`; override this in production by editing the Compose file or passing environment overrides in your host configuration.

### Vectors

`services/vectors` offers a [Qdrant](https://qdrant.tech/) vector database useful for retrieval-augmented generation pipelines. It publishes HTTP and gRPC endpoints on `6333` and `6334` respectively and persists storage under the `vectors_data` named volume. Use `psh svc up vectors` to launch it and `psh svc down vectors` to stop. Populate the collection schema via Qdrant's REST API, the CLI, or any of the official client libraries.

### ASR

`services/asr` exposes a custom Rust websocket server that streams speech-to-text using [`whisper-rs`](https://github.com/tazz4843/whisper-rs). Send 16-bit little-endian PCM frames (default `16 kHz`) to `/asr`; the service buffers audio, emits partial transcripts with token-level timings as the model converges, and publishes finalised chunks alongside a WAV payload once segments stabilise. Mount pretrained Whisper models into `services/asr/models` (run `psh svc setup asr` to download defaults) and bring the stack online with `psh svc up asr`.

## Development workflows

### Vendored ROS Rust bindings

Provisioning invokes `tools/bootstrap/generate_ros_rust_bindings.sh` after the ROS apt packages finish installing. The helper spins up a temporary Docker builder for `ros-${ROS_DISTRO:-kilted}-ros-base`, runs `colcon build` with `rosidl_generator_rs`, and copies the resulting crates into `vendor_msgs/`. Cargo is configured to patch common message crates (e.g. `std_msgs`, `sensor_msgs`) to those vendored copies so Rust builds no longer depend on a local colcon workspace.

If Docker was unavailable during provisioning—or you switch `ROS_DISTRO` later—rerun the script manually:

```bash
ROS_DISTRO=${ROS_DISTRO:-kilted} tools/bootstrap/generate_ros_rust_bindings.sh
```

The command refreshes the crates in `vendor_msgs/` and prints warnings for any package that fails to build. Commit the updated directories when upstream interface definitions change.

### Rust + ROS backend

- Build everything: `cargo build --workspace`
- Run the cockpit backend alone: `cargo run --manifest-path work/src/pilot/Cargo.toml --bin cockpit`
- Format / lint: `cargo fmt`, `cargo clippy --workspace --all-targets`

ROS packages live under `src/` (mirrored into `packages/` for colcon). Use `psh build` to compile ROS nodes without invoking `colcon` directly:

```bash
psh build                 # builds the entire workspace
psh build mpu6050driver   # optionally target specific packages
source install/setup.bash
```

### Pilot frontend

- Install dependencies: handled automatically by Deno (`deno.json` + `deno.lock`)
- Dev server: `deno task dev`
- Type-check: `deno check lib/cockpit.ts`
- Format: `deno fmt`

### Using the `psh` CLI

`psh` wraps common workflows:

- `psh host setup <host>` – executes the bootstrap scripts for a host defined in `hosts/<host>.toml`
- `psh mod list` – inspect module status
- `psh mod setup|teardown <module>` – manage symlinks + prep work
- `psh mod up|down <module>` – start/stop module services
- `psh svc list` – inspect containerised services and their status
- `psh svc setup|up|down <service>` – prepare assets (model downloads, etc.) and manage Docker Compose stacks
- `psh env` – injects a `psyched()` helper into your shell rc file so sourcing ROS + the local workspace is one command away
- `psh clean` – reset `work/` and re-establish local ROS package symlinks (including module packages)

## Testing & validation

- **Rust:** `cargo test --workspace`
- **Pilot Frontend:** `deno test` (add tests under `modules/pilot/frontend`). Use `deno check` to catch type errors.
- **ROS nodes:** Build via `colcon build` then run `ros2 test` or module-specific launch files as needed.

CI is currently manual; prefer running the commands above before pushing.

## Troubleshooting

- Missing ROS dependencies: ensure `ROS_DISTRO` is exported (defaults to `kilted` in scripts). Re-run `psh env` after changing the distro.
- Cockpit websocket unreachable: verify `cargo run --manifest-path work/src/pilot/Cargo.toml --bin cockpit` logs “listening on ws://…/ws” and that port `8088` is open on the host.
- Pilot frontend cannot type-check: delete `modules/pilot/frontend/deno.lock` and re-run `deno task cache` if your Deno version is older than the lockfile format.
- Module assets not visible in the UI: re-run `psh mod setup <module>` to regenerate symlinks.

## Roadmap

- Expand cockpit topics beyond `/conversation`
- Ship IMU + drive base dashboards in the pilot UI
- Add ear/eye/chat/voice modules (see `modules/` placeholders)
- Automate CI for cargo, deno, and colcon builds

## License

Licensed under the MIT License. See [`LICENSE`](./LICENSE).
