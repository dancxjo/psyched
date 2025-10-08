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
        - [Python + ROS backend](#python--ros-backend)
        - [Pilot frontend](#pilot-frontend)
        - [Using the `psh` CLI](#using-the-psh-cli)
        - [Workspace cleanup](#workspace-cleanup)
- [Docker dev container](#docker-dev-container)
- [Testing & validation](#testing--validation)
- [Troubleshooting](#troubleshooting)
- [Roadmap](#roadmap)
- [License](#license)

## Project overview

Psyched is organized around three ideas:

1. **Composable modules.** Each hardware or capability lives in `modules/<name>` with declarative metadata (`module.toml`) and lifecycle scripts (`launch_*`, `shutdown_*`). Modules can surface UI controls inside the pilot console without modifying the core frontend.
2. **Containerised services.** Cross-cutting capabilities (speech stacks, perception pipelines, etc.) live in `services/<name>` alongside a `service.toml` manifest and Docker Compose stack. They boot via `psh svc ...` and share helper assets under `tools/`.
3. **A ROS 2 + Python bridge.** The `pilot` package (linked into `work/src/pilot`) exposes a websocket API (`ws://<host>:8088/ws`) that forwards cockpit messages into ROS topics using `rclpy` and `websockets`.
4. **A modern pilot UI.** The `modules/pilot/frontend` package uses [Deno](https://deno.land/) and [Fresh](https://fresh.deno.dev/) with Preact hooks. It consumes the cockpit websocket via a reusable client in `lib/cockpit.ts`.

Supporting utilities live under `tools/` and the `psh` CLI: a Deno-powered orchestrator that provisions hosts, manages modules, and keeps the pilot frontend in sync.

## Quick start

### 1. Clone & bootstrap

```bash
git clone https://github.com/dancxjo/psyched.git
cd psyched
./setup  # installs core dependencies, registers the Deno-based psh CLI, and provisions host prerequisites
```

`./setup` installs the tooling required to run `psh`, configures mDNS, prepares Deno, and runs `psh host setup` for the current machine. Host provisioning skips module and service lifecycle steps so you can finish configuration once your shell sources the ROS environment. When the script completes, open a new terminal (or `source ~/.bashrc`) before running `psh setup` to orchestrate `host`, `mod`, and `srv` provisioning (or fall back to `psh mod setup` / `psh srv setup` if you only need part of the workflow).

### 2. Provision additional machines (optional)

```bash
psh host setup motherbrain
psh host setup forebrain
```

Each host file lists shell scripts under `tools/bootstrap/` (or `tools/provision/`) to install ROS 2, CUDA, Docker, and Deno.

Host configs can also declare module directives so provisioning automatically installs and launches the right services. `depends_on` fields let modules and services wait for other tasks – installers (`"docker"`), other services (`"service:ros2"`), or even module launches (`"module:pilot"`). For example:

```json
{
  "host": {
    "name": "motherbrain",
    "installers": ["ros2", "docker"],
    "modules": ["imu", "foot"],
    "services": ["tts", "ros2"]
  },
  "modules": {
    "imu": {
      "launch": true
    },
    "foot": {
      "launch": true,
      "depends_on": ["service:ros2"]
    }
  },
  "services": {
    "tts": {
      "setup": true,
      "up": true
    },
    "ros2": {
      "up": true,
      "depends_on": ["docker"]
    }
  }
}
```

When you add `--include-modules` or `--include-services`, `psh host setup` will run the corresponding lifecycle commands (`psh mod setup <name>` or `psh svc setup <name>`) and optionally launch/start the targets. Without those flags the command only runs installers and scripts, printing reminders to finish provisioning later.

ROS 2 domain settings are managed globally via `config/ros_domain_id` and are applied automatically when modules and services run.

### 3. Bring modules online

```bash
psh mod setup pilot   # prepare symlinks and dependencies
psh up pilot          # start the cockpit backend + Fresh frontend
```

Add other modules with `psh mod setup <name>` followed by `psh up <name>`. Use `psh down <name>` to stop a module or `psh down` to stop everything that is running. When you're ready for the full stack, `psh up` without arguments launches every module and service.

## Docker dev container

Prefer to run everything inside a ROS 2 container for tests? A ready‑to‑use dev image and Compose stack are included. Pick a hostname (matching `hosts/<name>.json`), build, and start:

```bash
export PSY_HOSTNAME=motherbrain   # or forebrain
docker compose -f docker/compose.yml up --build
```

The container sets its hostname so `psh` applies the selected host profile, runs `./setup` automatically for an informative provisioning flow, and exposes ports `8000` (pilot UI) and `8088` (cockpit backend). See `docs/docker.md` for details.

## Repository layout

```
├── modules/                       # Module definitions (pilot, foot, imu, …)
│   └── <name>/
│       ├── module.toml            # Manifest consumed by psh
│       ├── packages/              # ROS/Python packages linked into work/src/
│       └── pilot/                 # Fresh overlays (components, routes, islands, …)
├── services/                      # Containerised microservices (tts, language, graphs, vectors, …)
│   └── <name>/
│       ├── service.toml           # Manifest consumed by psh svc
│       └── docker-compose.yml     # Compose stack plus supporting assets
├── tools/                         # Host bootstrap & provisioning scripts
│   └── psh/                       # Deno CLI for provisioning + module orchestration
├── hosts/                         # Host configuration JSON/YAML files
├── setup                          # Top-level bootstrap script (see above)
└── work/                          # Colcon workspace (src/, build/, install/, log/) created by tools/clean_workspace
```

## Modules

### Pilot

The pilot module provides a browser-based cockpit with two parts:

- **Backend:** `modules/pilot/packages/pilot/pilot_cockpit/bridge.py` exposes a websocket at `ws://0.0.0.0:8088/ws`. Incoming messages are mapped to ROS topics (`/conversation`, `/cmd_vel`) and ROS telemetry (`/audio/transcript/final`, `/imu/data`, `/foot/telemetry`) is fanned out to the browser.
- **Frontend:** `modules/pilot/frontend` is a Fresh app. `lib/cockpit.ts` contains the websocket client (`CockpitClient`) and a `useCockpitTopic` hook that any component can reuse.

Bring it up with:

```bash
psh mod setup pilot
psh up pilot
```

During development you can run the frontend directly:

```bash
cd modules/pilot/frontend
deno task dev
```

### IMU

Hardware module for an MPU6050 IMU. The module pulls in [`ros2_mpu6050_driver`](https://github.com/hiwad-aziz/ros2_mpu6050_driver) and exposes pilot UI components under `modules/imu/pilot/`. Launch it with `psh up imu` once the hardware is attached.

### Foot

Integrates the iRobot Create 1 drive base. The module checks out upstream ROS packages (`create_robot`, `libcreate`) and provides cockpit controls in `modules/foot/pilot/`. Launch via `psh up foot`.

Modules can optionally contribute Fresh components or routes by placing files inside `modules/<name>/pilot/{components,routes,islands,static}`. `psh mod setup <name>` manages symlinks into the pilot frontend.

## Services

Services complement modules by packaging long-running capabilities (speech, perception, data pipelines) as Docker Compose stacks under `services/<name>`. Each service has a `service.toml` manifest that `psh svc ...` understands. The manifest points at a Compose file plus any setup scripts required to fetch assets such as model checkpoints.

Use the `psh svc` subcommands to interact with services:

- `psh svc setup <service>` – run any declarative setup scripts (for example, download models)
- `psh up <service>` / `psh down <service>` – start or stop the Compose stack (add `--service` if a module shares the name). The legacy `psh svc up|down` commands continue to work.
- `psh svc list` – inspect available services and their status
- `psh svc shell <service> [command]` – ensure the Compose stack is up then open an interactive shell inside the container (defaults to `bash`)

### TTS

`services/tts` hosts a streaming text-to-speech websocket based on 🐸Coqui TTS. The Compose stack builds a slim Python image (`services/tts/docker/tts-websocket.Dockerfile`) that runs `tools/tts_websocket/websocket_server.py` and exposes port `5002`.

Model assets live under `services/tts/models` and are mounted into the container at `/models`. Run `psh svc setup tts` (or execute `services/tts/setup.sh` manually) to pull the default English voice. Once prepared, start the service with `psh up tts` (or `psh svc up tts`) and connect clients to `ws://<host>:5002/tts`.

### Language

`services/language` provides a GPU-enabled [Ollama](https://ollama.com/) runtime. The Compose stack binds Ollama's data directory from `/usr/share/ollama/.ollama` so models persist across container restarts and exposes the standard API on port `11434`.

Before starting the stack, ensure `/usr/share/ollama/.ollama` exists on the host (create it and grant write access to the Docker daemon user if needed). Bring the service online with `psh up language` (or `psh svc up language`); stop it via `psh down language`. GPU access requires Docker's NVIDIA runtime—verify that `nvidia-smi` works on the host and the Docker daemon has `default-runtime=nvidia` or an equivalent device mapping configured. After the first run, load desired models inside the container using `ollama run <model>` or the HTTP API.

### Graphs

`services/graphs` runs a standalone [Neo4j](https://neo4j.com/) graph database. Ports `7474` (HTTP UI) and `7687` (Bolt) are exposed, and named volumes back `/data`, `/logs`, `/import`, and `/plugins` to persist graph state between restarts. Start and stop it with `psh up graphs` / `psh down graphs` (or the legacy `psh svc up|down graphs`). The `NEO4J_AUTH` environment variable defaults to `neo4j/password`; override this in production by editing the Compose file or passing environment overrides in your host configuration.

### Vectors

`services/vectors` offers a [Qdrant](https://qdrant.tech/) vector database useful for retrieval-augmented generation pipelines. It publishes HTTP and gRPC endpoints on `6333` and `6334` respectively and persists storage under the `vectors_data` named volume. Use `psh up vectors` to launch it and `psh down vectors` to stop (legacy `psh svc up|down vectors` still works). Populate the collection schema via Qdrant's REST API, the CLI, or any of the official client libraries.

### ASR

`services/asr` exposes a custom Rust websocket server that streams speech-to-text using [`whisper-rs`](https://github.com/tazz4843/whisper-rs). Send 16-bit little-endian PCM frames (default `16 kHz`) to `/asr`; the service buffers audio, emits partial transcripts with token-level timings as the model converges, and publishes finalised chunks alongside a WAV payload once segments stabilise. Mount pretrained Whisper models into `services/asr/models` (run `psh svc setup asr` to download defaults) and bring the stack online with `psh up asr`.

### ROS 2 (container shell)

`services/ros2` ships a prebuilt [osrf/ros:humble-desktop](https://hub.docker.com/r/osrf/ros/) workspace with the repository mounted at `/workspace/psyched`. Start the stack with `psh up ros2` (or `psh svc up ros2`) and drop into the container using `psh svc shell ros2`. By default the shell runs `/ros_entrypoint.sh bash`, giving you a ROS-ready prompt without touching the host install. ROS 2 domain IDs inherit from `config/ros_domain_id`; override temporarily by exporting `PSYCHED_ROS_DOMAIN_ID` (or `ROS_DOMAIN_ID`) before launching. Set `ROS_DISTRO` to select a different ROS distribution.

## Development workflows

### Workspace cleanup

Use `psh clean` (or run `tools/clean_workspace` directly) to stop active modules and services before recreating the ROS workspace from scratch. The helper wipes `work/` and leaves an empty `src/` directory so disabled modules stay out of the build graph; rerun `psh mod setup <module>` afterwards to link the packages you need. Pass `--skip-modules`, `--skip-services`, or `--skip-workspace` to tailor the reset when you only need part of the cleanup.

### Python + ROS backend

- Build the cockpit bridge: `colcon build --packages-select pilot`
- Run unit tests: `colcon test --packages-select pilot`
- Launch the websocket bridge: `ros2 run pilot cockpit`

ROS packages live under `modules/*/packages/` and are symlinked into `work/src/` during `psh mod setup`. Use `psh build` to compile ROS nodes without invoking `colcon` directly:

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

- `psh host setup [host]` – execute the bootstrap scripts for the detected host or the named profile in `hosts/<host>.json`
- `psh setup` – provision the host, modules, and services in one shot
- `psh teardown` – tear down modules/services and reset the ROS workspace
- `psh clean` – aggressively tear down modules/services and rebuild the ROS workspace in one step
- `psh mod list` – inspect module status
- `psh mod setup|teardown [module]` – manage symlinks + prep work
- `psh up|down [target]` – start/stop modules and services (use `--service` to disambiguate names shared with modules)
- `psh srv list` – inspect containerised services and their status
- `psh srv setup|up|down [service]` – prepare assets (model downloads, etc.) and manage Docker Compose stacks
- `psh sys setup|teardown|enable|disable|up|down <module>` – install and control user-level systemd units for module launch scripts

## Shell environment helpers

- `env/psyched_env.sh` centralises workspace variables and ROS fallbacks. Source it in automation and call `psyched::activate` to ensure the right setup script is loaded.
- The bootstrapper injects a `psyched` function into `.bashrc` that sources the workspace (or your ROS distro when no workspace exists). Run `psyched --workspace-only` or `psyched --ros-only` to force a mode. Set `PSYCHED_AUTO_ACTIVATE=0` before login to skip the automatic call.

## Testing & validation

- **Pilot backend:** `colcon test --packages-select pilot`
- **Pilot Frontend:** `deno test` (add tests under `modules/pilot/frontend`). Use `deno check` to catch type errors.
- **ROS nodes:** Build via `colcon build` then run `ros2 test` or module-specific launch files as needed.

CI is currently manual; prefer running the commands above before pushing.

## Troubleshooting

- Missing ROS dependencies: ensure `ROS_DISTRO` is exported (defaults to `kilted` in scripts). Source `env/psyched_env.sh` and run `psyched --ros-only` after changing the distro.
- Cockpit websocket unreachable: verify `ros2 run pilot cockpit` logs “listening on ws://…/ws” and that port `8088` is open on the host.
- Pilot frontend cannot type-check: delete `modules/pilot/frontend/deno.lock` and re-run `deno task cache` if your Deno version is older than the lockfile format.
- Module assets not visible in the UI: re-run `psh mod setup <module>` to regenerate symlinks.

## Roadmap

- Expand cockpit topics beyond `/conversation`
- Ship IMU + drive base dashboards in the pilot UI
- Add ear/chat/voice modules (see `modules/` placeholders)
- Automate CI for cargo, deno, and colcon builds

## License

Licensed under the MIT License. See [`LICENSE`](./LICENSE).
