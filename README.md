# psyched

**Psyched** is a modular robotics stack: a ROS 2 workspace, a collection of hardware-specific modules, and a web-based **Cockpit** console that lets you drive and monitor a robot from any browser.
The repository doubles as a reference implementation for building your own robot—swap out the hardware modules, keep the orchestration.

Pete itself is an embodied experiment in cognition and autonomy: an iRobot Create 1 base (**foot**) topped with sensors, a Raspberry Pi (**motherbrain**) running ROS 2 and coordination logic, and remote **forebrain** servers handling speech, perception, and memory.

---

## Table of contents

* [Project overview](#project-overview)
* [Quick start](#quick-start)
* [Repository layout](#repository-layout)
* [Modules](#modules)
* [Services](#services)
* [Development workflows](#development-workflows)
* [Shell environment helpers](#shell-environment-helpers)
* [Testing & validation](#testing--validation)
* [Troubleshooting](#troubleshooting)
* [Roadmap](#roadmap)
* [Design principles](#design-principles)
* [License](#license)

---

## Project overview

Psyched is organized around three ideas:

1. **Composable modules.**
   Each hardware or capability lives in `modules/<name>` with declarative metadata (`module.toml`) and lifecycle scripts (`launch_*`, `shutdown_*`).
   Modules can add UI panels to the Cockpit console without changing its core frontend.

2. **Containerized services.**
   Cross-cutting capabilities (speech stacks, perception pipelines, databases) live in `services/<name>` with a `service.toml` manifest and Docker Compose stack.
   They boot via `psh svc …` and share helper assets under `tools/`.

3. **Unified orchestration.**
   The Deno-based **`psh`** CLI provisions hosts, manages modules, and coordinates both ROS 2 and container lifecycles.
   It abstracts away platform differences between **motherbrain** (the robot) and **forebrain** (remote servers).

### System architecture

| Layer           | Role                                                                          | Typical host         |
| --------------- | ----------------------------------------------------------------------------- | -------------------- |
| **motherbrain** | Primary ROS 2 host; manages sensors, actuators, and local modules.            | Raspberry Pi 4 / 5   |
| **forebrain**   | Remote inference stack; runs ASR / TTS / LLM / memory services in containers. | GPU laptop or server |
| **cockpit**       | Browser-based cockpit for driving, diagnostics, and conversation.             | Any web client       |

### Cognitive intent

Beyond robotics, **Psyched** prototypes a modular cognition framework:
sensors feed **ear** and **eye** modules; impressions flow through the **pilot** for reasoning, are expressed via **voice**, and will eventually persist in planned **memory** services (Neo4j + Qdrant) to form a durable autobiographical record.
In time, Pete’s “psyche” will be able to recall, reason, and act across sessions.

---

## Quick start

### 1. Clone & bootstrap

```bash
git clone https://github.com/dancxjo/psyched.git
cd psyched
./setup
```

The bootstrap script installs prerequisites, registers the Deno-based `psh` CLI, configures mDNS, and runs initial host provisioning.
After it completes, open a new terminal or `source ~/.bashrc` before continuing.

### 2. Provision hosts

```bash
psh host setup motherbrain
psh host setup forebrain
```

Each host file under `hosts/` declares its installers (ROS 2, Docker, CUDA, Deno, etc.), modules, and dependent services.
Provisioning automatically applies those directives and sets the ROS 2 domain ID from `config/ros_domain_id`.

### 3. Bring modules online

```bash
psh mod setup cockpit
psh up cockpit
```

Then visit **http://<host>:8088/** to access the cockpit (HTTP + WebSocket bridge).
Launch other modules or services using `psh up <name>`; stop them with `psh down <name>`.

---

## Repository layout

```
├── modules/         # ROS or Python modules (cockpit, foot, imu, …)
├── services/        # Containerized microservices (tts, language, graphs, vectors, …)
├── tools/           # Provisioning scripts + Deno-based psh CLI
├── hosts/           # Host configuration JSON/YAML profiles
├── config/          # Global settings (ROS domain, etc.)
├── setup            # Top-level bootstrap script
└── work/            # Colcon workspace (src/, build/, install/, log/)
```

---

## Modules

| Module                                   | Purpose                                                                                                                                                                    |
| ---------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **IMU**                                  | Interfaces with an MPU-6050 sensor via [`ros2_mpu6050_driver`](https://github.com/hiwad-aziz/ros2_mpu6050_driver); provides orientation data and optional Cockpit UI widget. |
| **Foot**                                 | Controls the iRobot Create 1 drive base using upstream ROS 2 packages (`create_robot`, `libcreate`).                                                                       |
| **Cockpit**                                | Unified cockpit webserver and topic bridge.                                                                                                                                |
| *(planned)* **Ear**, **Chat**, **Voice** | Speech perception and expression modules built atop ASR / LLM / TTS services.                                                                                              |

Launch any module with `psh up <module>` once configured.

---

## Services

Containerized long-running capabilities under `services/<name>`:

| Service      | Description                                                                                 |
| ------------ | ------------------------------------------------------------------------------------------- |
| **tts**      | 🐸 Coqui TTS websocket on `5002`; run `psh svc setup tts` to fetch models.                  |
| **language** | [Ollama](https://ollama.com/) LLM runtime (GPU-enabled) on `11434`.                         |
| **graphs**   | [Neo4j](https://neo4j.com/) graph DB for relational memory.                                 |
| **vectors**  | [Qdrant](https://qdrant.tech/) vector DB for embeddings.                                    |
| **asr**      | Custom Rust websocket server using [`whisper-rs`](https://github.com/tazz4843/whisper-rs`). |
| **ros2**     | Dev-container shell with mounted workspace for isolated ROS builds.                         |

Start or stop them with `psh up <service>` / `psh down <service>`.

---

## Development workflows

### Python + ROS backend

```bash
psh build            # builds all packages
psh build cockpit      # or target one
source install/setup.bash
ros2 run cockpit cockpit
```

### Using the `psh` CLI

`psh` unifies module and service lifecycles:

```bash
psh setup            # full host + module + service provisioning
psh mod list         # view module status
psh up ear voice     # start modules
psh down             # stop all
psh sys enable cockpit # register systemd units (auto-starts at boot)
```

For details on systemd integration, see [docs/psh-sys.md](docs/psh-sys.md).

### Workspace cleanup

`psh clean` tears down everything and rebuilds the ROS 2 workspace.
It wipes `work/`, re-creates `src/`, and relinks active modules.

---

## Shell environment helpers

`env/psyched_env.sh` centralizes environment variables and provides helper functions such as `psyched::activate`.
A lightweight `psyched` function in `.bashrc` ensures correct sourcing on login.

---

## Testing & validation

* Build ROS packages via `colcon build`.
* Run unit tests with `colcon test`.
* Manual integration tests through Cockpit dashboard.

---

## Troubleshooting

| Symptom                  | Fix                                                                       |
| ------------------------ | ------------------------------------------------------------------------- |
| Missing ROS dependencies | Ensure `$ROS_DISTRO` is set (default `kilted`). Run `psyched --ros-only`. |
| Cockpit unreachable      | Confirm `ros2 run cockpit cockpit` prints its URL and port 8088 is open.    |
| Frontend type errors     | Delete `modules/cockpit/frontend/deno.lock` and rerun `deno task cache`.    |

---

## Roadmap

* Expand Cockpit topics for speech and transcript monitoring.
* Add full **ear/voice** stack.
* Integrate Neo4j + Qdrant memory daemons.
* CI for Cargo + Deno + Colcon builds.
* Declarative `pete.toml` hardware manifest for auto-introspection.

---

## Design principles

Psyched was created collaboratively between humans and AI agents.
Development has involved **Codex**, **Copilot**, **ChatGPT**, and other large-language models as active contributors—writing, refactoring, and documenting much of the code you see here.
This experiment in *co-development* treats the software itself as an emergent conversation between toolmakers, machines, and meaning.

Core principles:

1. **Transparency** – Every action can be inspected: modules declare their lifecycle, services their containers, hosts their profiles.
2. **Composability** – Each part stands alone; orchestration glues them into a coherent body.
3. **Embodiment** – Code runs on real hardware; cognition is grounded in sensors and motion.
4. **Narrative continuity** – Logs, memory, and dialogue are all one timeline.
5. **Collaboration** – Psyched acknowledges its mixed authorship: *a mind assembled in dialogue.*

---

## License

Licensed under the **MIT License**.
See [`LICENSE`](./LICENSE).
