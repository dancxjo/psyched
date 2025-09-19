# Psyched - Experimental Robot Framework

Psyched is an experimental robot framework built on ROS 2. The repository ships with
**psh** (the *psyched shell*), a single-file Rust CLI that replaces the legacy
Makefile workflows and streamlines day-to-day provisioning.

## Prerequisites

Before running `psh`, make sure the following tools are available:

- [Git](https://git-scm.com/) for cloning repositories
- [Rust toolchain](https://rustup.rs/) providing `cargo` for building the CLI
- `rosdep`, `colcon`, and a ROS 2 distribution (defaults to `kilted`). If they are
  missing, run `psh ros2` to install the upstream Debian packages automatically.
- `systemd` for managing long-running module services (or run launchers manually)

`psh` checks for these tools at runtime and surfaces actionable error messages with
links to installation instructions when something is missing.

## Installing and Updating `psh`

```bash
# Build in release mode from the repository root
cargo build --release

# Install the compiled binary into ~/.local/bin
cargo run -- install

# Or perform the full clone → build → install pipeline
cargo run -- update
```

By default, `psh` manages a checkout of this repository in `/opt/psyched` and installs
its binary into `~/.local/bin`. Set the `PSH_REPO_DIR` and `PSH_INSTALL_DIR`
environment variables to override these locations.

## Installing ROS 2 via `psh`

Install ROS 2 and its tooling using the bundled script:

```bash
# Install ROS 2 (defaults to the ROS_DISTRO environment variable or kilted)
psh ros2

# Install a specific distribution
psh ros2 --distro jazzy

# Run a custom provisioning script instead of scripts/install_ros2.sh
psh ros2 --script /path/to/script.sh
```

The command executes `scripts/install_ros2.sh`, which performs the apt-based
installation, initialises `rosdep`, and writes `/etc/profile.d/ros2-defaults.sh`
so the environment variables persist across shells.

## Hosts and Modules

`psh` keeps declarative host definitions under [`hosts/`](hosts/). Each host is a TOML
file listing the modules that should be prepared and optionally launched on that
machine. Two starter configurations are provided:

- `cerebellum.toml` — provisions and launches the `foot` module
- `forebrain.toml` — provisions the `foot` module without enabling the service

Apply a host configuration with:

```bash
psh host apply cerebellum
```

### Module Lifecycle Commands

Modules encapsulate ROS packages and their supporting services. The first module is the
`foot` module, which automates the `create_robot` bring-up workflow. Interact with
modules directly using:

```bash
# Clone dependencies and build the workspace for the module
psh module setup foot

# Remove module artefacts and disable its service
psh module remove foot

# Enable and start the module's systemd service
psh module launch foot
```

The launch command writes a `psyched-foot.service` unit file to `/etc/systemd/system`
and enables it so the robot bring-up runs automatically at boot.

Each module is a folder under [`modules/`](modules/) containing lifecycle scripts such as
`setup.sh`, `teardown.sh`, and `launch.sh`. `psh` simply executes these scripts inside the
checked-out repository, keeping the coordinator lightweight while allowing modules to
evolve independently.

## Workspace Layout

```
psyched/
├── Cargo.toml             # Workspace manifest (declares the tools/psh crate member)
├── hosts/                 # Host configuration files consumed by psh
├── modules/               # Module entrypoints implemented as shell scripts
├── tools/
│   ├── install_ros2.sh    # Provision ROS 2 from Debian packages
│   ├── setup_env.sh       # Interactive helper for sourcing environments
│   └── psh/
│       ├── Cargo.toml     # psh CLI crate metadata
│       └── src/           # Rust sources (thin coordinator)
├── packages/              # Tracked ROS 2 packages (ament)
│   ├── psyched/           # Example C++/Python package
│   └── voice/             # ament_python package (Piper TTS)
└── src/                   # Populated at setup time with symlinks to packages/

Notes on src/ vs packages/:
- Keep your packages under `packages/` in git.
- Module `setup.sh` scripts create a fresh `src/` and symlink the needed packages in.
- The entire `src/` dir is ignored by git to enable per-module curated workspaces.

### Voice module (Piper TTS)

The `voice` package is an `ament_python` node that subscribes to a `std_msgs/String`
topic and speaks queued messages via [Piper TTS](https://pypi.org/project/piper-tts/).

Quick start:

```bash
# Optional: download a voice and set PIPER_VOICE env var
python3 -m piper.download_voices en_US-lessac-medium
export PIPER_VOICE="$HOME/.local/share/piper/voices/en/en_US-lessac-medium.onnx"

# Prepare and build workspace for the voice module
psh module setup voice

# In a new shell (or after sourcing env), launch the node
ros2 launch voice voice.launch.py

# In another terminal, publish a message
ros2 topic pub -1 /voice std_msgs/String '{data: "Hello from Psyched"}'

# Control topics (std_msgs/Empty)
ros2 topic pub -1 /voice/pause std_msgs/Empty '{}'
ros2 topic pub -1 /voice/resume std_msgs/Empty '{}'
ros2 topic pub -1 /voice/clear std_msgs/Empty '{}'
```

Parameters:
- `topic` (default `/voice`): subscription topic for text
- `voice_path` (default `$PIPER_VOICE`): `.onnx` voice model path
- `use_cuda` (default `false`): enable GPU if `onnxruntime-gpu` is installed
- `wav_out_dir` (optional): write WAVs to this dir (also tries to play via `aplay`)
- `aplay` (default `true`): play audio via `aplay` if available
```

## CLI instead of Make

The Makefile has been reduced to a stub that points to `psh`. Use the CLI for all
automation; for example `psh ros2`, `psh module setup foot`, and `psh module remove foot`.
