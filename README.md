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

By default, `psh` manages a checkout of this repository in `$HOME/psyched` and installs
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

## Workspace Layout

```
psyched/
├── hosts/                 # Host configuration files consumed by psh
├── psh.rs                 # Single-file Rust CLI source
├── Cargo.toml             # Cargo metadata for the CLI
├── Makefile               # Legacy targets (now powered by psh)
├── scripts/               # Helper scripts for manual workflows
├── tools/                 # Provisioning helpers (install scripts, etc.)
└── src/                   # ROS 2 packages (ament build system)
```

## Legacy Makefile Targets

The existing Makefile remains available for compatibility. It mirrors the legacy
workflows for situations where `psh` is unavailable. The important targets are:

- `make ros2` — Install ROS 2 using the Debian packages
- `make workspace` — Create and build the ROS workspace at `$HOME/psyched`
- `make env` — Print environment setup instructions
- `make build` — Install dependencies and rebuild the workspace
- `make clean` — Remove build artefacts

New development should happen through the `psh` CLI to benefit from the consistent UX,
automated dependency checks, and host/module abstractions.
