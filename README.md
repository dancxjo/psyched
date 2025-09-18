# Psyched - Experimental Robot Framework

Psyched is an experimental robot framework built on ROS 2. The repository now ships
with **psh** (the *psyched shell*), a single-file Rust CLI that replaces the top-level
Makefile workflows and makes day-to-day development and provisioning less painful.

## Prerequisites

Before running `psh`, make sure the following tools are available:

- [Git](https://git-scm.com/) for cloning repositories
- [Rust toolchain](https://rustup.rs/) providing `cargo` for building the CLI
- `rosdep`, `colcon` and a ROS 2 distribution (defaults to `kilted`)
- `systemd` for managing long-running module services (or run module launchers manually)

If you are missing a tool, `psh` will surface an actionable error message and a link to
the recommended installation instructions.

Before running `psh`, make sure the following tools are available. The `psh ros2`
command can provision ROS 2 and its tooling automatically if you do not already
have them installed:

- [Git](https://git-scm.com/) for cloning repositories
- [Rust toolchain](https://rustup.rs/) providing `cargo` for building the CLI
- `rosdep`, `colcon` and a ROS 2 distribution (defaults to `kilted`). Run
  `psh ros2` to install these system dependencies using the upstream Debian
  packages.
- `systemd` for managing long-running module services (or run module launchers manually)
- [Rust toolchain](https://rustup.rs/) providing `cargo` for building the CLI
- `rosdep`, `colcon` and a ROS 2 distribution (defaults to `kilted`). Run
  `psh ros2` to install these system dependencies using the upstream Debian
  packages.
- `systemd` for managing long-running module services (or run module launchers manually)


If you are missing a tool, `psh` will surface an actionable error message and a link to
the recommended installation instructions.


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
environment variables if you prefer alternative locations.

## Hosts and Modules

`psh` keeps declarative host definitions under [`hosts/`](hosts/). Each host is a TOML
file that lists the modules that should be prepared and optionally launched on that
machine. Two starter configurations are provided:

- `cerebellum.toml` — provisions and launches the `foot` module
- `forebrain.toml` — provisions the `foot` module without enabling the service

Apply a host configuration with:

```bash
psh host apply cerebellum
```

### Module Lifecycle Commands

## Installing ROS 2 via `psh`

Provision ROS 2 in the same way the legacy Makefile did by calling the bundled
script:

```bash
# Install ROS 2 (defaults to the ROS_DISTRO environment variable or kilted)
psh ros2

# Install a specific distribution
psh ros2 --distro jazzy
```

The command executes `scripts/install_ros2.sh`, which performs the apt-based
installation, initialises `rosdep`, and writes `/etc/profile.d/ros2-defaults.sh`
so the environment variables persist across shells. Pass `--script` if you need
to run a customised provisioning script.

## Hosts and Modules

`psh` keeps declarative host definitions under [`hosts/`](hosts/). Each host is a TOML
file that lists the modules that should be prepared and optionally launched on that
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
and enables it so that the robot bring-up is executed automatically at boot.


## Workspace Layout

```
psyched/
├── hosts/                 # Host configuration files consumed by psh
├── psh.rs                 # Single-file Rust CLI source
├── Cargo.toml             # Cargo metadata for the CLI
├── Makefile               # Legacy targets (now powered by psh)
├── scripts/               # Helper scripts for manual workflows
└── src/                   # ROS 2 packages (ament build system)
```

├── tools/               # Helper scripts for manual workflows
└── src/                   # ROS 2 packages (ament build system)
```


## Legacy Makefile Targets

The existing Makefile remains available for compatibility. It mirrors the legacy
workflows for situations where `psh` is unavailable. The important targets are:

- `make ros2` — Install ROS 2 using the Debian packages
- `make workspace` — Create and build the ROS workspace at `/opt/psyched`
- `make env` — Print environment setup instructions
- `make build` — Install dependencies and rebuild the workspace
- `make clean` — Remove build artefacts

However, new development should happen through the `psh` CLI to benefit from the
consistent UX, automated dependency checks and host/module abstractions.
