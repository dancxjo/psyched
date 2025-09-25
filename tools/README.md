Provisioning helpers
====================

This directory now hosts the minimal set of scripts that back the `psh`
commands. They are kept as simple bash utilities so they can be invoked
manually when needed or wrapped by the CLI for idempotent provisioning.

install_docker.sh
-----------------

Installs Docker Engine and the Docker Compose plugin from Docker's official apt
repository. Requires sudo/root on Debian/Ubuntu systems.

Usage:

```bash
./install_docker.sh           # direct invocation
psh dep docker                # preferred CLI wrapper
```

Notes:

- Adds the current user to the `docker` group; log out/in or run `newgrp docker`
  to refresh group membership.
- For non-Ubuntu derivatives, adjust the release codename derived from
  `/etc/os-release` if necessary.

install_ros2.sh
---------------

Bootstraps the ROS 2 base distribution, rosdep, and Colcon tooling using the
upstream apt repository.

Usage:

```bash
ROS_DISTRO=kilted ./install_ros2.sh
psh dep ros2
```

setup_env.sh
------------

Prints or sources the shell code required to activate ROS 2 and the workspace.
This script underpins `psh env` and can be sourced directly from interactive
shells or systemd units.

```bash
# Emit shell code suitable for eval in .bashrc
SETUP_ENV_MODE=print bash tools/setup_env.sh

# Source into the current shell
source tools/setup_env.sh
```

systemd_entrypoint.sh
---------------------

Small helper that sources `tools/setup_env.sh` before executing the command
passed by `psh systemd` when launching modules under systemd. It ensures every
unit starts with the correct ROS environment.
