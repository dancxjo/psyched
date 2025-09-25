Docker and other helper installers
================================

This directory contains helper provisioning scripts used by the repository.

install_docker.sh
------------------

Installs Docker Engine and the Docker Compose plugin from Docker's official
apt repository. The script requires sudo/root and is designed for Debian/Ubuntu
based systems.

Usage:

  ./install_docker.sh

Or from the repository CLI wrapper (requires deno and the psh script):

  deno run -A psh/main.ts setup docker

Or after installing the `psh` wrapper:

  sudo deno run -A psh/main.ts install
  psh setup docker

Notes:
- The script will add your user to the `docker` group; you may need to log out
  and back in (or run `newgrp docker`) for the group change to take effect.
- If you're running on a non-Ubuntu Debian derivative, the script tries to use
  the `VERSION_CODENAME` from `/etc/os-release`. Adjust as needed for your
  distribution.
