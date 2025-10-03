# Docker Dev Container

This repo includes a simple ROS 2 development container you can use to run the bootstrapper and provisioning flow end‑to‑end without touching the host.

- Base image: `ros:humble-ros-base` (Ubuntu 22.04)
- Exposed ports:
  - `8000`: Fresh dev server for the pilot UI
  - `8088`: Cockpit websocket bridge (`ws://0.0.0.0:8088/ws`)
- The container “dresses” itself as a hostname you choose so `psh` applies the matching profile from `hosts/<name>.toml`.

## Quick start

From the repo root:

```bash
# 1) Choose a host profile by name (matches hosts/<name>.toml)
export PSY_HOSTNAME=motherbrain   # or forebrain

# 2) Build and start the dev container, running ./setup inside
docker compose -f docker/compose.yml up --build

# The setup script will install tools (inside the container), register the psh wrapper,
# and open an interactive provisioning wizard. It’s OK if some steps fail in tests.
```

To run interactively without tailing logs:

```bash
export PSY_HOSTNAME=motherbrain
# Build image
docker compose -f docker/compose.yml build
# Start a shell in the container and run the bootstrapper on demand
docker compose -f docker/compose.yml run --rm dev bash
# Inside the container
./setup
```

## Notes

- Hostname selection: `docker/compose.yml` sets `hostname: ${PSY_HOSTNAME:-motherbrain}`; `psh` reads `Deno.hostname()` and loads `hosts/<name>.toml` accordingly.
- ROS distro: provisioning defaults to a custom `kilted` name, so the Compose stack exports `ROS_DISTRO=humble` to match the base image.
- Networking: bridge mode is sufficient for local tests. If you need ROS discovery across host ↔ container, consider `network_mode: host` on Linux.
- Deno: the bootstrapper installs Deno inside the container; the `DENO_TLS_CA_STORE=system` env is set for restricted environments.
- Data: the repo is bind‑mounted into `/workspace/psyched` so builds and artifacts (e.g., `work/`) persist on the host.

## Common commands

- Rebuild image after Dockerfile changes:
  `docker compose -f docker/compose.yml build --no-cache dev`
- Drop into the running container (another shell):
  `docker exec -it $(docker ps --filter name=$PSY_HOSTNAME -q) bash`
- Stop and remove the container:
  `docker compose -f docker/compose.yml down`

