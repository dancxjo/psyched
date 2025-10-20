# Agent handbook

Welcome to the Psyched workspace. This guide summarizes everything an automated assistant (humans benefit, too) should know before touching the codebase.

## System snapshot

- **Mission:** orchestrate a modular ROS 2 stack for the robot "Pete" while exposing a cockpit control layer that also provides a browser-based view for human operators.
- **Languages & frameworks:** Python (`rclpy`, `aiohttp`), the `psh` CLI, ROS 2 (colcon), Bash, optional Rust for certain services (e.g. ASR), and assorted Python/C++ ROS packages pulled in as git dependencies.
- **Runtime topology:**
- The pilot module is Pete's autonomous control layer—the LLM-driven brain that directly consumes each module's API; the cockpit exposes the same APIs for humans through the CLI and browser-based controls when debugging or supervising.
- Human-facing surfaces (CLI, cockpit UI) exist for setup, debugging, and oversight—the pilot automation remains the primary operator of the system.
- The `psh` CLI provisions hosts (`hosts/*.json`) and synchronizes module assets into the static cockpit frontend served by the cockpit module.

## Key directories and entry points

| Path | Purpose |
| --- | --- |
| `tools/psh/` | `psh` Deno CLI for provisioning and module orchestration. Entry: `main.ts`. |
| `modules/<name>/module.toml` | Module manifest consumed by `psh`. Also defines bootstrap git repos and overlays. |
| `tools/bootstrap/` & `tools/provision/` | Host bootstrap scripts invoked by `psh host setup`. |
| `setup` | Top-level bootstrap script. Installs dependencies, installs the Deno-based `psh` wrapper, and instructs you to reboot before running `psh`. |
| `hosts/*.json` | Host manifests. Prefer `provision.installers = ["ros2", …]` over shell scripts and run `psh host setup --verbose` for detailed logs. Deno is bootstrapped by `setup`. |

## Build & test checklist

Always prefer running the smallest relevant command set.

| Domain | Commands |
| --- | --- |
| ROS packages (colcon) | `colcon build --packages-select <pkg>` followed by `source install/setup.bash` |
| `psh` CLI | `cd tools/psh && deno fmt && deno lint && deno task test` |
| Shell scripts | `shellcheck modules/**/launch_*.sh modules/**/shutdown_*.sh setup` |

> ✅ **Definition of done:** code formatted, linted, and the smallest relevant test/build commands above succeed.

## Coding guidelines

### Deno CLI

- Use the shared import map in `tools/psh/deno.json` (run scripts with `deno run --config tools/psh/deno.json ...`).
- Prefer [`dax`](https://deno.land/x/dax) for shell orchestration instead of bespoke bash snippets.
- Add regression coverage with `deno test` for new helpers before wiring them into the CLI (TDD/BDD encouraged).
- Document exported helpers with `/** ... */` JSDoc comments when behaviour isn’t self-evident.


### Shell

- Every script should start with `#!/usr/bin/env bash` and `set -euo pipefail` unless a different shell is mandatory.
- Use functions for repeated logic, log via `echo`/`printf` with context, and clean up background processes in `trap` handlers.

## Workflow guardrails

1. **Collect context first.** Read the relevant `module.toml`, scripts, and associated docs before editing.
2. **Prefer existing tooling.** Use `psh mod setup|up|down` rather than invoking lifecycle scripts manually.
3. **Keep modules isolated.** Touch only the module(s) you intend to change; module folders shouldn’t depend on each other directly.
4. **Update documentation.** When behavior changes, update `README.md`, module READMEs, or this handbook accordingly.
5. **Log notable decisions.** Commit messages and PR descriptions should mention affected hosts/modules and manual steps if any.

## Known pitfalls

- **Lockfile drift:** `deno.lock` enforces lockfile version ≥5. Older Deno releases will fail with “unsupported lockfile version”. Upgrade Deno or regenerate the lock.
- **ROS distro mismatch:** Scripts default to the custom `kilted` distro name. Override with `ROS_DISTRO=<distro>` before running `psh env` if you target `humble`/`jazzy`.
- **Workspace resets:** `tools/clean_workspace` wipes `work/` and leaves `src/` empty so disabled modules stay out of the build graph. Run it (or `psh clean`) when paths drift, then rerun `psh mod setup <module>` for any packages you need.
- **Network safety:** `psh clean` intentionally preserves connectivity-critical modules/services (Wi-Fi, SSH, mDNS). Keep those protections in place so remote sessions stay alive during cleanup.
- **Deno TLS certificates:** When fetching dependencies during `deno task test`, set `DENO_TLS_CA_STORE=system` if you encounter TLS certificate errors in restricted environments.
- **Deno permissions:** Prefer `deno task test` (which wraps `deno test --allow-all`) so permission-gated tests can read env vars and temp directories without manual flags.
- **Deno test harness:** Use `Deno.test(...)` when authoring unit tests—`deno test` is the CLI command and will not compile inside source files.
- **Deno availability in containers:** Some automation containers omit the `deno` binary; if commands such as `deno fmt` fail with "command not found," document the limitation instead of repeatedly retrying.
- **Front-end helper tests:** Several cockpit helper suites rely on `deno test`. Check that the `deno` binary is available before invoking them and call out the limitation when it is missing.
- **APT CLI stability:** Provisioning scripts must use `apt-get` (not `apt`) to avoid behaviour changes and interactive warnings during automation.
- **ROS tooling packages:** Avoid installing `python3-colcon-*` or other catkin/colcon Debian packages; rely on ros-base and rosdep instead to prevent dpkg conflicts on Pete's hosts.
- **ROS dev payload:** When tweaking installers ensure the `ros-<distro>-ros-dev-tools` meta-package stays in the dependency list so developer tooling (colcon, vcstool) lands without dragging in the desktop stack.
- **Colcon virtualenv:** The ROS installers create `/opt/ros/<distro>/colcon-venv` and symlink `/usr/local/bin/colcon` into it. Reuse that environment instead of layering additional pip/apt colcon installs.
- **Resource index drift:** When adding ROS 2 packages, ensure each `resource/<package>` file contains only the package name with a trailing newline and that Python packages ship a matching `setup.cfg` so ament indexing remains intact.
- **Post-bootstrap shell refresh:** `./setup` now runs `psh host setup` automatically while skipping module/service provisioning. Open a fresh shell (or `source ~/.bashrc`) before running `psh mod setup` / `psh svc setup` so the ROS environment is available.
- **Host setup scope:** `psh host setup` only runs installers/scripts by default. Pass `--include-modules` / `--include-services` (or run `psh mod setup` / `psh svc setup` afterward) when you need lifecycle steps.
- **Profile newline gotchas:** Deno's installer may append `source ~/.deno/env` without a trailing newline. Use `tools/bootstrap/profile_helpers.sh` to add exports to shell profiles so new lines aren't merged into the previous command.

## Useful references

- Cockpit frontend docs live alongside the cockpit module—no external build tooling is required.
- Create robot stack: <https://github.com/autonomylab/create_robot>
- MPU6050 ROS driver: <https://github.com/hiwad-aziz/ros2_mpu6050_driver>

Happy hacking!
