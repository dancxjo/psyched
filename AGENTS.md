# Agent handbook

Welcome to the Psyched workspace. This guide summarizes everything an automated assistant (humans benefit, too) should know before touching the codebase.

## System snapshot

- **Mission:** orchestrate a modular ROS 2 stack for the robot "Pete" while exposing a browser-based cockpit for operators.
- **Languages & frameworks:** Python (`rclpy`, `websockets`), Rust (`cargo` for `psh`), ROS 2 (colcon), Bash, Deno/Fresh + Preact for the pilot UI, and assorted Python/C++ ROS packages pulled in as git dependencies.
- **Runtime topology:**
  - `pilot` Python package hosts the cockpit websocket bridge (`ws://0.0.0.0:8088/ws`).
  - Each module in `modules/<name>` declares lifecycle scripts and optional UI widgets under `pilot/`.
  - The `psh` CLI provisions hosts (`hosts/*.toml`) and synchronizes module assets into the Fresh frontend.

## Key directories and entry points

| Path | Purpose |
| --- | --- |
| `psh/` | `psh` Rust CLI for provisioning and module orchestration. Entry: `src/main.rs`. |
| `modules/pilot/packages/pilot/pilot_cockpit/` | ROS-aware Python backend serving the cockpit websocket. |
| `modules/pilot/frontend/` | Deno Fresh app for the pilot console. |
| `modules/<name>/module.toml` | Module manifest consumed by `psh`. Also defines bootstrap git repos and pilot overlays. |
| `tools/bootstrap/` & `tools/provision/` | Host bootstrap scripts invoked by `psh host setup`. |
| `setup` | Top-level bootstrap script. Installs dependencies, builds `psh`, and launches `psh setup`. |

## Build & test checklist

Always prefer running the smallest relevant command set.

| Domain | Commands |
| --- | --- |
| Rust workspace | `cargo fmt`, `cargo check --workspace`, `cargo clippy --workspace --all-targets`, `cargo test --workspace` |
| Cockpit backend only | `colcon build --packages-select pilot && ros2 run pilot cockpit` |
| ROS packages (colcon) | `colcon build --packages-select <pkg>` followed by `source install/setup.bash` |
| Deno pilot UI | `deno fmt`, `deno check lib/cockpit.ts`, `deno task dev`, `deno test` |
| Shell scripts | `shellcheck modules/**/launch_*.sh modules/**/shutdown_*.sh setup` |

> ✅ **Definition of done:** code formatted, linted, and the smallest relevant test/build commands above succeed.

## Coding guidelines

### Rust

- Edition 2021/2024, keep `rustfmt` defaults.
- Prefer `anyhow`/`thiserror` for error handling.
- When touching ROS integration, guard long-running loops with graceful shutdown and avoid panicking inside callbacks.
- Document public structs/functions with `///` doc comments when behavior isn’t obvious.

### TypeScript / Fresh

- Use ES modules with explicit file extensions (Deno requirement).
- Stick to the hooks-based API exposed in `lib/cockpit.ts`. New topics should be declared in a single place so reconnect logic remains centralized.
- Keep hooks SSR-safe: guard browser-only APIs (`window`, `WebSocket`) with `typeof` checks.

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
- **Pilot backend is Python-first:** Only the `psh` CLI is written in Rust now. Expect ROS nodes and cockpit bridges to live under Python packages and keep their dependencies declared in module manifests.
- **Symlink overlays:** Deleting `modules/*/pilot` symlinks manually breaks the Fresh app. Always re-run `psh mod setup <module>`.
- **Background processes:** Launch scripts spawn long-lived processes (`cargo run`, `deno task`). Ensure traps stop them (`modules/pilot/launch_unit.sh` shows the pattern).
- **Workspace resets:** `tools/clean_workspace` wipes `work/` and relinks local ROS/Python packages. Run it (or `psh clean`) whenever package paths drift instead of editing `.cargo` configs manually.

## Useful references

- Fresh documentation: <https://fresh.deno.dev/docs>
- ROS 2 + Rust (`rclrs`): <https://github.com/sequenceplanner/rclrs>
- Create robot stack: <https://github.com/autonomylab/create_robot>
- MPU6050 ROS driver: <https://github.com/hiwad-aziz/ros2_mpu6050_driver>

Happy hacking!

## Miscellaneous
### Symlinks
* Each module has its own pilot folder, which is linked into the main Fresh app by `psh mod setup <module>`. This allows modules to declare their own UI components and pages without merging everything into a single codebase.
