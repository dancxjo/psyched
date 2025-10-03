# Copilot usage notes for `psyched`

These hints help GitHub Copilot generate higher-quality suggestions inside this repository.

## Project summary

- Robotics control stack for the robot **Pete**.
- Python + ROS 2 backend (rosbridge/web_video_server) providing the cockpit websocket bridge.
- Deno Fresh + Preact frontend (`modules/pilot/frontend`).
- `psh` Rust CLI orchestrates host provisioning and module lifecycles.
- Modules live in `modules/<name>` and may expose pilot UI overlays under `pilot/`.

## Authoring conventions

- **Rust:** Use `anyhow` for fallible operations and `tracing` for logging. Prefer async code with Tokio; avoid blocking in async contexts. Keep ROS topic names as constants or document them near the websocket bridge.
- **TypeScript/Preact:** Use hooks (`useState`, `useEffect`) from `preact/hooks`. Guard browser-only APIs for SSR. Export hooks/components via barrel files under `modules/pilot/frontend/lib` or `components`.
- **Shell:** Begin scripts with `#!/usr/bin/env bash` and `set -euo pipefail`. Clean up background processes with `trap`.
- **TOML manifests:** Keep keys alphabetized where practical and include inline comments when referencing external repositories.

## When suggesting code

1. Reuse the existing cockpit client (`lib/cockpit.ts`) for websocket access—do not hand-roll new clients.
2. Route module UI assets through `modules/<name>/pilot/...` so `psh mod setup` can symlink them automatically.
3. Prefer the structured `psh` commands instead of ad-hoc shell pipelines when orchestrating modules.
4. Surface new configuration knobs in docs (`README.md`, module-specific READMEs) and update sample commands.

## Testing expectations

- Run `cargo fmt --manifest-path psh/Cargo.toml` + `cargo check --manifest-path psh/Cargo.toml` after Rust edits to the `psh` CLI.
- For Fresh components, run `deno fmt`, `deno check`, and (when available) `deno test`.
- ROS changes typically require `colcon build` and sourcing `install/setup.bash`.

Copilot should default to these patterns unless the surrounding file dictates otherwise. When in doubt, add TODO comments describing open questions rather than guessing hardware-specific details.
