# Psyched Pilot UI

This Fresh 2 + Vite workspace hosts the operator console for Psyched. The
cockpit pulls ROS telemetry over the websocket bridge served by the
`pilot_cockpit` Python package and reuses PSH helpers for orchestration tasks.

## Commands

- `deno task dev` – run the Vite-powered dev server with hot reloading
- `deno task build` – create a production bundle in `.fresh`
- `deno task check` – format check, lint, and type-check the codebase

## Cockpit websocket signals

The shared websocket client in `lib/cockpit.ts` mirrors its connection state to
signals exposed by `lib/cockpit_signals.ts`. Prefer consuming those signals over
rolling ad-hoc polling logic—any island or utility can react to
`cockpitConnectionStatus`/`cockpitHasError` without prop drilling. See the
module's inline example for a quick usage refresher.

## Module overlays

Use `psh mod setup <name>` to link a module's `pilot/` directory into this
frontend. The CLI automatically regenerates Fresh manifests after linking or
tearing down overlays, so you rarely need manual intervention.

Symlinks land under `components/`, `islands/`, `routes/`, and `static/`. Avoid
creating real files that would conflict with those paths—let `psh` manage them.
