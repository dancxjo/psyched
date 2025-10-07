# Psyched Pilot UI (Fresh)

This directory hosts the operator interface for Psyched powered by
[Fresh](https://fresh.deno.dev/) and [Deno](https://deno.land/).

## Getting started

Install Deno (the PSH setup flow now installs it automatically on provision).
Then run the development server:

```bash
deno task dev
```

Open http://localhost:8000 to explore the pilot console.

When new routes or islands are linked in by `psh`, regenerate the manifest with:

```bash
deno task manifest
```

## Project layout

- `main.ts` – Fresh server entry point.
- `routes/` – HTTP route handlers and page components.
  - `modules/imu.tsx` – Read-only IMU telemetry view (also exposed under
    `/modules/imu`).
- `components/` – Shared UI components such as `ImuReadout`.
- `modules/<module>/components|routes|static|islands` – Symlinked entry points
  per module for pilot integration.
- `static/` – Static assets served as-is.

## Linking modules to pilot UI

Each module can expose pilot controls by placing files under
`modules/<name>/pilot/{components,routes,static,islands}`. Prepare the overlay
with:

```bash
psh setup-module <name>
```

When a module is torn down, run:

```bash
psh teardown-module <name>
```

These commands manage the symlinks into the Fresh project so the assets are
available to the pilot runtime without touching the launch/shutdown lifecycle.
