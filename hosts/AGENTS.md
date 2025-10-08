# Host manifest guidelines

- Keep manifests declarative: list `installers`, `modules`, and `services` directly under `[host]` so it is obvious what runs where.
- Populate `roles` to describe what each host is responsible for (e.g. `compute`, `ros2-control`).
- Describe services with `[services.<name>]` tables; include `intent`, `summary`, `runtime`, a quick `ports = [...]` list, and enumerate socket metadata under an `endpoints = [{ ... }]` array.
- Declare modules with `[modules.<name>]`, store runtime configuration in `env = { ... }`, and capture launch arguments via `launch = { ... }` inline tables.
- Record cross-host wiring with `dependencies = [{ ... }]` arrays (reference dependencies as `<host>.<service>` and bind env vars with `bind_env`).
- Keep module launch argument blocks unique; update existing tables instead of duplicating values so configuration remains easy to diff.
- Draft updates in an editor before overwriting files—redirects (`cat <<'EOF'`) won’t forgive stray placeholders.
