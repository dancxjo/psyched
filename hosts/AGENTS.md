# Host manifest guidelines

- Keep manifests declarative: `installers` arrays replace ad-hoc `[features]` toggles.
- Populate `roles` to describe what each host is responsible for (e.g. `compute`, `ros2-control`).
- Describe services with `[[services.<name>]]` entries; include `intent`, `summary`, `runtime`, and a quick `ports = [...]` list so orchestration tooling can render useful dashboards.
- Declare externally reachable sockets under `[[services.<name>.ports]]` and capture `bind`, `port`, `target`, and `advertise` values so consumers know how to connect.
- Declare modules with `[modules.<name>]`, add runtime configuration in `[modules.<name>.env]`, and capture cross-host wiring with `[[modules.<name>.dependencies]]` (reference dependencies as `<host>.<service>` and bind env vars with `bind_env`).
- Draft updates in an editor before overwriting files—redirects (`cat <<'EOF'`) won’t forgive stray placeholders.
