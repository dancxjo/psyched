# Host manifest guidelines

- Host profiles now live in JSON (or YAML) files. Populate the top-level `host`
  object with `name`, `roles`, `installers`, `modules`, and `services` so it is
  obvious what runs where.
- Capture service metadata under `services.<name>` objects. Include `intent`,
  `summary`, `runtime`, `ports`, and any socket data in an `endpoints` array.
- Module overrides live under `modules.<name>`. Store runtime configuration in
  an `env` object and capture launch arguments under `launch.arguments` when
  possible.
- Record cross-host wiring with `dependencies` arrays (reference dependencies as
  `<host>.<service>` and bind env vars with `bind_env`).
- Keep module launch argument blocks unique; update existing objects instead of
  duplicating values so configuration remains easy to diff.
- Draft updates in an editor before overwriting files—redirects (`cat <<'EOF'`)
  won’t forgive stray placeholders.
