# Host manifest guidelines

- Host profiles now live in TOML files. Populate the `[host]` table with
  `name`, `roles`, `installers`, `modules`, and `services` so it is obvious
  what runs where. When a `modules` array is present it is the canonical list
  of cockpit modules for that host—removing a name deactivates the module.
- Capture service metadata under `[config.srv.<name>]` tables. Include
  `intent`, `summary`, `runtime`, `ports`, and any socket data in
  `[[config.srv.<name>.endpoints]]` arrays.
- Module overrides live under `[config.mod.<name>]`. Store runtime
  configuration in nested `env` tables and capture launch arguments under
  `[config.mod.<name>.launch.arguments]` when possible. Having a config block
  alone does **not** activate the module; it must still be listed in
  `host.modules`.
- Record cross-host wiring with `[[config.mod.<name>.dependencies]]` arrays
  (reference dependencies as `<host>.<service>` and bind env vars with
  `bind_env`).
- Keep launch argument blocks unique; update existing tables instead of
  duplicating values so configuration remains easy to diff.
- Draft updates in an editor before overwriting files—redirects (`cat <<'EOF'`)
  won’t forgive stray placeholders.
