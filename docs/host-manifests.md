# Host manifest schema

Host manifests live in `hosts/*.toml` and describe how Pete's distributed stack
is orchestrated. The layout is intentionally declarative so tooling can reason
about installers, runtime roles, and cross-host wiring without relying on
bespoke shell scripts.

## Top-level tables

| Table         | Purpose                                                                                     |
| ------------- | ------------------------------------------------------------------------------------------- |
| `[host]`      | Identifies the machine and summarizes its responsibilities via the `roles` array.           |
| `[provision]` | Lists the `installers` that `psh host setup` should run. Use this instead of feature flags. |

## Services

Services are grouped by name so manifests read declaratively at a glance. Use
`[[services.<name>]]` to define one or more runtime targets for that service.
Each entry should advertise what capability it fulfils, plus the sockets it
exposes via `[[services.<name>.ports]]`. Keep a short `ports = [...]` list
alongside the detailed metadata so humans can spot the numbers immediately.

```toml
[[services.asr]]
intent = "speech-to-text"
summary = "Streaming Whisper-based ASR over websockets"
runtime = "container"
ports = [5003]

  [[services.asr.ports]]
  name = "ws"
  protocol = "ws"
  bind = "0.0.0.0"
  port = 5003
  target = 5003
  advertise = "ws://forebrain.local:5003/asr"
  description = "Websocket endpoint consumed by motherbrain.ear"
```

- `intent` captures what role the service plays in the overall stack.
- `summary` is a human-readable description that surfaces nicely in dashboards.
- `runtime` hints at the orchestration strategy (for example `container`,
  `systemd`, or `bare`).
- `ports` gives the quick list of TCP/UDP numbers the service claims.
- Each `[[services.<name>.ports]]` block documents a public socket:
  - `name` is how dependent modules refer to the port.
  - `protocol` is the transport (`http`, `ws`, `grpc`, `bolt`, ...).
  - `bind` is the local address to listen on (default `0.0.0.0`).
  - `port` is the externally reachable port number.
  - `target` records the in-container port when a proxy or NAT is involved.
  - `advertise` is the externally reachable URI for operators and dependent
    modules.
  - `description` provides any extra context that tooling cannot infer
    automatically.

## Modules

Modules run on hosts that provision ROS 2 nodes or other local workloads. Define
each module under its own table, `[modules.<name>]`, and place module-specific
environment configuration under `[modules.<name>.env]`.

````toml
[modules.ear]
launch = true

  [modules.ear.env]
  ROS_DOMAIN_ID = "25"
  EAR_BACKEND = "service"
  EAR_SERVICE_URI = "ws://forebrain.local:5003/asr"

When you need to provide module-specific launch arguments, wrap the flag in an
inline table so nested tables remain valid TOML:

```toml
[modules.nav]
launch = { enabled = true }

  [modules.nav.launch.arguments]
  kinect_rgb_topic = "/camera/color/image_raw"
  kinect_depth_topic = "/camera/depth/image_rect_raw"
  camera_frame = "camera_link"
````

````
### Declaring dependencies

When a module relies on a remote service, add `[[modules.<name>.dependencies]]` entries. Reference the dependency by `<host>.<service>` and bind each module-specific environment variable so orchestration can audit wiring without guessing endpoints.

```toml
  [[modules.ear.dependencies]]
  service = "forebrain.asr"
  port = "ws"
  via = "ws://forebrain.local:5003/asr"
  bind_env = "EAR_SERVICE_URI"
  summary = "Consumes Whisper websocket for speech recognition"
````

Tooling can now join module dependencies with service exports to validate port
assignments, surface connection metadata (for example environment variables),
and render cross-host diagrams without duplicating socket definitions.
