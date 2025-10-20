# Host manifest schema

Host manifests live in `hosts/*.json` (or `.yaml`) and describe how Pete's
distributed stack is orchestrated. The layout is intentionally declarative so
tooling can reason about installers, runtime roles, and cross-host wiring
without relying on bespoke shell scripts.

## Top-level objects

| Key | Purpose |
| --- | ------- |
| `host` | Identifies the machine, lists the `installers`, `modules`, and `services` it owns, and summarizes responsibilities via `roles`. |
| `services.<name>` | Declares a service (HTTP API, websocket, etc.) hosted on the machine. |
| `modules.<name>` | Configures a ROS module or other workload that should run locally. |

## Host metadata

Keep the host object compact and explicit:

```json
{
  "host": {
    "name": "motherbrain",
    "roles": ["ros2"],
    "installers": ["ros2", "docker"],
    "modules": ["cockpit", "ear"],
    "services": ["asr"]
  }
}
```

The `ros2` installer provisions a Docker-based ROS 2 helper by default, so pair
it with the `docker` installer (or override `[config.installer.ros2] mode = "native"`
for the legacy apt flow).

## Services

Describe each service under a single object. Capture a quick port summary and
the detailed socket metadata in an `endpoints` array so orchestration tooling
can map dependencies without chasing nested tables.

```json
{
  "services": {
    "asr": {
      "intent": "speech-to-text",
      "summary": "Streaming Whisper-based ASR over websockets",
      "runtime": "container",
      "ports": [5003],
      "endpoints": [
        {
          "name": "ws",
          "protocol": "ws",
          "bind": "0.0.0.0",
          "port": 5003,
          "target": 5003,
          "advertise": "ws://forebrain.local:5003/asr",
          "description": "Websocket endpoint consumed by motherbrain.ear"
        }
      ]
    }
  }
}
```

- `intent` captures what role the service plays in the overall stack.
- `summary` is a human-readable description that surfaces nicely in dashboards.
- `runtime` hints at the orchestration strategy (for example `container`,
  `systemd`, or `bare`).
- `ports` gives the quick list of TCP/UDP numbers the service claims.
- Each entry in `endpoints` documents a public socket:
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
each module under `modules.<name>` and place module-specific configuration under
compact objects so the manifest stays flat.

```json
{
  "modules": {
    "ear": {
      "launch": { "enabled": true },
      "env": {
        "EAR_BACKEND": "service",
        "EAR_SERVICE_URI": "ws://forebrain.local:5003/asr"
      }
    }
  }
}
```

ROS 2 domain IDs are configured globally via `config/ros_domain_id`, so individual modules typically do not need to set `ROS_DOMAIN_ID`.

When you need to provide module-specific launch arguments, embed an `arguments`
object so deeply nested structures remain unnecessary:

```json
{
  "modules": {
    "nav": {
      "launch": {
        "arguments": {
          "kinect_rgb_topic": "/camera/color/image_raw",
          "kinect_depth_topic": "/camera/depth/image_rect_raw",
          "camera_frame": "camera_link"
        }
      }
    }
  }
}
```

### Declaring dependencies

When a module relies on a remote service, add entries to the `dependencies`
array.
Reference the dependency by `<host>.<service>` and bind each module-specific
environment variable so orchestration can audit wiring without guessing
endpoints.

```json
{
  "modules": {
    "ear": {
      "dependencies": [
        {
          "service": "forebrain.asr",
          "port": "ws",
          "via": "ws://forebrain.local:5003/asr",
          "bind_env": "EAR_SERVICE_URI",
          "summary": "Consumes Whisper websocket for speech recognition"
        }
      ]
    }
  }
}
```

Tooling can now join module dependencies with service exports to validate port
assignments, surface connection metadata (for example environment variables),
and render cross-host diagrams without duplicating socket definitions.
