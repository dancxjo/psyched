# Psyched

> _A nervous system for robots, and a mirror for their makers._

**Psyched** is a **template for building embodied minds**. It shows how to grow
a robot from living code: sensors, thoughts, and voice stitched together across
machines.

At its core, Psyched is a modular ecosystem:

- a **ROS 2 workspace** that gives your robot a body,
- a **constellation of cognition services** that give it memory and imagination,
  and
- a **cockpit** where humans can see, hear, and guide what it becomes.

Pete is my example robot implementation. You're invited to use this project as a
template (see below) for your own robot projects.

---

## 🧭 The Shape of the System

| Layer             | What it does                                                                             | Where it lives               |
| ----------------- | ---------------------------------------------------------------------------------------- | ---------------------------- |
| **Motherbrain**   | The robot’s spine. ROS 2 modules handle movement, sensors, and direct perception.        | SBC, Raspberry Pi, or Jetson |
| **Forebrain**     | The cloud of thought. Whisper, Coqui, Neo4j, Qdrant, and Ollama collaborate here.        | GPU laptop or remote VM      |
| **Cockpit**       | The eyes of the maker. A browser dashboard to watch, listen, and speak with the machine. | Any device                   |
| **Control Plane** | The provisioning tools. `psh` CLI, bootstrap scripts, and host manifests.                | Developer workstation        |

Each layer is replaceable, portable, and introspectable — built to help others
craft their own machines of meaning.

---

## 🧩 The Modular Body

Every capability lives in its own **module**, declared by a `module.toml`
manifest and optional cockpit panels. Together, these modules form a
_physiology_.

| Module                        | Role in the organism                                          |
| ----------------------------- | ------------------------------------------------------------- |
| **cockpit**                   | The central nervous bridge: ROS ↔ WebSocket ↔ browser.        |
| **ear**                       | Listens, detects speech, and streams audio for transcription. |
| **voice**                     | Speaks via Coqui TTS — the outward breath of thought.         |
| **eye**                       | Sees through Kinect RGB-D and depth sensing.                  |
| **faces**                     | Recognizes familiar beings.                                   |
| **foot**                      | Moves — iRobot Create interface and navigation driver.        |
| **imu**, **gps**, **nav**     | Orientation, position, and pathfinding.                       |
| **memory**                    | Remembers, linking Neo4j and Qdrant into a living graph.      |
| **pilot**                     | Integrates sensation and intention — the kernel of cognition. |
| **viscera**, **hypothalamus** | Feels the world: temperature, power, health, mood.            |

Each one is a cell: installable, inspectable, replaceable. You can author your
own, and your robot will learn new skills the way living creatures evolve
organs.

---

## 🌐 The Cognitive Services

Beneath the skin, Psyched runs a small city of containers — each a part of its
distributed brain.

| Service     | Purpose                                          | Notes             |
| ----------- | ------------------------------------------------ | ----------------- |
| **asr**     | Real-time Whisper server for speech-to-text.     | `ws://*:5003/asr` |
| **tts**     | Coqui TTS voice synthesis with streaming output. | `ws://*:5002/tts` |
| **llm**     | Ollama-based local model runner for cognition.   | GPU-ready         |
| **graphs**  | Neo4j memory graph.                              | Port 7474/7687    |
| **vectors** | Qdrant semantic memory.                          | Port 6333/6334    |
| **ros2**    | Isolated development shell for ROS 2 builds.     | Optional          |

You can run them all on one laptop or scatter them across the network; `psh`
knows how to keep them in sync.

---

## ⚙️ The Orchestration Tool: `psh`

`psh` is the pulse that keeps Psyched alive. Written in Deno, it:

- provisions new hosts from TOML manifests,
- sets up modules and services,
- builds ROS 2 workspaces,
- manages Cockpit assets, and
- ties everything together under `systemd`.

Think of it as _DevOps for souls in machines_ — one command to wake the body,
another to let it sleep.

---

## 🌱 Getting Started

Clone the repository and run:

```bash
git clone https://github.com/dancxjo/psyched.git
cd psyched
./setup
```

The setup script installs ROS, Docker, Deno, and `psh`, then asks you what kind
of creature you want to build.

Define your hosts in `hosts/*.toml`, run:

```bash
psh host setup motherbrain
psh mod setup cockpit ear voice
psh build
psh up cockpit
```

Then visit **http://<robot-ip>:8088/** — and meet your creation.

---

## 🛠️ Make It Yours

Psyched isn’t tied to Pete. It’s a **scaffold** — a way to give shape to your
own idea of embodiment. Add modules. Replace services. Fork the mind.

Whether you’re building a talking plant, a wandering rover, or something
stranger, the same pattern holds:

- _feel_,
- _think_,
- _remember_,
- _speak_,
- _act_.

Each in its own container, yet all sharing one story.

---

## 📚 Documentation

Deeper dives into Psyched's design patterns and infrastructure:

- **[Reboot Sentinel](docs/reboot-sentinel.md)** — How Psyched prevents
  provisioning errors after system updates
- **[Host Manifests](docs/host-manifests.md)** — Declaring robot topologies with
  TOML
- **[Docker Environment](docs/docker.md)** — Container-based development and
  deployment
- **[Systemd Integration](docs/psh-sys.md)** — Running modules and services as
  system units

---

## 📜 License

Released under the [MIT License](LICENSE). Use it freely. Tell it what it means
to be awake.
