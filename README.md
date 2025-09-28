# psyche(d)

**psyched** is a modular cognition stack for embodied agents, combining **ROS 2** robotics with **LLM-driven perception and memory**.
It’s designed to run onboard a mobile robot (Pete: an iRobot Create 1 base) with a **split-brain architecture**:

* **Cerebellum (motherbrain)** → Raspberry Pi 4/5, runs ROS 2 stack, manages sensors and actuators.
* **Forebrain** → Headless laptop with GPU, mounted onboard, runs ASR/LLM/TTS in containers (no ROS 2).
* **Future memory** → Graph (Neo4j) + vector embeddings (Qdrant) integrate multimodal experience into long-term memory.

---

## 🧩 Architecture

### Cerebellum (Raspberry Pi, ROS 2 Jazzy)

* Runs the ROS 2 packages from `modules/*`.
* Handles low-level IO: IMU, wheel odometry, lidar, Kinect, microphones, speakers.
* Systemd units under `hosts/cerebellum/systemd/` ensure persistent operation.
* Key modules:

  * **ear**: audio capture, silence detection, Whisper transcription.
  * **voice**: text-to-speech playback.
  * **chat**: coordinates conversation with Ollama responses.
  * **pilot**: LCARS-style web control interface.
  * **foot/nav/imu/eye/gps/wifi/will**: navigation and sensing.

### Forebrain (headless laptop, GPU)

* Runs `psh speech up` → Docker Compose stack (`compose/speech-stack.compose.yml`):

  * **ASR** → whisper server for longer or higher-quality transcription.
  * **TTS** → Coqui/Piper server for natural speech output.
  * **LLM** → Ollama or gpt-oss:20b for high-quality reasoning.
* Communicates with cerebellum via WebSockets and topic bridges.
* Offloads heavy computation so the Pi can stay real-time and lightweight.

### Memory Integration (planned)

* **Vector memory** (Qdrant): store embeddings of sensory summaries, conversation, and observations.
* **Graph memory** (Neo4j): link experiences into episodes, relationships, and contexts.
* Together → support recall, clustering, and narrative continuity.

---

## 🎛 Data Flow

1. **Ear (cerebellum)** captures microphone input → VAD → transcription.
2. **Chat (cerebellum)** sends text to **LLM (forebrain)** → receives responses.
3. **Voice (cerebellum)** synthesizes responses using **TTS (forebrain)** or local espeak.
4. **Pilot** provides a web dashboard for control and monitoring.
5. (Planned) Graph + vector memory log every sensation and impression for long-term learning.

---

## 🚀 Quickstart

### 1. On the cerebellum (Raspberry Pi)

```bash
psh provision cerebellum
psh install
psh systemd enable
```

### 2. On the forebrain (GPU laptop)

```bash
psh speech up   # starts ASR, TTS, LLM containers
```

### 3. Back on the cerebellum

```bash
psh launch ear
psh launch voice
psh launch chat
psh launch pilot
```

### 4. Connect to the Pilot web UI

Visit `http://<cerebellum-host>:8080` in your browser.

---

## 📖 Status

* ✅ ROS 2 stack runs on Pi (“cerebellum”).
* ✅ GPU laptop (“forebrain”) runs ASR/LLM/TTS containers.
* ✅ Systemd + `psh` CLI orchestrate installation and services.
* 🔄 Graph + vector memory integration in progress.

