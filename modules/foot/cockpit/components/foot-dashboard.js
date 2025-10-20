import {
  css,
  html,
  LitElement,
} from "https://unpkg.com/lit@3.1.4/index.js?module";
import { callRosService, createTopicSocket } from "/js/cockpit.js";
import { surfaceStyles } from "/components/cockpit-style.js";
import "/components/joystick-control.js";

import {
  buildParameterRequest,
  buildPowerLedPayload,
  buildDefineSongPayload,
  formatJointState,
  formatOdometry,
  formatParameterEvent,
  SONG_LIMITS,
  parseSongSheet,
  toAsciiPayload,
} from "./foot-dashboard.helpers.js";

function createFootSocket(options, onError) {
  const socket = createTopicSocket({ module: "foot", ...options });
  const handler = typeof onError === "function" ? onError : () => undefined;
  socket.ready.catch((error) => {
    try {
      handler(error, options);
    } catch (callbackError) {
      console.warn("Foot socket error handler threw", callbackError);
    }
  });
  return socket;
}

const BATTERY_TOPICS = [
  { topic: "battery/charge", type: "std_msgs/msg/Float32", key: "charge" },
  { topic: "battery/capacity", type: "std_msgs/msg/Float32", key: "capacity" },
  { topic: "battery/charge_ratio", type: "std_msgs/msg/Float32", key: "ratio" },
  { topic: "battery/voltage", type: "std_msgs/msg/Float32", key: "voltage" },
  { topic: "battery/current", type: "std_msgs/msg/Float32", key: "current" },
  {
    topic: "battery/temperature",
    type: "std_msgs/msg/Int16",
    key: "temperature",
  },
  {
    topic: "battery/charging_state",
    type: "create_msgs/msg/ChargingState",
    key: "charging",
  },
];

const SENSOR_TOPICS = [
  { topic: "bumper", type: "create_msgs/msg/Bumper", handler: "updateBumper" },
  { topic: "cliff", type: "create_msgs/msg/Cliff", handler: "updateCliff" },
  {
    topic: "wheeldrop",
    type: "std_msgs/msg/Empty",
    handler: "updateWheelDrop",
  },
];

const BUTTON_TOPICS = [
  { topic: "clean_button", label: "Clean button" },
  { topic: "spot_button", label: "Spot button" },
  { topic: "dock_button", label: "Dock button" },
  { topic: "day_button", label: "Day button" },
  { topic: "hour_button", label: "Hour button" },
  { topic: "minute_button", label: "Minute button" },
];

const TELEMETRY_TOPICS = [
  { topic: "ir_omni", type: "std_msgs/msg/UInt16", handler: "updateIrOmni" },
  {
    topic: "joint_states",
    type: "sensor_msgs/msg/JointState",
    handler: "updateJointStates",
  },
  { topic: "mode", type: "create_msgs/msg/Mode", handler: "updateMode" },
  { topic: "odom", type: "nav_msgs/msg/Odometry", handler: "updateOdometry" },
  {
    topic: "main_brush_motor",
    type: "std_msgs/msg/Float32",
    handler: "updateMainBrushMotor",
  },
  {
    topic: "side_brush_motor",
    type: "std_msgs/msg/Float32",
    handler: "updateSideBrushMotor",
  },
  {
    topic: "vacuum_motor",
    type: "std_msgs/msg/Float32",
    handler: "updateVacuumMotor",
  },
  {
    topic: "diagnostics",
    type: "diagnostic_msgs/msg/DiagnosticArray",
    handler: "updateDiagnostics",
  },
  {
    topic: "parameter_events",
    type: "rcl_interfaces/msg/ParameterEvent",
    handler: "updateParameterEvent",
  },
  { topic: "tf", type: "tf2_msgs/msg/TFMessage", handler: "updateTf" },
  {
    topic: "tf_static",
    type: "tf2_msgs/msg/TFMessage",
    handler: "updateTfStatic",
  },
];

const LED_TOPICS = {
  debris: { topic: "debris_led", type: "std_msgs/msg/Bool" },
  spot: { topic: "spot_led", type: "std_msgs/msg/Bool" },
  dock: { topic: "dock_led", type: "std_msgs/msg/Bool" },
  check: { topic: "check_led", type: "std_msgs/msg/Bool" },
};

const SIMPLE_COMMANDS = {
  dock: { topic: "dock", type: "std_msgs/msg/Empty", payload: {} },
  undock: { topic: "undock", type: "std_msgs/msg/Empty", payload: {} },
};

const DEFAULT_PARAMETER_FORM = {
  node: "/create_driver",
  dev: "/dev/ttyUSB0",
  base_frame: "base_footprint",
  odom_frame: "odom",
  latch_cmd_duration: "0.2",
  loop_hz: "10.0",
  publish_tf: "true",
  robot_model: "CREATE_2",
  baud: "",
  oi_mode_workaround: "false",
  desc: "true",
  config: "",
};

const DEFAULT_SONG_FORM = {
  id: "0",
  notes: "60,0.25\n64,0.25\n67,0.5",
};

function toNumber(value) {
  if (value == null) return null;
  const coerced = Number(value);
  return Number.isFinite(coerced) ? coerced : null;
}

function clampPercent(value) {
  const numeric = toNumber(value);
  if (numeric == null) return null;
  return Math.min(100, Math.max(0, numeric));
}

function chargingStatus(message) {
  if (!message || typeof message !== "object") {
    return { label: "Unknown", state: "idle" };
  }
  if ("state" in message && message.state) {
    const label = message.state === 2 ? "Charging" : String(message.state);
    const normalized = label.toLowerCase();
    if (normalized.includes("charge") || normalized.includes("dock")) {
      return { label, state: "charging" };
    }
    return { label, state: "idle" };
  }
  if (message.is_charging === true) {
    return { label: "Charging", state: "charging" };
  }
  return { label: "Discharging", state: "idle" };
}

function percentFromCharge(charge, capacity) {
  const chargeVal = toNumber(charge);
  const capacityVal = toNumber(capacity);
  if (chargeVal == null || capacityVal == null || capacityVal <= 0) {
    return null;
  }
  return clampPercent((chargeVal / capacityVal) * 100);
}

class FootDashboard extends LitElement {
  static properties = {
    battery: { state: true },
    sensors: { state: true },
    telemetry: { state: true },
    parameterForm: { state: true },
    parameterStatus: { state: true },
    parameterBusy: { state: true },
    displayState: { state: true },
    songForm: { state: true },
    songStatus: { state: true },
    actionStatus: { state: true },
    lastCommand: { state: true },
    buttonLog: { state: true },
    parameterLog: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      .battery-gauge {
        position: relative;
        height: 16px;
        border-radius: 999px;
        background: rgba(255, 255, 255, 0.08);
        overflow: hidden;
      }
      .battery-gauge__fill {
        position: absolute;
        inset: 0;
        background: linear-gradient(
          90deg,
          var(--lcars-accent),
          var(--lcars-success)
        );
        transform-origin: left center;
        transition: transform 180ms ease;
      }
      .drive-panel {
        display: flex;
        flex-direction: column;
        gap: 1rem;
        align-items: center;
      }
      .drive-panel__log {
        width: 100%;
        max-height: 120px;
        overflow: auto;
        font-size: 0.8rem;
        box-sizing: border-box;
      }
      .sensor-list {
        list-style: none;
        margin: 0;
        padding: 0;
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
      }
      .sensor-item {
        display: flex;
        align-items: center;
        justify-content: space-between;
        padding: 0.55rem 0.75rem;
        border-radius: 0.5rem;
        background: rgba(255, 255, 255, 0.05);
        font-size: 0.85rem;
      }
      .sensor-item[data-state="alert"] {
        background: rgba(255, 111, 97, 0.2);
        color: var(--lcars-error);
      }
      .sensor-item__dot {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        background: var(--lcars-accent-secondary);
      }
      .sensor-item[data-state="alert"] .sensor-item__dot {
        background: var(--lcars-error);
        box-shadow: 0 0 12px var(--lcars-error);
      }
      .surface-actions--grid {
        grid-template-columns: repeat(auto-fit, minmax(110px, 1fr));
      }
      .led-grid {
        display: grid;
        gap: 0.75rem;
        grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
      }
      .telemetry-grid {
        display: grid;
        gap: 0.75rem;
      }
      .log-list {
        list-style: none;
        margin: 0;
        padding: 0;
        display: flex;
        flex-direction: column;
        gap: 0.4rem;
        max-height: 260px;
        overflow-y: auto;
      }
      .log-entry {
        background: rgba(255, 255, 255, 0.05);
        border-radius: 0.5rem;
        padding: 0.5rem 0.75rem;
        font-size: 0.8rem;
        display: grid;
        gap: 0.25rem;
      }
      .log-entry__meta {
        display: flex;
        justify-content: space-between;
        color: var(--lcars-muted);
        font-size: 0.75rem;
      }
      form {
        display: grid;
        gap: 0.75rem;
      }
      label {
        display: flex;
        flex-direction: column;
        gap: 0.35rem;
        font-size: 0.75rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--metric-label-color);
      }
      input,
      textarea,
      select {
        font: inherit;
        padding: 0.55rem 0.75rem;
        border-radius: 0.5rem;
        border: 1px solid var(--control-surface-border);
        background: rgba(0, 0, 0, 0.3);
        color: var(--lcars-text);
      }
      textarea {
        min-height: 120px;
        resize: vertical;
      }
      fieldset {
        border: 1px solid rgba(255, 255, 255, 0.08);
        border-radius: 0.75rem;
        padding: 0.75rem;
      }
      fieldset legend {
        padding: 0 0.5rem;
        font-size: 0.75rem;
        text-transform: uppercase;
        letter-spacing: 0.05em;
        color: var(--lcars-muted);
      }
    `,
  ];

  constructor() {
    super();
    this.battery = {
      percent: 0,
      chargeDisplay: "—",
      capacityDisplay: "—",
      ratioDisplay: "—",
      voltageDisplay: "—",
      currentDisplay: "—",
      temperatureDisplay: "—",
      status: "Awaiting telemetry…",
      state: "idle",
    };
    this.sensors = {
      bumpers: false,
      cliffs: false,
      wheels: false,
    };
    this.telemetry = {
      mode: "Unknown",
      irOmni: "—",
      joint: "joint state unavailable",
      odom: "odometry unavailable",
      tf: { last: "Never", count: 0 },
      tfStatic: { last: "Never", count: 0 },
      mainBrush: "—",
      sideBrush: "—",
      vacuum: "—",
      diagnostics: "Awaiting diagnostic messages…",
    };
    this.parameterForm = { ...DEFAULT_PARAMETER_FORM };
    this.parameterStatus = "";
    this.parameterBusy = false;
    this.displayState = {
      ascii: "PETE",
      powerColor: 128,
      powerIntensity: 255,
      ledOverrides: {
        debris: false,
        spot: false,
        dock: false,
        check: false,
      },
    };
    this.songForm = { ...DEFAULT_SONG_FORM };
    this.songStatus = "";
    this.actionStatus = "";
    this.lastCommand = "{ }";
    this.buttonLog = [];
    this.parameterLog = [];
    this.sockets = [];
    this.publishers = new Map();
    this.cmdVelPublisher = null;
    this.connectionWarnings = new Set();
  }

  connectedCallback() {
    super.connectedCallback();
    this.subscribeBattery();
    this.subscribeSensors();
    this.subscribeButtons();
    this.subscribeTelemetry();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this.shutdown();
  }

  shutdown() {
    for (const socket of this.sockets) {
      try {
        socket.close();
      } catch (_error) {
        // ignore
      }
    }
    this.sockets.length = 0;
    this.publishers.clear();
    this.cmdVelPublisher = null;
  }

  subscribeBattery() {
    const state = {};
    for (const entry of BATTERY_TOPICS) {
      const socket = createFootSocket(
        { topic: entry.topic, type: entry.type, role: "subscribe" },
        (error) => this.handleSocketError(entry.topic, entry.type, error),
      );
      socket.addEventListener("message", (event) => {
        try {
          const payload = JSON.parse(event.data);
          if (payload.event !== "message") {
            return;
          }
          state[entry.key] = payload.data;
          this.updateBattery(state);
        } catch (error) {
          console.warn("Failed to parse battery payload", error);
        }
      });
      this.sockets.push(socket);
    }
  }

  updateBattery(state) {
    const charge = toNumber(state.charge?.data ?? state.charge);
    const capacity = toNumber(state.capacity?.data ?? state.capacity);
    const ratio = toNumber(state.ratio?.data ?? state.ratio);
    const voltage = toNumber(state.voltage?.data ?? state.voltage);
    const current = toNumber(state.current?.data ?? state.current);
    const temperature = toNumber(state.temperature?.data ?? state.temperature);
    const percent = ratio != null
      ? clampPercent(ratio * 100)
      : percentFromCharge(charge, capacity);
    const charging = chargingStatus(state.charging);

    this.battery = {
      percent: percent ?? 0,
      chargeDisplay: charge != null ? `${charge.toFixed(2)} Ah` : "—",
      capacityDisplay: capacity != null ? `${capacity.toFixed(2)} Ah` : "—",
      ratioDisplay: percent != null ? `${(percent / 100).toFixed(2)}` : "—",
      voltageDisplay: voltage != null ? `${voltage.toFixed(2)} V` : "—",
      currentDisplay: current != null ? `${current.toFixed(2)} A` : "—",
      temperatureDisplay: temperature != null
        ? `${temperature.toFixed(1)} °C`
        : "—",
      status: percent != null
        ? `${percent.toFixed(0)}% · ${charging.label}`
        : charging.label,
      state: percent != null && percent < 20 ? "critical" : charging.state,
    };
  }

  subscribeSensors() {
    for (const entry of SENSOR_TOPICS) {
      const socket = createFootSocket(
        { topic: entry.topic, type: entry.type, role: "subscribe" },
        (error) => this.handleSocketError(entry.topic, entry.type, error),
      );
      socket.addEventListener("message", (event) => {
        try {
          const payload = JSON.parse(event.data);
          if (payload.event !== "message") return;
          const handler = this[entry.handler];
          if (typeof handler === "function") {
            handler.call(this, payload.data);
          }
        } catch (error) {
          console.warn("Failed to parse sensor payload", error);
        }
      });
      this.sockets.push(socket);
    }
  }

  subscribeButtons() {
    for (const entry of BUTTON_TOPICS) {
      const socket = createFootSocket(
        { topic: entry.topic, type: "std_msgs/msg/Empty", role: "subscribe" },
        (error) =>
          this.handleSocketError(entry.topic, "std_msgs/msg/Empty", error),
      );
      socket.addEventListener("message", () => {
        this.pushButtonEvent(entry.label, entry.topic);
      });
      this.sockets.push(socket);
    }
  }

  subscribeTelemetry() {
    for (const entry of TELEMETRY_TOPICS) {
      const socket = createFootSocket(
        { topic: entry.topic, type: entry.type, role: "subscribe" },
        (error) => this.handleSocketError(entry.topic, entry.type, error),
      );
      socket.addEventListener("message", (event) => {
        try {
          const payload = JSON.parse(event.data);
          if (payload.event !== "message") {
            return;
          }
          const handler = this[entry.handler];
          if (typeof handler === "function") {
            handler.call(this, payload.data);
          }
        } catch (error) {
          console.warn("Failed to parse telemetry payload", error);
        }
      });
      this.sockets.push(socket);
    }
  }

  updateBumper(message) {
    const candidates = [
      "is_left_pressed",
      "is_right_pressed",
      "is_center_pressed",
    ];
    this.sensors = {
      ...this.sensors,
      bumpers: candidates.some((key) => Boolean(message?.[key])),
    };
  }

  updateCliff(message) {
    const candidates = [
      "is_left_detected",
      "is_right_detected",
      "is_front_left",
      "is_front_right",
    ];
    this.sensors = {
      ...this.sensors,
      cliffs: candidates.some((key) => Boolean(message?.[key])),
    };
  }

  updateWheelDrop(_message) {
    this.sensors = { ...this.sensors, wheels: true };
    setTimeout(() => {
      this.sensors = { ...this.sensors, wheels: false };
    }, 2000);
  }

  updateIrOmni(message) {
    const code = Number(message?.data ?? message);
    this.telemetry = {
      ...this.telemetry,
      irOmni: Number.isFinite(code) && code > 0
        ? `IR code ${code}`
        : "No IR signal",
    };
  }

  updateJointStates(message) {
    this.telemetry = {
      ...this.telemetry,
      joint: formatJointState(message),
    };
  }

  updateMode(message) {
    const label = message?.mode ?? message?.data ?? "Unknown";
    this.telemetry = {
      ...this.telemetry,
      mode: String(label),
    };
  }

  updateOdometry(message) {
    this.telemetry = {
      ...this.telemetry,
      odom: formatOdometry(message),
    };
  }

  updateMainBrushMotor(message) {
    const value = toNumber(message?.data ?? message);
    this.telemetry = {
      ...this.telemetry,
      mainBrush: value != null ? `${value.toFixed(2)}%` : "—",
    };
  }

  updateSideBrushMotor(message) {
    const value = toNumber(message?.data ?? message);
    this.telemetry = {
      ...this.telemetry,
      sideBrush: value != null ? `${value.toFixed(2)}%` : "—",
    };
  }

  updateVacuumMotor(message) {
    const value = toNumber(message?.data ?? message);
    this.telemetry = {
      ...this.telemetry,
      vacuum: value != null ? `${value.toFixed(2)}%` : "—",
    };
  }

  updateDiagnostics(message) {
    const status = Array.isArray(message?.status) && message.status.length
      ? message.status
        .map((entry) => `${entry.name ?? "Unknown"}: ${entry.message ?? ""}`)
        .join("\n")
      : "Awaiting diagnostic messages…";
    this.telemetry = {
      ...this.telemetry,
      diagnostics: status,
    };
  }

  updateParameterEvent(message) {
    const entry = {
      id: crypto.randomUUID ? crypto.randomUUID() : `evt-${Date.now()}`,
      time: new Date().toLocaleTimeString(),
      detail: formatParameterEvent(message),
    };
    this.parameterLog = [entry, ...this.parameterLog].slice(0, 16);
  }

  updateTf(message) {
    const count = Array.isArray(message?.transforms)
      ? message.transforms.length
      : 0;
    this.telemetry = {
      ...this.telemetry,
      tf: {
        last: new Date().toLocaleTimeString(),
        count,
      },
    };
  }

  updateTfStatic(message) {
    const count = Array.isArray(message?.transforms)
      ? message.transforms.length
      : 0;
    this.telemetry = {
      ...this.telemetry,
      tfStatic: {
        last: new Date().toLocaleTimeString(),
        count,
      },
    };
  }

  pushButtonEvent(label, topic) {
    const entry = {
      id: crypto.randomUUID
        ? crypto.randomUUID()
        : `btn-${Date.now()}-${Math.random()}`,
      label,
      topic,
      time: new Date().toLocaleTimeString(),
    };
    this.buttonLog = [entry, ...this.buttonLog].slice(0, 20);
  }

  handleSocketError(topic, type, error) {
    const detail = error instanceof Error ? error.message : String(error);
    const key = `${topic}:${type}:${detail}`;
    if (this.connectionWarnings.has(key)) {
      return;
    }
    this.connectionWarnings.add(key);
    if (detail.includes("No module named 'create_msgs'")) {
      this.actionStatus =
        "Foot telemetry unavailable: create_msgs is not installed. Run `psh mod setup foot` or rebuild the foot workspace.";
      return;
    }
    this.actionStatus = `Foot stream error for ${topic}: ${detail}`;
  }

  ensurePublisher(topic, type) {
    const key = `${topic}:${type}`;
    if (this.publishers.has(key)) {
      return this.publishers.get(key);
    }
    const socket = createFootSocket(
      { topic, type, role: "publish" },
      (error) => this.handleSocketError(topic, type, error),
    );
    this.publishers.set(key, socket);
    this.sockets.push(socket);
    return socket;
  }

  firstUpdated() {
    const joystick = this.shadowRoot.querySelector("cockpit-joystick-control");
    if (joystick) {
      this.cmdVelPublisher = this.ensurePublisher(
        "cmd_vel",
        "geometry_msgs/msg/Twist",
      );
      joystick.record = {
        send: (message) => this.sendVelocity(message),
      };
    }
  }

  sendVelocity(message) {
    try {
      if (!this.cmdVelPublisher) {
        this.cmdVelPublisher = this.ensurePublisher(
          "cmd_vel",
          "geometry_msgs/msg/Twist",
        );
      }
      this.cmdVelPublisher.send(JSON.stringify(message));
      this.lastCommand = JSON.stringify(message, null, 2);
    } catch (error) {
      this.actionStatus = error instanceof Error
        ? error.message
        : String(error);
    }
  }

  sendSimple(name) {
    const spec = SIMPLE_COMMANDS[name];
    if (!spec) {
      this.actionStatus = `Unknown action: ${name}`;
      return;
    }
    try {
      const socket = this.ensurePublisher(spec.topic, spec.type);
      socket.send(JSON.stringify(spec.payload ?? {}));
      this.actionStatus = `${name.replace(/_/g, " ")} command sent.`;
    } catch (error) {
      this.actionStatus = error instanceof Error
        ? error.message
        : String(error);
    }
  }

  toggleLed(name, value) {
    const spec = LED_TOPICS[name];
    if (!spec) {
      return;
    }
    try {
      const socket = this.ensurePublisher(spec.topic, spec.type);
      socket.send(JSON.stringify({ data: Boolean(value) }));
      this.displayState = {
        ...this.displayState,
        ledOverrides: {
          ...this.displayState.ledOverrides,
          [name]: Boolean(value),
        },
      };
    } catch (error) {
      this.actionStatus = error instanceof Error
        ? error.message
        : String(error);
    }
  }

  sendAscii(event) {
    event.preventDefault();
    try {
      const payload = toAsciiPayload(this.displayState.ascii);
      const socket = this.ensurePublisher(
        "set_ascii",
        "std_msgs/msg/UInt8MultiArray",
      );
      socket.send(JSON.stringify(payload));
      this.actionStatus = "ASCII display updated.";
    } catch (error) {
      this.actionStatus = error instanceof Error
        ? error.message
        : String(error);
    }
  }

  sendPowerLed(event) {
    event.preventDefault();
    try {
      const payload = buildPowerLedPayload(
        this.displayState.powerColor,
        this.displayState.powerIntensity,
      );
      const socket = this.ensurePublisher(
        "power_led",
        "std_msgs/msg/UInt8MultiArray",
      );
      socket.send(JSON.stringify(payload));
      this.actionStatus = "Power LED updated.";
    } catch (error) {
      this.actionStatus = error instanceof Error
        ? error.message
        : String(error);
    }
  }

  async submitParameters(event) {
    event.preventDefault();
    const { node, ...parameters } = this.parameterForm;
    if (!node || !node.trim()) {
      this.parameterStatus =
        "Target node path is required (example: /create_driver).";
      return;
    }
    const requestParameters = buildParameterRequest(parameters);
    if (!requestParameters.length) {
      this.parameterStatus = "At least one parameter value must be provided.";
      return;
    }
    this.parameterBusy = true;
    this.parameterStatus = "Dispatching parameter update…";
    try {
      await callRosService({
        module: "foot",
        service: `${node.trim()}/set_parameters`,
        type: "rcl_interfaces/srv/SetParameters",
        args: { parameters: requestParameters },
      });
      this.parameterStatus = "Parameters updated successfully.";
    } catch (error) {
      this.parameterStatus = error instanceof Error
        ? error.message
        : String(error);
    } finally {
      this.parameterBusy = false;
    }
  }

  async submitSongDefinition(event) {
    event.preventDefault();
    const parsed = parseSongSheet(this.songForm.notes);
    if (!parsed.length) {
      this.songStatus =
        'Provide at least one "note,duration" pair to define a song.';
      return;
    }
    try {
      const payload = buildDefineSongPayload(this.songForm.id, parsed);
      const socket = this.ensurePublisher(
        "define_song",
        "create_msgs/msg/DefineSong",
      );
      socket.send(JSON.stringify(payload));
      this.songStatus = `Song ${payload.song} programmed with ${payload.length} notes.`;
    } catch (error) {
      this.songStatus = error instanceof Error ? error.message : String(error);
    }
  }

  playSong(event) {
    event.preventDefault();
    const id = Number(this.songForm.id);
    if (
      !Number.isInteger(id) ||
      id < SONG_LIMITS.minId ||
      id > SONG_LIMITS.maxId
    ) {
      this.songStatus =
        `Song ID must be between ${SONG_LIMITS.minId} and ${SONG_LIMITS.maxId}.`;
      return;
    }
    try {
      const socket = this.ensurePublisher(
        "play_song",
        "create_msgs/msg/PlaySong",
      );
      socket.send(JSON.stringify({ song: id }));
      this.songStatus = `Play request sent for song ${id}.`;
    } catch (error) {
      this.songStatus = error instanceof Error ? error.message : String(error);
    }
  }

  updateParameterField(field, value) {
    this.parameterForm = { ...this.parameterForm, [field]: value };
  }

  updateDisplayField(field, value) {
    this.displayState = { ...this.displayState, [field]: value };
  }

  updateSongField(field, value) {
    this.songForm = { ...this.songForm, [field]: value };
  }

  render() {
    const gaugeFill = Math.max(
      0,
      Math.min(1, (this.battery.percent ?? 0) / 100),
    );
    const chipVariant = this.battery.state === "critical"
      ? "critical"
      : this.battery.state === "charging"
      ? "success"
      : undefined;
    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card">
          <h3 class="surface-card__title">Battery</h3>
          <div
            class="battery-gauge"
            role="meter"
            aria-valuenow="${this.battery.percent}"
            aria-valuemin="0"
            aria-valuemax="100"
          >
            <div class="battery-gauge__fill" style="transform: scaleX(${gaugeFill})">
            </div>
          </div>
          <span class="surface-chip" data-variant="${chipVariant ?? ""}">${this
            .battery.status}</span>
          <div class="surface-grid surface-grid--dense surface-grid--narrow">
            <div class="surface-metric">
              <span class="surface-metric__label">Charge</span>
              <span class="surface-metric__value">${this.battery
                .chargeDisplay}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Capacity</span>
              <span class="surface-metric__value">${this.battery
                .capacityDisplay}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Charge ratio</span>
              <span class="surface-metric__value">${this.battery
                .ratioDisplay}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Voltage</span>
              <span class="surface-metric__value">${this.battery
                .voltageDisplay}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Current</span>
              <span class="surface-metric__value">${this.battery
                .currentDisplay}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Temperature</span>
              <span class="surface-metric__value">${this.battery
                .temperatureDisplay}</span>
            </div>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Drive</h3>
          <div class="drive-panel">
            <cockpit-joystick-control></cockpit-joystick-control>
            <p class="surface-status surface-mono">
              Drag to publish <code>/cmd_vel</code>.
            </p>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Proximity &amp; Safety</h3>
          <ul class="sensor-list">
            ${this.renderSensorRow("Bumpers", this.sensors.bumpers)} ${this
              .renderSensorRow("Cliff sensors", this.sensors.cliffs)} ${this
              .renderSensorRow("Wheel drop", this.sensors.wheels)}
          </ul>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Mobility actions</h3>
          <div class="surface-actions surface-actions--grid">
            <button class="surface-action" @click="${() =>
              this.sendSimple("dock")}">Dock</button>
            <button class="surface-action" @click="${() =>
              this.sendSimple("undock")}">Undock</button>
          </div>
          ${this.actionStatus
            ? html`
              <p class="surface-status" data-variant="${this.actionStatus
                  .includes("Unknown")
                ? "error"
                : "success"}">${this.actionStatus}</p>
            `
            : ""}
        </article>
      </div>

      <div class="surface-grid surface-grid--wide" style="margin-top: 1rem;">
        <article class="surface-card">
          <h3 class="surface-card__title">LED controls</h3>
          <div class="led-grid">
            ${Object.entries(LED_TOPICS).map(([name, spec]) => {
              const label = `${name} LED`.replace("_", " ");
              const checked = Boolean(this.displayState.ledOverrides[name]);
              return html`
                <label>
                  ${label}
                  <input
                    type="checkbox"
                    .checked="${checked}"
                    @change="${(event) =>
                      this.toggleLed(name, event.target.checked)}"
                    aria-label="${`Toggle ${spec.topic}`}"
                  />
                </label>
              `;
            })}
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Display controls</h3>
          <form @submit="${(event) => this.sendAscii(event)}">
            <label>
              ASCII text (4 chars)
              <input
                maxlength="4"
                .value="${this.displayState.ascii}"
                @input="${(event) =>
                  this.updateDisplayField(
                    "ascii",
                    event.target.value.toUpperCase(),
                  )}"
              />
            </label>
            <div class="surface-actions">
              <button class="surface-action" type="submit">Update ASCII</button>
            </div>
          </form>
          <form @submit="${(event) => this.sendPowerLed(event)}">
            <label>
              LED colour (0→green · 255→red)
              <input
                type="range"
                min="0"
                max="255"
                .value="${String(this.displayState.powerColor)}"
                @input="${(event) =>
                  this.updateDisplayField(
                    "powerColor",
                    Number(event.target.value),
                  )}"
              />
            </label>
            <label>
              LED intensity
              <input
                type="range"
                min="0"
                max="255"
                .value="${String(this.displayState.powerIntensity)}"
                @input="${(event) =>
                  this.updateDisplayField(
                    "powerIntensity",
                    Number(event.target.value),
                  )}"
              />
            </label>
            <div class="surface-actions">
              <button class="surface-action" type="submit">Update power LED</button>
            </div>
          </form>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Songs</h3>
          <form @submit="${(event) => this.submitSongDefinition(event)}">
            <div class="surface-grid surface-grid--dense">
              <label>
                Song ID
                <input
                  type="number"
                  min="0"
                  max="15"
                  .value="${this.songForm.id}"
                  @input="${(event) =>
                    this.updateSongField("id", event.target.value)}"
                  required
                />
              </label>
              <label>
                Notes (note,duration)
                <textarea
                  .value="${this.songForm.notes}"
                  @input="${(event) =>
                    this.updateSongField("notes", event.target.value)}"
                  placeholder="60,0.5\\n64,0.25"
                ></textarea>
              </label>
            </div>
            <div class="surface-actions surface-actions--grid">
              <button class="surface-action" type="submit">Define song</button>
              <button class="surface-action" type="button" @click="${(event) =>
                this.playSong(event)}">
                Play song
              </button>
            </div>
          </form>
          ${this.songStatus
            ? html`
              <p class="surface-status">${this.songStatus}</p>
            `
            : ""}
        </article>
      </div>

      <div class="surface-grid surface-grid--wide" style="margin-top: 1rem;">
        <article class="surface-card">
          <h3 class="surface-card__title">Telemetry</h3>
          <div class="telemetry-grid">
            <div class="surface-metric">
              <span class="surface-metric__label">Mode</span>
              <span class="surface-metric__value">${this.telemetry.mode}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">IR omni</span>
              <span class="surface-metric__value">${this.telemetry
                .irOmni}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Joint state</span>
              <span class="surface-metric__value">${this.telemetry.joint}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Odometry</span>
              <span class="surface-metric__value">${this.telemetry.odom}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">TF updates</span>
              <span class="surface-metric__value"
              >${this.telemetry.tf.count} frames · ${this.telemetry.tf
                .last}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">TF static</span>
              <span class="surface-metric__value"
              >${this.telemetry.tfStatic.count} frames · ${this.telemetry
                .tfStatic.last}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Main brush</span>
              <span class="surface-metric__value">${this.telemetry
                .mainBrush}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Side brush</span>
              <span class="surface-metric__value">${this.telemetry
                .sideBrush}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Vacuum motor</span>
              <span class="surface-metric__value">${this.telemetry
                .vacuum}</span>
            </div>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Diagnostics</h3>
          <pre
            class="surface-panel surface-mono"
            style="max-height: 220px; overflow-y: auto;"
          >${this.telemetry.diagnostics}</pre>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Event log</h3>
          <div class="surface-grid surface-grid--dense surface-grid--narrow">
            <section>
              <h4 class="surface-subtitle">Buttons</h4>
              <ul class="log-list">
                ${this.buttonLog.length
                  ? this.buttonLog.map(
                    (entry) =>
                      html`
                        <li class="log-entry">
                          <div class="log-entry__meta">
                            <span>${entry.label}</span>
                            <span>${entry.time}</span>
                          </div>
                          <code class="surface-mono">/${entry.topic}</code>
                        </li>
                      `,
                  )
                  : html`
                    <li class="surface-muted">No button presses observed.</li>
                  `}
              </ul>
            </section>
            <section>
              <h4 class="surface-subtitle">Parameter events</h4>
              <ul class="log-list">
                ${this.parameterLog.length
                  ? this.parameterLog.map(
                    (entry) =>
                      html`
                        <li class="log-entry">
                          <div class="log-entry__meta">
                            <span>${entry.time}</span>
                          </div>
                          <div>${entry.detail}</div>
                        </li>
                      `,
                  )
                  : html`
                    <li class="surface-muted">No parameter changes received.</li>
                  `}
              </ul>
            </section>
          </div>
        </article>
      </div>
    `;
  }

  renderSensorRow(label, alert) {
    return html`
      <li class="sensor-item" data-state="${alert ? "alert" : "ok"}">
        <span>${label}</span>
        <span class="sensor-item__dot" aria-hidden="true"></span>
      </li>
    `;
  }
}

customElements.define("foot-dashboard", FootDashboard);
