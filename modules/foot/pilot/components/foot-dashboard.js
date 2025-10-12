import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';
import '/components/joystick-control.js';

const BATTERY_TOPICS = [
  { topic: 'battery/charge', type: 'std_msgs/msg/Float32', key: 'charge' },
  { topic: 'battery/capacity', type: 'std_msgs/msg/Float32', key: 'capacity' },
  { topic: 'battery/voltage', type: 'std_msgs/msg/Float32', key: 'voltage' },
  { topic: 'battery/current', type: 'std_msgs/msg/Float32', key: 'current' },
  { topic: 'battery/temperature', type: 'std_msgs/msg/Int16', key: 'temperature' },
  { topic: 'battery/charging_state', type: 'create_msgs/msg/ChargingState', key: 'charging' },
];

const SENSOR_TOPICS = [
  { topic: 'bumper', type: 'create_msgs/msg/Bumper', handler: 'updateBumper' },
  { topic: 'cliff', type: 'create_msgs/msg/Cliff', handler: 'updateCliff' },
  { topic: 'wheeldrop', type: 'std_msgs/msg/Empty', handler: 'updateWheelDrop' },
];

const SIMPLE_COMMANDS = {
  dock: { topic: 'dock', type: 'std_msgs/msg/Empty', payload: {} },
  undock: { topic: 'undock', type: 'std_msgs/msg/Empty', payload: {} },
  clean_button: { topic: 'clean_button', type: 'std_msgs/msg/Empty', payload: {} },
  spot_button: { topic: 'spot_button', type: 'std_msgs/msg/Empty', payload: {} },
  dock_button: { topic: 'dock_button', type: 'std_msgs/msg/Empty', payload: {} },
  power_led: { topic: 'power_led', type: 'std_msgs/msg/UInt8MultiArray', payload: { data: [0, 128] } },
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
  if (!message || typeof message !== 'object') {
    return { label: 'Unknown', state: 'idle' };
  }
  if ('state' in message) {
    const stateValue = String(message.state).toLowerCase();
    if (stateValue.includes('charge')) {
      return { label: message.state, state: 'charging' };
    }
    if (stateValue.includes('dock')) {
      return { label: message.state, state: 'charging' };
    }
    return { label: message.state, state: 'idle' };
  }
  if (message.is_charging === true) {
    return { label: 'Charging', state: 'charging' };
  }
  return { label: 'Discharging', state: 'idle' };
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
    actionStatus: { state: true },
    lastCommand: { state: true },
  };

  static styles = css`
    :host {
      display: block;
    }
    .foot-grid {
      display: grid;
      gap: 1rem;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
    }
    .foot-card {
      background: var(--control-surface-bg);
      border: 1px solid var(--control-surface-border);
      border-radius: var(--control-surface-radius);
      padding: var(--control-surface-padding);
      box-shadow: var(--control-surface-shadow);
      display: flex;
      flex-direction: column;
      gap: 1rem;
    }
    .foot-card h3 {
      margin: 0;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      font-size: 0.95rem;
      color: var(--metric-title-color);
    }
    .battery-metrics {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 0.75rem;
    }
    .metric {
      display: flex;
      flex-direction: column;
      gap: 0.25rem;
    }
    .metric .label {
      font-size: 0.75rem;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: var(--metric-label-color);
    }
    .metric .value {
      font-size: 1.05rem;
      font-weight: 600;
      color: var(--lcars-text);
      font-family: var(--metric-value-font);
    }
    .battery-gauge {
      position: relative;
      height: 16px;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.07);
      overflow: hidden;
    }
    .battery-gauge-fill {
      position: absolute;
      inset: 0;
      background: linear-gradient(90deg, var(--lcars-accent), var(--lcars-success));
      transform-origin: left center;
      transition: transform 180ms ease;
    }
    .status-chip {
      display: inline-flex;
      align-items: center;
      gap: 0.5rem;
      padding: 0.35rem 0.75rem;
      border-radius: 999px;
      font-size: 0.8rem;
      letter-spacing: 0.05em;
      text-transform: uppercase;
      background: rgba(88, 178, 220, 0.2);
      color: var(--lcars-text);
      align-self: flex-start;
    }
    .status-chip[data-state='charging'] {
      background: rgba(92, 209, 132, 0.25);
    }
    .status-chip[data-state='critical'] {
      background: rgba(255, 111, 97, 0.25);
    }
    .drive-panel {
      display: flex;
      flex-direction: column;
      gap: 1rem;
      align-items: center;
    }
    .hint {
      color: var(--lcars-muted);
      font-size: 0.85rem;
      text-align: center;
    }
    .drive-panel pre {
      background: rgba(0, 0, 0, 0.3);
      border-radius: 0.5rem;
      padding: 0.5rem;
      font-size: 0.75rem;
      max-height: 100px;
      overflow: auto;
      font-family: var(--metric-value-font);
      width: 100%;
      box-sizing: border-box;
    }
    .sensor-badges {
      display: grid;
      gap: 0.5rem;
    }
    .sensor-badge {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0.5rem 0.75rem;
      border-radius: 0.5rem;
      background: rgba(255, 255, 255, 0.04);
      font-size: 0.85rem;
    }
    .sensor-badge[data-state='alert'] {
      background: rgba(255, 111, 97, 0.2);
      color: var(--lcars-error);
    }
    .sensor-badge .status-dot {
      width: 10px;
      height: 10px;
      border-radius: 999px;
      background: var(--lcars-accent-secondary);
    }
    .sensor-badge[data-state='alert'] .status-dot {
      background: var(--lcars-error);
      box-shadow: 0 0 12px var(--lcars-error);
    }
    .action-grid {
      display: grid;
      gap: 0.5rem;
      grid-template-columns: repeat(auto-fit, minmax(100px, 1fr));
    }
    .action-grid button {
      background: rgba(88, 178, 220, 0.2);
      border: 1px solid var(--control-surface-border);
      border-radius: 0.5rem;
      padding: 0.65rem 0.75rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      font-size: 0.7rem;
      font-weight: 600;
      color: var(--lcars-text);
      cursor: pointer;
      transition: background 120ms ease, transform 120ms ease;
    }
    .action-grid button:hover {
      background: rgba(88, 178, 220, 0.35);
      transform: translateY(-2px);
    }
  `;

  constructor() {
    super();
    this.battery = {
      percent: 0,
      chargeDisplay: '—',
      voltageDisplay: '—',
      currentDisplay: '—',
      temperatureDisplay: '—',
      status: 'Awaiting telemetry…',
      state: 'idle',
    };
    this.sensors = {
      bumpers: false,
      cliffs: false,
      wheels: false,
    };
    this.actionStatus = '';
    this.lastCommand = '{ }';
    this.sockets = [];
    this.publishers = new Map();
    this.cmdVelPublisher = null;
  }

  connectedCallback() {
    super.connectedCallback();
    this.subscribeBattery();
    this.subscribeSensors();
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
      const socket = createTopicSocket({ topic: entry.topic, type: entry.type, role: 'subscribe' });
      socket.addEventListener('message', (event) => {
        try {
          const payload = JSON.parse(event.data);
          if (payload.event !== 'message') {
            return;
          }
          state[entry.key] = payload.data;
          this.updateBattery(state);
        } catch (error) {
          console.warn('Failed to parse battery payload', error);
        }
      });
      this.sockets.push(socket);
    }
  }

  updateBattery(state) {
    const charge = toNumber(state.charge?.data ?? state.charge);
    const capacity = toNumber(state.capacity?.data ?? state.capacity);
    const voltage = toNumber(state.voltage?.data ?? state.voltage);
    const current = toNumber(state.current?.data ?? state.current);
    const temperature = toNumber(state.temperature?.data ?? state.temperature);
    const percent = percentFromCharge(charge, capacity);
    const charging = chargingStatus(state.charging);

    this.battery = {
      percent: percent ?? 0,
      chargeDisplay: charge != null ? `${charge.toFixed(2)} Ah` : '—',
      voltageDisplay: voltage != null ? `${voltage.toFixed(2)} V` : '—',
      currentDisplay: current != null ? `${current.toFixed(2)} A` : '—',
      temperatureDisplay: temperature != null ? `${temperature.toFixed(1)} °C` : '—',
      status: percent != null ? `${percent.toFixed(0)}% · ${charging.label}` : charging.label,
      state: percent != null && percent < 20 ? 'critical' : charging.state,
    };
  }

  subscribeSensors() {
    for (const entry of SENSOR_TOPICS) {
      const socket = createTopicSocket({ topic: entry.topic, type: entry.type, role: 'subscribe' });
      socket.addEventListener('message', (event) => {
        try {
          const payload = JSON.parse(event.data);
          if (payload.event !== 'message') return;
          const handler = this[entry.handler];
          if (typeof handler === 'function') {
            handler.call(this, payload.data);
          }
        } catch (error) {
          console.warn('Failed to parse sensor payload', error);
        }
      });
      this.sockets.push(socket);
    }
  }

  updateBumper(message) {
    const candidates = ['is_left_pressed', 'is_right_pressed', 'is_center_pressed'];
    this.sensors = { ...this.sensors, bumpers: candidates.some((key) => Boolean(message?.[key])) };
  }

  updateCliff(message) {
    const candidates = ['is_left_detected', 'is_right_detected', 'is_front_left', 'is_front_right'];
    this.sensors = { ...this.sensors, cliffs: candidates.some((key) => Boolean(message?.[key])) };
  }

  updateWheelDrop(_message) {
    this.sensors = { ...this.sensors, wheels: true };
    setTimeout(() => {
      this.sensors = { ...this.sensors, wheels: false };
    }, 2000);
  }

  ensurePublisher(topic, type) {
    const key = `${topic}:${type}`;
    if (this.publishers.has(key)) {
      return this.publishers.get(key);
    }
    const socket = createTopicSocket({ topic, type, role: 'publish' });
    this.publishers.set(key, socket);
    this.sockets.push(socket);
    return socket;
  }

  firstUpdated() {
    const joystick = this.shadowRoot.querySelector('pilot-joystick-control');
    if (joystick) {
      this.cmdVelPublisher = this.ensurePublisher('cmd_vel', 'geometry_msgs/msg/Twist');
      joystick.record = {
        send: (message) => this.sendVelocity(message),
      };
    }
  }

  sendVelocity(message) {
    try {
      if (!this.cmdVelPublisher) {
        this.cmdVelPublisher = this.ensurePublisher('cmd_vel', 'geometry_msgs/msg/Twist');
      }
      this.cmdVelPublisher.send(JSON.stringify(message));
      this.lastCommand = JSON.stringify(message, null, 2);
    } catch (error) {
      this.actionStatus = error instanceof Error ? error.message : String(error);
    }
  }

  sendSimple(name, override) {
    const spec = SIMPLE_COMMANDS[name];
    if (!spec) {
      this.actionStatus = `Unknown action: ${name}`;
      return;
    }
    try {
      const socket = this.ensurePublisher(spec.topic, spec.type);
      const payload = override ?? spec.payload ?? {};
      socket.send(JSON.stringify(payload));
      this.actionStatus = `${name.replace(/_/g, ' ')} command sent.`;
    } catch (error) {
      this.actionStatus = error instanceof Error ? error.message : String(error);
    }
  }

  render() {
    return html`
      <div class="foot-grid">
        <article class="foot-card">
          <h3>Battery</h3>
          <div class="battery-gauge">
            <div class="battery-gauge-fill" style="transform: scaleX(${(this.battery.percent ?? 0) / 100})"></div>
          </div>
          <div class="status-chip" data-state="${this.battery.state}">
            ${this.battery.status}
          </div>
          <div class="battery-metrics">
            <div class="metric">
              <span class="label">Charge</span>
              <span class="value">${this.battery.chargeDisplay}</span>
            </div>
            <div class="metric">
              <span class="label">Voltage</span>
              <span class="value">${this.battery.voltageDisplay}</span>
            </div>
            <div class="metric">
              <span class="label">Current</span>
              <span class="value">${this.battery.currentDisplay}</span>
            </div>
            <div class="metric">
              <span class="label">Temperature</span>
              <span class="value">${this.battery.temperatureDisplay}</span>
            </div>
          </div>
        </article>

        <article class="foot-card">
          <h3>Drive</h3>
          <div class="drive-panel">
            <pilot-joystick-control></pilot-joystick-control>
            <p class="hint">
              Drag to send <code>/cmd_vel</code>
            </p>
            <pre>${this.lastCommand}</pre>
          </div>
        </article>

        <article class="foot-card">
          <h3>Proximity &amp; Safety</h3>
          <div class="sensor-badges">
            <div class="sensor-badge" data-state="${this.sensors.bumpers ? 'alert' : 'ok'}">
              <span>Bumpers</span>
              <span class="status-dot"></span>
            </div>
            <div class="sensor-badge" data-state="${this.sensors.cliffs ? 'alert' : 'ok'}">
              <span>Cliff Sensors</span>
              <span class="status-dot"></span>
            </div>
            <div class="sensor-badge" data-state="${this.sensors.wheels ? 'alert' : 'ok'}">
              <span>Wheel Drop</span>
              <span class="status-dot"></span>
            </div>
          </div>
        </article>

        <article class="foot-card">
          <h3>Actions</h3>
          <div class="action-grid">
            <button @click=${() => this.sendSimple('dock')}>Dock</button>
            <button @click=${() => this.sendSimple('undock')}>Undock</button>
            <button @click=${() => this.sendSimple('clean_button')}>Clean</button>
            <button @click=${() => this.sendSimple('spot_button')}>Spot</button>
            <button @click=${() => this.sendSimple('dock_button')}>Dock Btn</button>
            <button @click=${() => this.sendSimple('power_led', { data: [0, 255] })}>LED Max</button>
          </div>
          ${this.actionStatus ? html`<p class="hint">${this.actionStatus}</p>` : ''}
        </article>
      </div>
    `;
  }
}

customElements.define('foot-dashboard', FootDashboard);
