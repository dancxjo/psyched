import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';
import { surfaceStyles } from '/components/pilot-style.js';
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
        background: linear-gradient(90deg, var(--lcars-accent), var(--lcars-success));
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
      .sensor-item[data-state='alert'] {
        background: rgba(255, 111, 97, 0.2);
        color: var(--lcars-error);
      }
      .sensor-item__dot {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        background: var(--lcars-accent-secondary);
      }
      .sensor-item[data-state='alert'] .sensor-item__dot {
        background: var(--lcars-error);
        box-shadow: 0 0 12px var(--lcars-error);
      }
      .surface-actions--grid {
        grid-template-columns: repeat(auto-fit, minmax(110px, 1fr));
      }
    `,
    ];

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
        const gaugeFill = Math.max(0, Math.min(1, (this.battery.percent ?? 0) / 100));
        const chipVariant = this.battery.state === 'critical' ? 'critical' : this.battery.state === 'charging' ? 'success' : undefined;
        return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card">
          <h3 class="surface-card__title">Battery</h3>
          <div class="battery-gauge" role="meter" aria-valuenow="${this.battery.percent}" aria-valuemin="0" aria-valuemax="100">
            <div class="battery-gauge__fill" style="transform: scaleX(${gaugeFill})"></div>
          </div>
          <span class="surface-chip" data-variant="${chipVariant ?? ''}">${this.battery.status}</span>
          <div class="surface-grid surface-grid--dense surface-grid--narrow">
            <div class="surface-metric">
              <span class="surface-metric__label">Charge</span>
              <span class="surface-metric__value">${this.battery.chargeDisplay}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Voltage</span>
              <span class="surface-metric__value">${this.battery.voltageDisplay}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Current</span>
              <span class="surface-metric__value">${this.battery.currentDisplay}</span>
            </div>
            <div class="surface-metric">
              <span class="surface-metric__label">Temperature</span>
              <span class="surface-metric__value">${this.battery.temperatureDisplay}</span>
            </div>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Drive</h3>
          <div class="drive-panel">
            <pilot-joystick-control></pilot-joystick-control>
            <p class="surface-status surface-mono">Drag to publish <code>/cmd_vel</code>.</p>
            <pre class="surface-panel surface-mono drive-panel__log">${this.lastCommand}</pre>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Proximity &amp; Safety</h3>
          <ul class="sensor-list">
            ${this.renderSensorRow('Bumpers', this.sensors.bumpers)}
            ${this.renderSensorRow('Cliff Sensors', this.sensors.cliffs)}
            ${this.renderSensorRow('Wheel Drop', this.sensors.wheels)}
          </ul>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Actions</h3>
          <div class="surface-actions surface-actions--grid">
            <button class="surface-action" @click=${() => this.sendSimple('dock')}>Dock</button>
            <button class="surface-action" @click=${() => this.sendSimple('undock')}>Undock</button>
            <button class="surface-action" @click=${() => this.sendSimple('clean_button')}>Clean</button>
            <button class="surface-action" @click=${() => this.sendSimple('spot_button')}>Spot</button>
            <button class="surface-action" @click=${() => this.sendSimple('dock_button')}>Dock Btn</button>
            <button class="surface-action" @click=${() => this.sendSimple('power_led', { data: [0, 255] })}>LED Max</button>
          </div>
          ${this.actionStatus
            ? html`<p class="surface-status" data-variant="${this.actionStatus.includes('Unknown') ? 'error' : 'success'}">${this.actionStatus}</p>`
            : ''}
        </article>
      </div>
    `;
    }

    renderSensorRow(label, alert) {
        return html`
      <li class="sensor-item" data-state="${alert ? 'alert' : 'ok'}">
        <span>${label}</span>
        <span class="sensor-item__dot" aria-hidden="true"></span>
      </li>
    `;
    }
}

customElements.define('foot-dashboard', FootDashboard);
