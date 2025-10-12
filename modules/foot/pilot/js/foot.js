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

function formatValue(value, suffix = '') {
  if (value == null) return '—';
  return `${value}${suffix}`;
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

export function dashboard() {
  return {
    battery: {
      percent: null,
      chargeDisplay: '—',
      voltageDisplay: '—',
      currentDisplay: '—',
      temperatureDisplay: '—',
      status: 'Awaiting telemetry…',
      state: 'idle',
    },
    sensors: {
      bumpers: false,
      cliffs: false,
      wheels: false,
    },
    actionStatus: '',
    lastCommand: '{ }',
    sockets: [],
    publishers: new Map(),
    cmdVelPublisher: null,

    init() {
      this.subscribeBattery();
      this.subscribeSensors();
      this.setupJoystick();
      window.addEventListener('beforeunload', () => this.shutdown(), { once: true });
    },

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
    },

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
    },

    updateBattery(state) {
      const charge = toNumber(state.charge?.data ?? state.charge);
      const capacity = toNumber(state.capacity?.data ?? state.capacity);
      const voltage = toNumber(state.voltage?.data ?? state.voltage);
      const current = toNumber(state.current?.data ?? state.current);
      const temperature = toNumber(state.temperature?.data ?? state.temperature);
      const percent = percentFromCharge(charge, capacity);
      const charging = chargingStatus(state.charging);

      this.battery.percent = percent ?? 0;
      this.battery.chargeDisplay = charge != null ? `${charge.toFixed(2)} Ah` : '—';
      this.battery.voltageDisplay = voltage != null ? `${voltage.toFixed(2)} V` : '—';
      this.battery.currentDisplay = current != null ? `${current.toFixed(2)} A` : '—';
      this.battery.temperatureDisplay = temperature != null ? `${temperature.toFixed(1)} °C` : '—';
      this.battery.status = percent != null ? `${percent.toFixed(0)}% · ${charging.label}` : charging.label;
      this.battery.state = percent != null && percent < 20 ? 'critical' : charging.state;
    },

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
    },

    updateBumper(message) {
      const candidates = ['is_left_pressed', 'is_right_pressed', 'is_center_pressed'];
      this.sensors.bumpers = candidates.some((key) => Boolean(message?.[key]));
    },

    updateCliff(message) {
      const candidates = ['is_left_detected', 'is_right_detected', 'is_front_left', 'is_front_right'];
      this.sensors.cliffs = candidates.some((key) => Boolean(message?.[key]));
    },

    updateWheelDrop(_message) {
      this.sensors.wheels = true;
      setTimeout(() => {
        this.sensors.wheels = false;
      }, 2000);
    },

    ensurePublisher(topic, type) {
      const key = `${topic}:${type}`;
      if (this.publishers.has(key)) {
        return this.publishers.get(key);
      }
      const socket = createTopicSocket({ topic, type, role: 'publish' });
      this.publishers.set(key, socket);
      this.sockets.push(socket);
      return socket;
    },

    setupJoystick() {
      const joystick = this.$refs?.cmdVelJoystick;
      if (!joystick) {
        return;
      }
      this.cmdVelPublisher = this.ensurePublisher('cmd_vel', 'geometry_msgs/msg/Twist');
      joystick.record = {
        send: (message) => this.sendVelocity(message),
      };
    },

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
    },

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
    },
  };
}

window.Foot = { dashboard };
