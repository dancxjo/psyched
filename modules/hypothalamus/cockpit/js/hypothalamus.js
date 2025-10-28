import { createTopicSocket } from '/js/cockpit.js';

function asNumber(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

export function dashboard() {
  return {
    status: 'Connecting…',
    backend: 'initializing',
    temperatureC: null,
    temperatureF: null,
    humidityPercent: null,
    lastUpdate: 'Never',
    createSocket(options) {
      // Cockpit routes sockets by module; omitting the module key causes runtime failures.
      const action = options?.action || resolveStreamAction(options?.topic);
      const actionArguments = {};
      if (typeof options?.topic === 'string' && options.topic.trim()) {
        actionArguments.topic = options.topic.trim();
      }
      if (Number.isFinite(options?.queueLength) && options.queueLength > 0) {
        actionArguments.queue_length = Math.floor(options.queueLength);
      }
      return createTopicSocket({
        module: 'hypothalamus',
        ...options,
        ...(action
          ? {
              action,
              arguments: Object.keys(actionArguments).length ? actionArguments : undefined,
            }
          : {}),
      });
    },
    init() {
      this.connectTemperature();
      this.connectFahrenheit();
      this.connectHumidity();
      this.connectStatus();
    },
    connectTemperature() {
      const socket = this.createSocket({
        topic: '/environment/temperature',
        type: 'sensor_msgs/msg/Temperature',
        role: 'subscribe',
      });
      socket.addEventListener('message', (event) => {
        const payload = JSON.parse(event.data);
        if (payload.event === 'message' && payload.data && 'temperature' in payload.data) {
          const celsius = asNumber(payload.data.temperature);
          if (celsius !== null) {
            this.temperatureC = celsius;
            this.status = 'Live';
            this.touch();
          }
        }
      });
      socket.addEventListener('close', () => {
        this.status = 'Disconnected';
      });
      socket.addEventListener('error', () => {
        this.status = 'Error';
      });
    },
    connectFahrenheit() {
      const socket = this.createSocket({
        topic: '/environment/temperature_fahrenheit',
        type: 'std_msgs/msg/Float32',
        role: 'subscribe',
      });
      socket.addEventListener('message', (event) => {
        const payload = JSON.parse(event.data);
        if (payload.event === 'message' && payload.data && 'data' in payload.data) {
          const fahrenheit = asNumber(payload.data.data);
          if (fahrenheit !== null) {
            this.temperatureF = fahrenheit;
            this.touch();
          }
        }
      });
    },
    connectHumidity() {
      const socket = this.createSocket({
        topic: '/environment/humidity_percent',
        type: 'std_msgs/msg/Float32',
        role: 'subscribe',
      });
      socket.addEventListener('message', (event) => {
        const payload = JSON.parse(event.data);
        if (payload.event === 'message' && payload.data && 'data' in payload.data) {
          const humidity = asNumber(payload.data.data);
          if (humidity !== null) {
            this.humidityPercent = humidity;
            this.touch();
          }
        }
      });
    },
    connectStatus() {
      const socket = this.createSocket({
        topic: '/environment/thermostat_status',
        type: 'std_msgs/msg/String',
        role: 'subscribe',
      });
      socket.addEventListener('message', (event) => {
        const payload = JSON.parse(event.data);
        if (payload.event === 'message' && payload.data && 'data' in payload.data) {
          this.backend = String(payload.data.data || 'unknown');
        }
      });
    },
    touch() {
      this.lastUpdate = new Date().toLocaleTimeString();
    },
    format(value, fractionDigits = 1) {
      if (value === null || value === undefined) {
        return '—';
      }
      return Number(value).toFixed(fractionDigits);
    },
  };
}

window.Hypothalamus = { dashboard };
function resolveStreamAction(topic) {
  if (typeof topic !== 'string') {
    return null;
  }
  const trimmed = topic.trim();
  switch (trimmed) {
    case '/environment/temperature':
      return 'temperature_stream';
    case '/environment/temperature_fahrenheit':
      return 'temperature_fahrenheit_stream';
    case '/environment/humidity_percent':
      return 'humidity_stream';
    case '/environment/thermostat_status':
      return 'thermostat_status_stream';
    default:
      return null;
  }
}
