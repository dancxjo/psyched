import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';

function asNumber(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

class HypothalamusDashboard extends LitElement {
  static properties = {
    status: { state: true },
    backend: { state: true },
    temperatureC: { state: true },
    temperatureF: { state: true },
    humidityPercent: { state: true },
    lastUpdate: { state: true },
  };

  static styles = css`
    :host {
      display: block;
    }
    .hypo-grid {
      display: grid;
      gap: 1rem;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    }
    .hypo-card {
      background: var(--control-surface-bg);
      border: 1px solid var(--control-surface-border);
      border-radius: var(--control-surface-radius);
      padding: var(--control-surface-padding);
      box-shadow: var(--control-surface-shadow);
      display: flex;
      flex-direction: column;
      gap: 0.75rem;
    }
    .hypo-card h3 {
      margin: 0;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      font-size: 0.9rem;
      color: var(--metric-title-color);
    }
    .metric-pair {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      gap: 0.75rem;
    }
    .metric-pair .label {
      font-size: 0.75rem;
      color: var(--metric-label-color);
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }
    .metric-pair .value {
      font-family: var(--metric-value-font);
      font-size: 0.95rem;
      font-weight: 600;
      color: var(--lcars-text);
    }
    .status-bar {
      background: rgba(0, 0, 0, 0.3);
      border-radius: 0.5rem;
      padding: 0.5rem 0.75rem;
      text-align: center;
      font-size: 0.8rem;
      color: var(--lcars-muted);
      grid-column: 1 / -1;
    }
    .status-bar.live {
      color: var(--lcars-success);
    }
    .status-bar.error {
      color: var(--lcars-error);
    }
  `;

  constructor() {
    super();
    this.status = 'Connecting…';
    this.backend = 'initializing';
    this.temperatureC = null;
    this.temperatureF = null;
    this.humidityPercent = null;
    this.lastUpdate = 'Never';
    this.sockets = [];
  }

  connectedCallback() {
    super.connectedCallback();
    this.connectTemperature();
    this.connectFahrenheit();
    this.connectHumidity();
    this.connectStatus();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    for (const socket of this.sockets) {
      try {
        socket.close();
      } catch (_error) {
        // ignore
      }
    }
    this.sockets.length = 0;
  }

  connectTemperature() {
    const socket = createTopicSocket({
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
    this.sockets.push(socket);
  }

  connectFahrenheit() {
    const socket = createTopicSocket({
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
    this.sockets.push(socket);
  }

  connectHumidity() {
    const socket = createTopicSocket({
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
    this.sockets.push(socket);
  }

  connectStatus() {
    const socket = createTopicSocket({
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
    this.sockets.push(socket);
  }

  touch() {
    this.lastUpdate = new Date().toLocaleTimeString();
  }

  format(value, fractionDigits = 1) {
    if (value === null || value === undefined) {
      return '—';
    }
    return Number(value).toFixed(fractionDigits);
  }

  render() {
    const statusClass = this.status === 'Live' ? 'live' : this.status === 'Error' ? 'error' : '';
    return html`
      <div class="hypo-grid">
        <div class="status-bar ${statusClass}">Status: ${this.status}</div>
        
        <div class="hypo-card">
          <h3>Temperature</h3>
          <div class="metric-pair">
            <span class="label">Celsius</span>
            <span class="value">${this.format(this.temperatureC)}</span>
          </div>
          <div class="metric-pair">
            <span class="label">Fahrenheit</span>
            <span class="value">${this.format(this.temperatureF)}</span>
          </div>
        </div>

        <div class="hypo-card">
          <h3>Humidity</h3>
          <div class="metric-pair">
            <span class="label">Relative</span>
            <span class="value">${this.format(this.humidityPercent)}%</span>
          </div>
        </div>

        <div class="hypo-card">
          <h3>Diagnostics</h3>
          <div class="metric-pair">
            <span class="label">Backend</span>
            <span class="value">${this.backend}</span>
          </div>
          <div class="metric-pair">
            <span class="label">Last update</span>
            <span class="value">${this.lastUpdate}</span>
          </div>
        </div>
      </div>
    `;
  }
}

customElements.define('hypothalamus-dashboard', HypothalamusDashboard);
