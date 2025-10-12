import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';

class ImuDashboard extends LitElement {
  static properties = {
    status: { state: true },
    orientation: { state: true },
    angularVelocity: { state: true },
    linearAcceleration: { state: true },
    temperatureC: { state: true },
    temperatureF: { state: true },
  };

  static styles = css`
    :host {
      display: block;
    }
    .imu-grid {
      display: grid;
      gap: 1rem;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    }
    .imu-card {
      background: var(--control-surface-bg);
      border: 1px solid var(--control-surface-border);
      border-radius: var(--control-surface-radius);
      padding: var(--control-surface-padding);
      box-shadow: var(--control-surface-shadow);
      display: flex;
      flex-direction: column;
      gap: 0.75rem;
    }
    .imu-card h3 {
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
    this.orientation = { x: 0, y: 0, z: 0, w: 0 };
    this.angularVelocity = { x: 0, y: 0, z: 0 };
    this.linearAcceleration = { x: 0, y: 0, z: 0 };
    this.temperatureC = null;
    this.temperatureF = null;
    this.sockets = [];
  }

  connectedCallback() {
    super.connectedCallback();
    this.connectImu();
    this.connectTemperature();
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

  connectImu() {
    const socket = createTopicSocket({
      topic: '/imu/data',
      type: 'sensor_msgs/msg/Imu',
      role: 'subscribe',
    });
    socket.addEventListener('message', (event) => {
      const payload = JSON.parse(event.data);
      if (payload.event === 'message' && payload.data) {
        const data = payload.data;
        this.orientation = data.orientation ?? this.orientation;
        this.angularVelocity = data.angular_velocity ?? this.angularVelocity;
        this.linearAcceleration = data.linear_acceleration ?? this.linearAcceleration;
        this.status = 'Live';
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

  connectTemperature() {
    const socket = createTopicSocket({
      topic: '/imu/temperature',
      type: 'std_msgs/msg/Float32',
      role: 'subscribe',
    });
    socket.addEventListener('message', (event) => {
      const payload = JSON.parse(event.data);
      if (payload.event === 'message' && payload.data && 'data' in payload.data) {
        const celsius = Number(payload.data.data);
        if (!Number.isNaN(celsius)) {
          this.temperatureC = celsius;
          this.temperatureF = celsius * (9 / 5) + 32;
        }
      }
    });
    this.sockets.push(socket);
  }

  format(value, fractionDigits = 2) {
    return Number(value).toFixed(fractionDigits);
  }

  render() {
    const statusClass = this.status === 'Live' ? 'live' : this.status === 'Error' ? 'error' : '';
    return html`
      <div class="imu-grid">
        <div class="status-bar ${statusClass}">Status: ${this.status}</div>
        
        <div class="imu-card">
          <h3>Orientation (Quaternion)</h3>
          <div class="metric-pair">
            <span class="label">X</span>
            <span class="value">${this.format(this.orientation.x, 3)}</span>
          </div>
          <div class="metric-pair">
            <span class="label">Y</span>
            <span class="value">${this.format(this.orientation.y, 3)}</span>
          </div>
          <div class="metric-pair">
            <span class="label">Z</span>
            <span class="value">${this.format(this.orientation.z, 3)}</span>
          </div>
          <div class="metric-pair">
            <span class="label">W</span>
            <span class="value">${this.format(this.orientation.w, 3)}</span>
          </div>
        </div>

        <div class="imu-card">
          <h3>Angular Velocity (rad/s)</h3>
          <div class="metric-pair">
            <span class="label">X</span>
            <span class="value">${this.format(this.angularVelocity.x)}</span>
          </div>
          <div class="metric-pair">
            <span class="label">Y</span>
            <span class="value">${this.format(this.angularVelocity.y)}</span>
          </div>
          <div class="metric-pair">
            <span class="label">Z</span>
            <span class="value">${this.format(this.angularVelocity.z)}</span>
          </div>
        </div>

        <div class="imu-card">
          <h3>Linear Acceleration (m/s²)</h3>
          <div class="metric-pair">
            <span class="label">X</span>
            <span class="value">${this.format(this.linearAcceleration.x)}</span>
          </div>
          <div class="metric-pair">
            <span class="label">Y</span>
            <span class="value">${this.format(this.linearAcceleration.y)}</span>
          </div>
          <div class="metric-pair">
            <span class="label">Z</span>
            <span class="value">${this.format(this.linearAcceleration.z)}</span>
          </div>
        </div>

        <div class="imu-card">
          <h3>Temperature</h3>
          <div class="metric-pair">
            <span class="label">Celsius</span>
            <span class="value">${this.temperatureC !== null ? this.format(this.temperatureC, 1) : '—'}</span>
          </div>
          <div class="metric-pair">
            <span class="label">Fahrenheit</span>
            <span class="value">${this.temperatureF !== null ? this.format(this.temperatureF, 1) : '—'}</span>
          </div>
        </div>
      </div>
    `;
  }
}

customElements.define('imu-dashboard', ImuDashboard);
