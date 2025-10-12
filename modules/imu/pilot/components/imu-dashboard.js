import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';
import { surfaceStyles } from '/components/pilot-style.js';

class ImuDashboard extends LitElement {
    static properties = {
        status: { state: true },
        orientation: { state: true },
        angularVelocity: { state: true },
        linearAcceleration: { state: true },
        temperatureC: { state: true },
        temperatureF: { state: true },
    };

    static styles = [
        surfaceStyles,
        css`
      .imu-status {
        grid-column: 1 / -1;
        text-align: center;
        background: rgba(0, 0, 0, 0.3);
        border-radius: 0.5rem;
        padding: 0.5rem 0.75rem;
      }
    `,
    ];

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
        const statusVariant = this.status === 'Live' ? 'success' : this.status === 'Error' ? 'error' : undefined;
        return html`
      <div class="surface-grid surface-grid--medium">
        <p class="surface-status imu-status" data-variant="${statusVariant ?? ''}">Status: ${this.status}</p>

        <article class="surface-card">
          <h3 class="surface-card__title">Orientation (Quaternion)</h3>
          ${['x', 'y', 'z', 'w'].map((axis) =>
            html`<div class="surface-metric surface-metric--inline">
              <span class="surface-metric__label">${axis.toUpperCase()}</span>
              <span class="surface-metric__value">${this.format(this.orientation[axis], 3)}</span>
            </div>`)}
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Angular Velocity (rad/s)</h3>
          ${['x', 'y', 'z'].map((axis) =>
            html`<div class="surface-metric surface-metric--inline">
              <span class="surface-metric__label">${axis.toUpperCase()}</span>
              <span class="surface-metric__value">${this.format(this.angularVelocity[axis])}</span>
            </div>`)}
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Linear Acceleration (m/s²)</h3>
          ${['x', 'y', 'z'].map((axis) =>
            html`<div class="surface-metric surface-metric--inline">
              <span class="surface-metric__label">${axis.toUpperCase()}</span>
              <span class="surface-metric__value">${this.format(this.linearAcceleration[axis])}</span>
            </div>`)}
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Temperature</h3>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Celsius</span>
            <span class="surface-metric__value">${this.temperatureC !== null ? this.format(this.temperatureC, 1) : '—'}</span>
          </div>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Fahrenheit</span>
            <span class="surface-metric__value">${this.temperatureF !== null ? this.format(this.temperatureF, 1) : '—'}</span>
          </div>
        </article>
      </div>
    `;
    }
}

customElements.define('imu-dashboard', ImuDashboard);
