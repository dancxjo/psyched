import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';

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

    static styles = [
        surfaceStyles,
        css`
      .hypo-status {
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
        this.backend = 'initializing';
        this.temperatureC = null;
        this.temperatureF = null;
        this.humidityPercent = null;
        this.lastUpdate = 'Never';
        this.sockets = [];
    }

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
        const socket = createTopicSocket({
            module: 'hypothalamus',
            ...options,
            ...(action
                ? {
                      action,
                      arguments: Object.keys(actionArguments).length ? actionArguments : undefined,
                  }
                : {}),
        });
        this.sockets.push(socket);
        return socket;
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
    }

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
    }

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
    }

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
        const statusVariant = this.status === 'Live' ? 'success' : this.status === 'Error' ? 'error' : undefined;
        return html`
      <div class="surface-grid surface-grid--medium">
        <p class="surface-status hypo-status" data-variant="${statusVariant ?? ''}">Status: ${this.status}</p>

        <article class="surface-card">
          <h3 class="surface-card__title">Temperature</h3>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Celsius</span>
            <span class="surface-metric__value">${this.format(this.temperatureC)}</span>
          </div>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Fahrenheit</span>
            <span class="surface-metric__value">${this.format(this.temperatureF)}</span>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Humidity</h3>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Relative</span>
            <span class="surface-metric__value">${this.format(this.humidityPercent)}%</span>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Diagnostics</h3>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Backend</span>
            <span class="surface-metric__value">${this.backend}</span>
          </div>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Last update</span>
            <span class="surface-metric__value">${this.lastUpdate}</span>
          </div>
        </article>
      </div>
    `;
    }
}

customElements.define('hypothalamus-dashboard', HypothalamusDashboard);
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
