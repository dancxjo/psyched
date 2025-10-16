import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';

const HOST_HEALTH_MSG_TYPE = 'psyched_msgs/msg/HostHealth';

/**
 * Host telemetry surface that subscribes to the viscera HostHealth topic.
 *
 * Usage example:
 * ```html
 * <viscera-dashboard></viscera-dashboard>
 * ```
 *
 * The widget resolves the topic path automatically from the host metadata
 * published by the cockpit backend, but the `topic` attribute can override it
 * when embedding the component on diagnostic pages:
 *
 * ```html
 * <viscera-dashboard topic="/hosts/health/testbed"></viscera-dashboard>
 * ```
 */
class VisceraDashboard extends LitElement {
  #socket;

  static properties = {
    status: { state: true },
    lastUpdate: { state: true },
    metrics: { state: true },
    topic: { type: String, reflect: true },
  };

  static styles = [
    surfaceStyles,
    css`
      .dashboard-status {
        grid-column: 1 / -1;
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.75rem;
        flex-wrap: wrap;
      }

      .dashboard-status__note {
        margin: 0;
      }

      .metric-card {
        gap: 0.5rem;
      }

      .metric-card .surface-metric__value {
        font-size: 1.35rem;
      }

      .metric-card .surface-note {
        margin: 0;
      }
    `,
  ];

  constructor() {
    super();
    this.status = 'Connecting…';
    this.lastUpdate = 'Never';
    this.metrics = this.#blankMetrics();
    this.topic = '';
  }

  connectedCallback() {
    super.connectedCallback();
    this.#connect();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this.#teardown();
  }

  updated(changed) {
    if (changed.has('topic')) {
      this.#connect();
    }
  }

  #blankMetrics() {
    return {
      cpuPercent: null,
      loadAvg1: null,
      loadAvg5: null,
      loadAvg15: null,
      memPercent: null,
      memUsedMb: null,
      memTotalMb: null,
      diskPercent: null,
      tempC: null,
      uptimeSec: null,
    };
  }

  #connect() {
    this.#teardown();
    const topic = this.topic && this.topic.trim() ? this.topic.trim() : resolveHostHealthTopic();
    try {
      const socket = createTopicSocket({
        topic,
        type: HOST_HEALTH_MSG_TYPE,
        role: 'subscribe',
      });
      socket.addEventListener('open', () => {
        this.status = 'Live';
      });
      socket.addEventListener('close', () => {
        this.status = 'Disconnected';
      });
      socket.addEventListener('error', () => {
        this.status = 'Error';
      });
      socket.addEventListener('message', (event) => this.#handleMessage(event));
      this.#socket = socket;
    } catch (error) {
      console.warn('Failed to open HostHealth socket', error);
      this.status = 'Error';
    }
  }

  #teardown() {
    if (this.#socket) {
      try {
        this.#socket.close();
      } catch (_error) {
        // ignored
      }
    }
    this.#socket = null;
  }

  #handleMessage(event) {
    try {
      const payload = JSON.parse(event.data);
      if (!payload || payload.event !== 'message' || !payload.data) {
        return;
      }
      const data = payload.data;
      this.metrics = {
        cpuPercent: toOptionalNumber(data.cpu_percent),
        loadAvg1: toOptionalNumber(data.load_avg_1),
        loadAvg5: toOptionalNumber(data.load_avg_5),
        loadAvg15: toOptionalNumber(data.load_avg_15),
        memPercent: toOptionalNumber(data.mem_used_percent),
        memUsedMb: toOptionalNumber(data.mem_used_mb),
        memTotalMb: toOptionalNumber(data.mem_total_mb),
        diskPercent: toOptionalNumber(data.disk_used_percent_root),
        tempC: toOptionalNumber(data.temp_c),
        uptimeSec: toOptionalNumber(data.uptime_sec),
      };
      this.status = 'Live';
      this.lastUpdate = new Date().toLocaleTimeString();
    } catch (error) {
      console.warn('Failed to parse HostHealth payload', error);
      this.status = 'Error';
    }
  }

  #renderStatusChipVariant() {
    if (this.status === 'Live') {
      return 'success';
    }
    if (this.status === 'Error') {
      return 'critical';
    }
    if (this.status === 'Disconnected') {
      return 'warning';
    }
    return '';
  }

  #formatPercent(value) {
    if (typeof value !== 'number' || Number.isNaN(value)) {
      return '—';
    }
    return `${value.toFixed(1)}%`;
  }

  #formatLoad() {
    const { loadAvg1, loadAvg5, loadAvg15 } = this.metrics;
    if ([loadAvg1, loadAvg5, loadAvg15].some((value) => value == null)) {
      return '—';
    }
    return `${loadAvg1.toFixed(2)} · ${loadAvg5.toFixed(2)} · ${loadAvg15.toFixed(2)}`;
  }

  #formatTemperature(value) {
    if (typeof value !== 'number' || Number.isNaN(value)) {
      return '—';
    }
    return `${value.toFixed(1)} °C`;
  }

  #formatMemory() {
    const { memUsedMb, memTotalMb } = this.metrics;
    if (memUsedMb == null || memTotalMb == null) {
      return '—';
    }
    return `${memUsedMb.toFixed(0)} / ${memTotalMb.toFixed(0)} MiB`;
  }

  #formatUptime(value) {
    if (typeof value !== 'number' || Number.isNaN(value)) {
      return '—';
    }
    const rounded = Math.max(0, Math.floor(value));
    const hours = Math.floor(rounded / 3600);
    const minutes = Math.floor((rounded % 3600) / 60);
    const seconds = rounded % 60;
    return `${pad(hours)}:${pad(minutes)}:${pad(seconds)}`;
  }

  render() {
    return html`
      <section class="surface-grid surface-grid--medium surface-grid--dense">
        <div class="dashboard-status">
          <span class="surface-chip" data-variant="${this.#renderStatusChipVariant()}">${this.status}</span>
          <p class="surface-note dashboard-status__note">Last update ${this.lastUpdate}</p>
        </div>
        <article class="surface-card surface-card--compact metric-card">
          <h3 class="surface-card__title">CPU load</h3>
          <div class="surface-metric">
            <span class="surface-metric__label">Utilisation</span>
            <span class="surface-metric__value surface-metric__value--large">
              ${this.#formatPercent(this.metrics.cpuPercent)}
            </span>
          </div>
          <p class="surface-note surface-mono">Load averages ${this.#formatLoad()}</p>
        </article>
        <article class="surface-card surface-card--compact metric-card">
          <h3 class="surface-card__title">Memory usage</h3>
          <div class="surface-metric">
            <span class="surface-metric__label">Utilisation</span>
            <span class="surface-metric__value surface-metric__value--large">
              ${this.#formatPercent(this.metrics.memPercent)}
            </span>
          </div>
          <p class="surface-note surface-mono">${this.#formatMemory()}</p>
        </article>
        <article class="surface-card surface-card--compact metric-card">
          <h3 class="surface-card__title">Disk utilisation</h3>
          <div class="surface-metric">
            <span class="surface-metric__label">Utilisation</span>
            <span class="surface-metric__value surface-metric__value--large">
              ${this.#formatPercent(this.metrics.diskPercent)}
            </span>
          </div>
          <p class="surface-note">Root filesystem consumption</p>
        </article>
        <article class="surface-card surface-card--compact metric-card">
          <h3 class="surface-card__title">Temperature</h3>
          <div class="surface-metric">
            <span class="surface-metric__label">Sensor reading</span>
            <span class="surface-metric__value surface-metric__value--large">
              ${this.#formatTemperature(this.metrics.tempC)}
            </span>
          </div>
          <p class="surface-note">Reported by sensors_temperatures</p>
        </article>
        <article class="surface-card surface-card--compact metric-card">
          <h3 class="surface-card__title">Uptime</h3>
          <div class="surface-metric">
            <span class="surface-metric__label">Elapsed</span>
            <span class="surface-metric__value surface-metric__value--large surface-mono">
              ${this.#formatUptime(this.metrics.uptimeSec)}
            </span>
          </div>
          <p class="surface-note">Hours · Minutes · Seconds</p>
        </article>
      </section>
    `;
  }
}

function pad(value) {
  return value.toString().padStart(2, '0');
}

function toOptionalNumber(value) {
  if (value === null || value === undefined) {
    return null;
  }
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function resolveHostHealthTopic() {
  const hostGlobals = typeof window !== 'undefined' && window.Cockpit ? window.Cockpit.host || {} : {};
  const raw = typeof hostGlobals.shortname === 'string' ? hostGlobals.shortname.trim() : '';
  const shortname = raw || deriveHostShortname(window.location?.hostname);
  return `/hosts/health/${shortname || 'host'}`;
}

function deriveHostShortname(hostname) {
  if (!hostname) {
    return '';
  }
  const cleaned = hostname.trim();
  if (!cleaned) {
    return '';
  }
  const short = cleaned.split('.')[0];
  return short || cleaned;
}

customElements.define('viscera-dashboard', VisceraDashboard);

export { resolveHostHealthTopic, deriveHostShortname };
