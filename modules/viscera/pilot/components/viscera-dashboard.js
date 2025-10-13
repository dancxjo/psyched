import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';
import { surfaceStyles } from '/components/pilot-style.js';

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
 * published by the pilot backend, but the `topic` attribute can override it
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
      .surface-status {
        grid-column: 1 / -1;
        text-align: center;
      }

      .metric-value {
        font-size: 2.5rem;
        font-weight: 600;
        margin: 0.5rem 0 0;
      }

      .metric-subtext {
        margin: 0.25rem 0 0;
        color: rgba(255, 255, 255, 0.75);
      }

      .metric-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(18rem, 1fr));
        gap: 1rem;
      }

      .uptime {
        font-family: 'IBM Plex Mono', 'SFMono-Regular', Consolas, 'Liberation Mono', Menlo, monospace;
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

  #renderStatusVariant() {
    if (this.status === 'Live') {
      return 'success';
    }
    if (this.status === 'Error') {
      return 'error';
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
      <section class="surface-grid surface-grid--medium">
        <p class="surface-status" data-variant="${this.#renderStatusVariant()}">
          Status: ${this.status} · Last update ${this.lastUpdate}
        </p>
        <div class="metric-grid">
          <article class="surface-card">
            <h2>CPU Load</h2>
            <p class="metric-value">${this.#formatPercent(this.metrics.cpuPercent)}</p>
            <p class="metric-subtext">Load averages: ${this.#formatLoad()}</p>
          </article>
          <article class="surface-card">
            <h2>Memory Usage</h2>
            <p class="metric-value">${this.#formatPercent(this.metrics.memPercent)}</p>
            <p class="metric-subtext">${this.#formatMemory()}</p>
          </article>
          <article class="surface-card">
            <h2>Disk Utilisation</h2>
            <p class="metric-value">${this.#formatPercent(this.metrics.diskPercent)}</p>
            <p class="metric-subtext">Root filesystem consumption</p>
          </article>
          <article class="surface-card">
            <h2>Temperature</h2>
            <p class="metric-value">${this.#formatTemperature(this.metrics.tempC)}</p>
            <p class="metric-subtext">Reported by sensors_temperatures</p>
          </article>
          <article class="surface-card">
            <h2>Uptime</h2>
            <p class="metric-value uptime">${this.#formatUptime(this.metrics.uptimeSec)}</p>
            <p class="metric-subtext">Hours · Minutes · Seconds</p>
          </article>
        </div>
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
  const hostGlobals = typeof window !== 'undefined' && window.Pilot ? window.Pilot.host || {} : {};
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
