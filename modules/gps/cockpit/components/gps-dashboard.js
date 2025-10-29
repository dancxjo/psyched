import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';
import {
  buildGpsResetPayload,
  describeNavSatFix,
  normalizeResetMode,
} from './gps-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

/**
 * Dashboard for supervising GPS telemetry and maintenance actions.
 *
 * Events:
 * - ``gps-reset-request`` → `{ detail: GpsResetPayload }`
 */
class GpsDashboard extends LitElement {
  static properties = {
    fixStatus: { state: true },
    fixTone: { state: true },
    latitude: { state: true },
    longitude: { state: true },
    altitude: { state: true },
    lastFix: { state: true },
    resetMode: { state: true },
    resetNote: { state: true },
    resetFeedback: { state: true },
    eventLog: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      form {
        display: grid;
        gap: 0.75rem;
      }

      .form-row {
        display: grid;
        gap: 0.5rem;
      }

      .form-row--split {
        grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
        gap: 0.75rem;
      }

      label {
        display: flex;
        flex-direction: column;
        gap: 0.35rem;
        font-size: 0.75rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--metric-label-color);
      }

      input,
      select,
      textarea {
        font: inherit;
        padding: 0.55rem 0.75rem;
        border-radius: 0.5rem;
        border: 1px solid var(--control-surface-border);
        background: rgba(0, 0, 0, 0.3);
        color: var(--lcars-text);
        font-family: var(--metric-value-font);
      }

      textarea {
        min-height: 80px;
        resize: vertical;
      }

      .event-log {
        list-style: none;
        padding: 0;
        margin: 0;
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
        max-height: 260px;
        overflow-y: auto;
      }

      .event-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.35rem;
      }

      .event-entry__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }
    `,
  ];

  constructor() {
    super();
    this.fixStatus = 'Connecting…';
    this.fixTone = 'warning';
    this.latitude = '—';
    this.longitude = '—';
    this.altitude = '—';
    this.lastFix = '—';
    this.resetMode = 'hot';
    this.resetNote = '';
    this.resetFeedback = '';
    this.eventLog = [];
    this.sockets = [];
    this.lastTelemetrySummary = null;
  }

  connectedCallback() {
    super.connectedCallback();
    if (!this.sockets.length) {
      this.connectTelemetry();
    }
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    for (const socket of this.sockets) {
      try {
        socket.close();
      } catch (_error) {
        // ignore close failures
      }
    }
    this.sockets.length = 0;
  }

  connectTelemetry() {
    const socket = createTopicSocket({
      module: 'gps',
      action: 'navsat_fix_stream',
      type: 'sensor_msgs/msg/NavSatFix',
      role: 'subscribe',
    });
    socket.addEventListener('open', () => {
      this.recordEvent('Connected to GPS telemetry stream.', 'status');
      this.fixStatus = 'Awaiting satellite lock';
      this.fixTone = 'warning';
    });
    socket.addEventListener('message', (event) => {
      try {
        const payload = JSON.parse(event.data);
        if (payload?.event === 'message' && payload.data) {
          this.applyFixTelemetry(describeNavSatFix(payload.data));
        }
      } catch (error) {
        this.recordEvent(`Telemetry parse error: ${error.message}`, 'error');
        this.fixTone = 'error';
      }
    });
    socket.addEventListener('close', () => {
      this.recordEvent('GPS telemetry stream closed.', 'status');
      this.fixStatus = 'Disconnected';
      this.fixTone = 'error';
    });
    socket.addEventListener('error', () => {
      this.recordEvent('GPS telemetry stream error.', 'error');
      this.fixTone = 'error';
    });
    this.sockets.push(socket);
  }

  applyFixTelemetry(telemetry) {
    if (!telemetry) {
      return;
    }
    this.fixStatus = telemetry.statusText;
    this.fixTone = telemetry.tone;
    this.latitude = telemetry.latitude ?? '—';
    this.longitude = telemetry.longitude ?? '—';
    this.altitude = telemetry.altitude ?? '—';
    this.lastFix = telemetry.hasFix && telemetry.timestamp
      ? telemetry.timestamp.toLocaleTimeString()
      : '—';

    if (telemetry.eventSummary && telemetry.eventSummary !== this.lastTelemetrySummary) {
      this.recordEvent(telemetry.eventSummary, 'telemetry');
      this.lastTelemetrySummary = telemetry.eventSummary;
    }
    if (!telemetry.eventSummary) {
      this.lastTelemetrySummary = null;
    }
  }

  render() {
    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card">
          <h3 class="surface-card__title">Fix status</h3>
          <p class="surface-status" data-variant="${this.fixTone}">${this.fixStatus}</p>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Latitude</span>
            <span class="surface-metric__value">${this.latitude}</span>
          </div>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Longitude</span>
            <span class="surface-metric__value">${this.longitude}</span>
          </div>
          <div class="surface-metric surface-metric--inline">
            <span class="surface-metric__label">Altitude</span>
            <span class="surface-metric__value">${this.altitude}</span>
          </div>
          <p class="surface-card__subtitle">Last fix: ${this.lastFix}</p>
          <div class="surface-actions">
            <button type="button" class="surface-button" @click=${this.simulateFix}>Mark 3D fix</button>
            <button type="button" class="surface-button surface-button--ghost" @click=${this.simulateDrop}>
              Simulate loss
            </button>
          </div>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Reset receiver</h3>
          ${this.resetFeedback
            ? html`<p class="surface-status" data-variant="error">${this.resetFeedback}</p>`
            : ''}
          <form @submit=${this.handleResetSubmit}>
            <div class="form-row form-row--split">
              <label>
                Mode
                <select
                  name="mode"
                  .value=${this.resetMode}
                  @change=${(event) => (this.resetMode = normalizeResetMode(event.target.value))}
                >
                  <option value="hot">Hot start</option>
                  <option value="warm">Warm start</option>
                  <option value="cold">Cold start</option>
                </select>
              </label>
              <label>
                Operator note
                <textarea
                  name="note"
                  maxlength="240"
                  placeholder="Describe why a reset is necessary"
                  .value=${this.resetNote}
                  @input=${(event) => (this.resetNote = event.target.value)}
                ></textarea>
              </label>
            </div>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Queue reset</button>
              <button type="button" class="surface-button surface-button--ghost" @click=${this.clearResetForm}>
                Clear form
              </button>
            </div>
          </form>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Event log</h3>
          ${this.eventLog.length
            ? html`<ol class="event-log">
                ${this.eventLog.map(
                  (entry) => html`<li class="event-entry">
                    <div class="event-entry__meta">
                      <span>${entry.timestamp}</span>
                      <span>${entry.variant}</span>
                    </div>
                    <p class="event-entry__message">${entry.message}</p>
                  </li>`
                )}
              </ol>`
            : html`<p class="surface-empty">No GPS events recorded yet.</p>`}
        </article>
      </div>
    `;
  }

  handleResetSubmit(event) {
    event.preventDefault();
    const payload = buildGpsResetPayload({ mode: this.resetMode, note: this.resetNote });
    if (!payload.ok) {
      this.resetFeedback = payload.error;
      return;
    }
    this.resetFeedback = '';
    this.dispatchEvent(
      new CustomEvent('gps-reset-request', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.recordEvent(`Reset requested (${payload.value.mode} start).`, 'action');
    this.clearResetForm();
  }

  clearResetForm() {
    this.resetMode = 'hot';
    this.resetNote = '';
  }

  simulateFix() {
    this.fixStatus = '3D fix';
    this.fixTone = 'success';
    this.latitude = '49.2827°N';
    this.longitude = '123.1207°W';
    this.altitude = '70.0 m';
    this.lastFix = new Date().toLocaleTimeString();
    this.recordEvent('Simulated fix update received.', 'telemetry');
    this.lastTelemetrySummary = 'Simulated fix update received.';
  }

  simulateDrop() {
    this.fixStatus = 'Awaiting satellite lock';
    this.fixTone = 'warning';
    this.latitude = '—';
    this.longitude = '—';
    this.altitude = '—';
    this.lastFix = '—';
    this.recordEvent('Fix dropped; awaiting satellite lock.', 'telemetry');
    this.lastTelemetrySummary = 'Fix dropped; awaiting satellite lock.';
  }

  recordEvent(message, variant) {
    const entry = {
      id: makeId('gps'),
      timestamp: new Date().toLocaleTimeString(),
      message,
      variant,
    };
    this.eventLog = [entry, ...this.eventLog].slice(0, 40);
  }
}

customElements.define('gps-dashboard', GpsDashboard);
