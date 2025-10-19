import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { surfaceStyles } from '/components/cockpit-style.js';
import { createTopicSocket } from '/js/cockpit.js';
import { buildPilotIntentPayload } from './pilot-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

/**
 * Dashboard for orchestrating Pilot intent overrides.
 *
 * Custom events emitted for backend wiring:
 * - ``pilot-intent-submit`` → `{ detail: PilotIntentPayload }`
 */
class PilotDashboard extends LitElement {
  static properties = {
    valence: { state: true },
    arousal: { state: true },
    stance: { state: true },
    context: { state: true },
    statusMessage: { state: true },
    statusTone: { state: true },
    intentLog: { state: true },
    // Debugging / telemetry
    connected: { state: true },
    moduleStatus: { state: true },
    lastHeartbeat: { state: true },
    inboundMessages: { state: true },
    moduleConfig: { state: true },
    moduleLogs: { state: true },
    moduleErrors: { state: true },
    scriptRuns: { state: true },
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

      label {
        display: flex;
        flex-direction: column;
        gap: 0.35rem;
        font-size: 0.75rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--metric-label-color);
      }

      input[type='range'] {
        width: 100%;
      }

      textarea {
        font: inherit;
        padding: 0.6rem 0.75rem;
        border-radius: 0.5rem;
        border: 1px solid var(--control-surface-border);
        background: rgba(0, 0, 0, 0.3);
        color: var(--lcars-text);
        min-height: 100px;
        resize: vertical;
      }

      .intent-log {
        list-style: none;
        padding: 0;
        margin: 0;
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
        max-height: 260px;
        overflow-y: auto;
      }

      .intent-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.35rem;
      }

      .intent-entry__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }
    `,
  ];

  constructor() {
    super();
    this.valence = 0;
    this.arousal = 0.2;
    this.stance = 0.5;
    this.context = '';
    this.statusMessage = 'Compose a feeling intent to steer downstream planners.';
    this.statusTone = 'info';
    this.intentLog = [];
    // debug defaults
    this.connected = false;
    this.moduleStatus = 'unknown';
    this.lastHeartbeat = null;
    this.inboundMessages = [];
    this.moduleConfig = null;
    this.moduleLogs = [];
    this.moduleErrors = [];
    this.scriptRuns = [];
  }

  connectedCallback() {
    super.connectedCallback();
    // Listen for module-sent debug/status events (backend should bubble these)
    window.addEventListener('pilot-module-debug', this._onModuleDebug);
    window.addEventListener('pilot-module-log', this._onModuleLog);
    window.addEventListener('pilot-module-status', this._onModuleStatus);
    window.addEventListener('pilot-module-message', this._onModuleMessage);
    // Also try subscribing to the pilot debug stream via the cockpit API
    try {
      this._debugSocket = createTopicSocket({ module: 'pilot', action: 'debug_stream' });
      this._debugSocket.addEventListener('message', (ev) => {
        try {
          const payload = JSON.parse(ev.data);
          // payload expected: { event: 'message', data: { data: '<json-string>' } }
          const envelope = JSON.parse(payload.data?.data || payload.data || '{}');
          // Merge known keys into state
          if (envelope.config) this.moduleConfig = envelope.config;
          if (envelope.status) this.moduleStatus = envelope.status;
          if (envelope.heartbeat) this.lastHeartbeat = envelope.heartbeat;
          if (Array.isArray(envelope.logs)) this.moduleLogs = [...envelope.logs, ...this.moduleLogs].slice(0, 500);
          if (Array.isArray(envelope.errors)) this.moduleErrors = [...envelope.errors, ...this.moduleErrors].slice(0, 200);
          if (Array.isArray(envelope.scripts)) this.scriptRuns = envelope.scripts;
          // Record the raw message as inbound
          this.inboundMessages = [
            { timestamp: new Date().toLocaleTimeString(), payload: envelope, source: 'ros:/pilot/debug' },
            ...this.inboundMessages,
          ].slice(0, 200);
          this.connected = true;
        } catch (err) {
          // ignore JSON parse errors
        }
      });
      this._debugSocket.addEventListener('close', () => (this.connected = false));
    } catch (_error) {
      // ignore socket initialisation errors; cockpit may not be available in static preview
    }
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    window.removeEventListener('pilot-module-debug', this._onModuleDebug);
    window.removeEventListener('pilot-module-log', this._onModuleLog);
    window.removeEventListener('pilot-module-status', this._onModuleStatus);
    window.removeEventListener('pilot-module-message', this._onModuleMessage);
  }

  // Event handlers bound as fields so they can be removed correctly
  _onModuleDebug = (ev) => this.handleModuleDebug(ev);
  _onModuleLog = (ev) => this.handleModuleLog(ev);
  _onModuleStatus = (ev) => this.handleModuleStatus(ev);
  _onModuleMessage = (ev) => this.handleModuleMessage(ev);

  render() {
    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Feeling intent composer</h3>
          <p class="surface-status" data-variant="${this.statusTone}">${this.statusMessage}</p>
          <form @submit=${this.handleIntentSubmit}>
            <div class="form-row">
              <label>
                Valence (${this.valence.toFixed(2)})
                <input
                  type="range"
                  min="-1"
                  max="1"
                  step="0.05"
                  .value=${String(this.valence)}
                  @input=${(event) => (this.valence = Number(event.target.value))}
                />
              </label>
              <label>
                Arousal (${this.arousal.toFixed(2)})
                <input
                  type="range"
                  min="0"
                  max="1"
                  step="0.05"
                  .value=${String(this.arousal)}
                  @input=${(event) => (this.arousal = Number(event.target.value))}
                />
              </label>
              <label>
                Stance (${this.stance.toFixed(2)})
                <input
                  type="range"
                  min="0"
                  max="1"
                  step="0.05"
                  .value=${String(this.stance)}
                  @input=${(event) => (this.stance = Number(event.target.value))}
                />
              </label>
            </div>
            <label>
              Narrative context
              <textarea
                name="context"
                placeholder="Summarise the desired behaviour or motivation..."
                .value=${this.context}
                @input=${(event) => (this.context = event.target.value)}
              ></textarea>
            </label>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Broadcast intent</button>
              <button type="button" class="surface-button surface-button--ghost" @click=${this.resetForm}>
                Reset composer
              </button>
            </div>
          </form>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Intent log</h3>
          ${this.intentLog.length
        ? html`<ol class="intent-log">
                ${this.intentLog.map(
          (entry) => html`<li class="intent-entry">
                    <div class="intent-entry__meta">
                      <span>${entry.timestamp}</span>
                      <span>V:${entry.valence.toFixed(2)}</span>
                      <span>A:${entry.arousal.toFixed(2)}</span>
                      <span>S:${entry.stance.toFixed(2)}</span>
                    </div>
                    <p class="intent-entry__context">${entry.context}</p>
                    ${entry.command_script
                      ? html`<pre style="white-space:pre-wrap;margin:0;font:inherit;background:transparent;border:0;padding:0;">${entry.command_script}</pre>`
                      : ''}
                  </li>`
        )}
              </ol>`
        : html`<p class="surface-empty">No intents broadcast yet.</p>`}
          <div class="surface-actions">
            <button type="button" class="surface-button surface-button--ghost" @click=${this.simulatePulse}>
              Simulate inbound intent
            </button>
          </div>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Pilot module debug</h3>
          <p class="surface-status" data-variant="${this.connected ? 'success' : 'warning'}">
            Module: ${this.moduleStatus} ${this.lastHeartbeat ? html`— heartbeat ${this.lastHeartbeat}` : ''}
          </p>

          <div style="display:grid;gap:0.5rem;">
            <div style="display:flex;gap:0.5rem;flex-wrap:wrap;">
              <button class="surface-button" @click=${this.requestDebugDump}>Request debug snapshot</button>
              <button class="surface-button surface-button--ghost" @click=${this.requestConfig}>Refresh config</button>
              <button class="surface-button surface-button--ghost" @click=${this.clearModuleLogs}>Clear logs</button>
              <button class="surface-button surface-button--ghost" @click=${this.copyConfigToClipboard}>Copy config</button>
            </div>

            <details open>
              <summary>Recent inbound messages (${this.inboundMessages.length})</summary>
              ${this.inboundMessages.length
        ? html`<ol class="intent-log">
                    ${this.inboundMessages.map(
          (m) => html`<li class="intent-entry">
                        <div class="intent-entry__meta">
                          <span>${m.timestamp}</span>
                          <span>src:${m.source ?? 'unknown'}</span>
                        </div>
                        <pre style="white-space:pre-wrap;margin:0;font:inherit;background:transparent;border:0;padding:0;">${this._pretty(m.payload)}</pre>
                      </li>`
        )}
                  </ol>`
        : html`<p class="surface-empty">No inbound messages seen yet.</p>`}
            </details>

            <details open>
              <summary>Recent command scripts (${this.scriptRuns.length})</summary>
              ${this.scriptRuns.length
        ? html`<ol class="intent-log">
                    ${this.scriptRuns.map(
          (run) => html`<li class="intent-entry">
                        <div class="intent-entry__meta">
                          <span>${run.requested_at ?? 'unknown'}</span>
                          <span>${run.source ?? 'unknown'}</span>
                          <span>${run.status ?? 'unknown'}</span>
                        </div>
                        <pre style="white-space:pre-wrap;margin:0;font:inherit;background:transparent;border:0;padding:0;">${run.script ?? ''}</pre>
                        ${Array.isArray(run.actions) && run.actions.length
                          ? html`<details open>
                              <summary>Actions (${run.actions.length})</summary>
                              <ol class="intent-log">
                                ${run.actions.map(
                                  (a) => html`<li class="intent-entry">
                                      <div class="intent-entry__meta">
                                        <span>${a.timestamp ?? ''}</span>
                                        <span>${a.status ?? ''}</span>
                                      </div>
                                      <div>${a.action ?? ''}</div>
                                      ${a.response
                                        ? html`<pre style="white-space:pre-wrap;margin:0;font:inherit;background:transparent;border:0;padding:0;">${a.response}</pre>`
                                        : ''}
                                    </li>`
                                )}
                              </ol>
                            </details>`
                          : ''}
                      </li>`
        )}
                  </ol>`
        : html`<p class="surface-empty">No scripts have been queued.</p>`}
            </details>

            <details>
              <summary>Module configuration</summary>
              ${this.moduleConfig ? html`<pre style="white-space:pre-wrap;margin:0;font:inherit;">${this._pretty(this.moduleConfig)}</pre>` : html`<p class="surface-empty">No config received.</p>`}
            </details>

            <details>
              <summary>Logs (${this.moduleLogs.length})</summary>
              ${this.moduleLogs.length
        ? html`<ol class="intent-log">
                    ${this.moduleLogs.map((l) => html`<li class="intent-entry"><div class="intent-entry__meta"><span>${l.timestamp}</span><span>${l.level}</span></div><pre style="white-space:pre-wrap;margin:0;font:inherit;">${l.message}</pre></li>`)}
                  </ol>`
        : html`<p class="surface-empty">No logs yet.</p>`}
            </details>

            <details>
              <summary>Errors (${this.moduleErrors.length})</summary>
              ${this.moduleErrors.length
        ? html`<ol class="intent-log">
                    ${this.moduleErrors.map((e) => html`<li class="intent-entry"><div class="intent-entry__meta"><span>${e.timestamp}</span><span>${e.level ?? 'error'}</span></div><pre style="white-space:pre-wrap;margin:0;font:inherit;">${e.message}${e.stack ? '\n' + e.stack : ''}</pre></li>`)}
                  </ol>`
        : html`<p class="surface-empty">No errors reported.</p>`}
            </details>
          </div>
        </article>
      </div>
    `;
  }

  handleIntentSubmit(event) {
    event.preventDefault();
    const payload = buildPilotIntentPayload({
      valence: this.valence,
      arousal: this.arousal,
      stance: this.stance,
      context: this.context,
    });
    if (!payload.ok) {
      this.statusMessage = payload.error;
      this.statusTone = 'error';
      return;
    }
    this.dispatchEvent(
      new CustomEvent('pilot-intent-submit', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = 'Intent queued for broadcasting downstream.';
    this.statusTone = 'success';
    this.recordIntent(payload.value, 'cockpit override');
  }

  resetForm() {
    this.valence = 0;
    this.arousal = 0.2;
    this.stance = 0.5;
    this.context = '';
    this.statusMessage = 'Composer reset to neutral baseline.';
    this.statusTone = 'info';
  }

  simulatePulse() {
    const entry = {
      valence: Math.random() * 2 - 1,
      arousal: Math.random(),
      stance: Math.random(),
      context: 'Simulated mission feedback pulse.',
    };
    this.recordIntent(entry, 'autonomy');
  }

  recordIntent(intent, source) {
    const logEntry = {
      id: makeId('pilot'),
      timestamp: new Date().toLocaleTimeString(),
      valence: intent.valence,
      arousal: intent.arousal,
      stance: intent.stance,
      context: `[${source}] ${intent.context}`,
      command_script: intent.command_script || '',
    };
    this.intentLog = [logEntry, ...this.intentLog].slice(0, 40);
  }

  /* ---------- Debug helpers & event wiring ---------- */

  // Pretty-print objects for UI
  _pretty(obj) {
    try {
      return typeof obj === 'string' ? obj : JSON.stringify(obj, null, 2);
    } catch (err) {
      return String(obj);
    }
  }

  // Dispatch an event to request a debug snapshot from the backend
  requestDebugDump() {
    this.dispatchEvent(new CustomEvent('pilot-debug-request', { bubbles: true, composed: true }));
    this.statusMessage = 'Requested debug snapshot from pilot module…';
    this.statusTone = 'info';
  }

  requestConfig() {
    this.dispatchEvent(new CustomEvent('pilot-config-request', { bubbles: true, composed: true }));
    this.statusMessage = 'Requested config from pilot module…';
    this.statusTone = 'info';
  }

  clearModuleLogs() {
    this.moduleLogs = [];
    this.moduleErrors = [];
    this.inboundMessages = [];
    this.dispatchEvent(new CustomEvent('pilot-clear-logs', { bubbles: true, composed: true }));
    this.statusMessage = 'Cleared cockpit-side logs.';
    this.statusTone = 'info';
  }

  copyConfigToClipboard = async () => {
    try {
      const text = this.moduleConfig ? JSON.stringify(this.moduleConfig, null, 2) : '';
      await navigator.clipboard.writeText(text);
      this.statusMessage = 'Module config copied to clipboard.';
      this.statusTone = 'success';
    } catch (err) {
      this.statusMessage = 'Failed to copy config.';
      this.statusTone = 'error';
    }
  };

  // Handlers: these expect the backend to post CustomEvents with `detail` containing payloads
  handleModuleDebug(ev) {
    const payload = ev?.detail ?? {};
    // payload may include { config, logs, errors, status }
    if (payload.config) this.moduleConfig = payload.config;
    if (Array.isArray(payload.logs)) this.moduleLogs = [...payload.logs, ...this.moduleLogs].slice(0, 200);
    if (Array.isArray(payload.errors)) this.moduleErrors = [...payload.errors, ...this.moduleErrors].slice(0, 200);
    if (Array.isArray(payload.scripts)) this.scriptRuns = payload.scripts;
    if (payload.status) this.moduleStatus = payload.status;
    if (payload.heartbeat) this.lastHeartbeat = payload.heartbeat;
    this.connected = true;
    this.statusMessage = 'Received debug snapshot from pilot module.';
    this.statusTone = 'success';
  }

  handleModuleLog(ev) {
    const payload = ev?.detail ?? {};
    const entry = { timestamp: new Date().toLocaleTimeString(), level: payload.level ?? 'info', message: payload.message ?? String(payload) };
    this.moduleLogs = [entry, ...this.moduleLogs].slice(0, 500);
    if ((payload.level || '').toLowerCase() === 'error') {
      this.moduleErrors = [{ ...entry, stack: payload.stack }, ...this.moduleErrors].slice(0, 200);
    }
  }

  handleModuleStatus(ev) {
    const payload = ev?.detail ?? {};
    this.moduleStatus = payload.status ?? this.moduleStatus;
    if (payload.heartbeat) this.lastHeartbeat = payload.heartbeat;
    this.connected = !!payload.connected || this.connected;
  }

  handleModuleMessage(ev) {
    const payload = ev?.detail ?? {};
    const msg = { timestamp: new Date().toLocaleTimeString(), payload: payload.payload ?? payload, source: payload.source ?? 'module' };
    this.inboundMessages = [msg, ...this.inboundMessages].slice(0, 200);
  }
}

customElements.define('pilot-dashboard', PilotDashboard);
