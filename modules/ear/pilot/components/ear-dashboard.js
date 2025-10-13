import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { surfaceStyles } from '/components/pilot-style.js';
import {
  buildEarConfigPayload,
  normalizeBackend,
} from './ear-dashboard.helpers.js';

function timestamp() {
  const now = new Date();
  return now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
}

/**
 * Ear module dashboard that exposes backend controls and transcript telemetry.
 *
 * Example usage:
 * ```html
 * <ear-dashboard></ear-dashboard>
 * ```
 *
 * Events dispatched for backend integration:
 * - ``ear-config-request`` with `{ detail: EarConfigPayload }` when the
 *   configuration form is submitted successfully.
 * - ``ear-transcript-submit`` with `{ detail: { text: string } }` whenever the
 *   operator injects a manual transcript.
 */
class EarDashboard extends LitElement {
  static properties = {
    backend: { state: true },
    serviceUri: { state: true },
    language: { state: true },
    beamSize: { state: true },
    sampleRate: { state: true },
    channels: { state: true },
    configFeedback: { state: true },
    configTone: { state: true },
    transcriptDraft: { state: true },
    transcriptFeedback: { state: true },
    vadActive: { state: true },
    transcripts: { state: true },
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
        min-height: 90px;
        resize: vertical;
      }

      .transcript-log {
        list-style: none;
        padding: 0;
        margin: 0;
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
        max-height: 280px;
        overflow-y: auto;
      }

      .transcript-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.35rem;
      }

      .transcript-entry__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .surface-status {
        margin-bottom: 0.5rem;
      }
    `,
  ];

  constructor() {
    super();
    this.backend = 'console';
    this.serviceUri = 'ws://127.0.0.1:8089/ws';
    this.language = '';
    this.beamSize = 5;
    this.sampleRate = 16000;
    this.channels = 1;
    this.configFeedback = 'Adjust backend settings and press apply to queue changes.';
    this.configTone = 'info';
    this.transcriptDraft = '';
    this.transcriptFeedback = '';
    this.vadActive = false;
    this.transcripts = [];
  }

  render() {
    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card">
          <h3 class="surface-card__title">Backend configuration</h3>
          <p class="surface-status" data-variant="${this.configTone}">${this.configFeedback}</p>
          <form @submit=${this.handleConfigSubmit}>
            <div class="form-row form-row--split">
              <label>
                Backend
                <select name="backend" .value=${this.backend} @change=${this.handleBackendChange}>
                  <option value="console">Console</option>
                  <option value="faster_whisper">Faster Whisper</option>
                  <option value="service">Streaming service</option>
                </select>
              </label>
              <label>
                Sample rate (Hz)
                <input
                  type="number"
                  name="sampleRate"
                  min="8000"
                  max="192000"
                  step="1000"
                  .value=${String(this.sampleRate)}
                  @input=${(event) => (this.sampleRate = Number(event.target.value))}
                />
              </label>
              <label>
                Channels
                <input
                  type="number"
                  name="channels"
                  min="1"
                  max="8"
                  .value=${String(this.channels)}
                  @input=${(event) => (this.channels = Number(event.target.value))}
                />
              </label>
            </div>
            <div class="form-row form-row--split">
              <label>
                Beam size
                <input
                  type="number"
                  name="beamSize"
                  min="1"
                  max="20"
                  .value=${String(this.beamSize)}
                  @input=${(event) => (this.beamSize = Number(event.target.value))}
                />
              </label>
              <label>
                Language hint
                <input
                  type="text"
                  name="language"
                  placeholder="auto"
                  .value=${this.language}
                  @input=${(event) => (this.language = event.target.value)}
                />
              </label>
              <label ?hidden=${this.backend !== 'service'}>
                Service URI
                <input
                  type="url"
                  name="serviceUri"
                  placeholder="ws://host:port/ws"
                  .value=${this.serviceUri}
                  @input=${(event) => (this.serviceUri = event.target.value)}
                />
              </label>
            </div>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Apply configuration</button>
            </div>
          </form>
        </article>

        <article class="surface-card">
          <header class="surface-card__title">
            <span>Voice activity</span>
            <span class="surface-pill" data-variant=${this.vadActive ? 'success' : 'muted'}>
              ${this.vadActive ? 'Detected' : 'Idle'}
            </span>
          </header>
          <p class="surface-card__subtitle">
            Toggle the indicator to mark observed speech bursts while testing hardware levels.
          </p>
          <div class="surface-actions">
            <button type="button" class="surface-button" @click=${this.toggleVad}>
              ${this.vadActive ? 'Mark silence' : 'Mark speech'}
            </button>
          </div>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Manual transcript injection</h3>
          ${this.transcriptFeedback
            ? html`<p class="surface-status" data-variant="error">${this.transcriptFeedback}</p>`
            : ''}
          <form @submit=${this.handleTranscriptSubmit}>
            <label>
              Transcript text
              <textarea
                name="transcript"
                .value=${this.transcriptDraft}
                @input=${(event) => (this.transcriptDraft = event.target.value)}
              ></textarea>
            </label>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Inject transcript</button>
              <button type="button" class="surface-button surface-button--ghost" @click=${this.clearTranscriptDraft}>
                Clear
              </button>
            </div>
          </form>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Transcript log</h3>
          ${this.transcripts.length === 0
            ? html`<p class="surface-empty">Awaiting audio events…</p>`
            : html`<ol class="transcript-log">
                ${this.transcripts.map(
                  (entry) => html`<li class="transcript-entry">
                    <div class="transcript-entry__meta">
                      <span>${entry.timestamp}</span>
                      <span>${entry.origin}</span>
                    </div>
                    <p class="transcript-entry__text">${entry.text}</p>
                  </li>`
                )}
              </ol>`}
        </article>
      </div>
    `;
  }

  handleBackendChange(event) {
    this.backend = normalizeBackend(event.target.value);
    this.requestUpdate();
  }

  handleConfigSubmit(event) {
    event.preventDefault();
    const payload = buildEarConfigPayload({
      backend: this.backend,
      serviceUri: this.serviceUri,
      language: this.language,
      beamSize: this.beamSize,
      sampleRate: this.sampleRate,
      channels: this.channels,
    });
    if (!payload.ok) {
      this.configFeedback = payload.error;
      this.configTone = 'error';
      return;
    }
    this.dispatchEvent(
      new CustomEvent('ear-config-request', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.configFeedback = 'Configuration submitted for backend reconciliation.';
    this.configTone = 'success';
  }

  handleTranscriptSubmit(event) {
    event.preventDefault();
    const text = this.transcriptDraft.trim();
    if (!text) {
      this.transcriptFeedback = 'Transcript text is required.';
      return;
    }
    this.dispatchEvent(
      new CustomEvent('ear-transcript-submit', {
        detail: { text },
        bubbles: true,
        composed: true,
      }),
    );
    this.transcriptFeedback = '';
    this.transcripts = [
      {
        id: crypto.randomUUID ? crypto.randomUUID() : `ear-${Date.now()}`,
        text,
        timestamp: timestamp(),
        origin: 'pilot override',
      },
      ...this.transcripts,
    ].slice(0, 50);
    this.transcriptDraft = '';
  }

  clearTranscriptDraft() {
    this.transcriptDraft = '';
    this.transcriptFeedback = '';
  }

  toggleVad() {
    this.vadActive = !this.vadActive;
  }
}

customElements.define('ear-dashboard', EarDashboard);
