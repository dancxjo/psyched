import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { surfaceStyles } from '/components/pilot-style.js';
import {
  buildEyeSettingsPayload,
  normalizeDepthMode,
} from './eye-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

/**
 * Dashboard for the Eye module exposing Kinect stream controls.
 *
 * Custom events:
 * - ``eye-settings-request`` → `{ detail: EyeSettingsPayload }`
 * - ``eye-capture-request`` → `{ detail: { mode: 'colour' | 'depth' } }`
 */
class EyeDashboard extends LitElement {
  static properties = {
    width: { state: true },
    height: { state: true },
    frameRate: { state: true },
    depthMode: { state: true },
    alignDepth: { state: true },
    autoExposure: { state: true },
    exposure: { state: true },
    gain: { state: true },
    statusMessage: { state: true },
    statusTone: { state: true },
    captureFeedback: { state: true },
    captureHistory: { state: true },
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
      select {
        font: inherit;
        padding: 0.55rem 0.75rem;
        border-radius: 0.5rem;
        border: 1px solid var(--control-surface-border);
        background: rgba(0, 0, 0, 0.3);
        color: var(--lcars-text);
        font-family: var(--metric-value-font);
      }

      .preview {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.75rem;
        min-height: 240px;
        display: grid;
        place-items: center;
        color: var(--lcars-muted);
        text-align: center;
        padding: 1.5rem;
      }

      .capture-history {
        list-style: none;
        padding: 0;
        margin: 0;
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
        max-height: 240px;
        overflow-y: auto;
      }

      .capture-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.25rem;
      }

      .capture-entry__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }
    `,
  ];

  constructor() {
    super();
    this.width = 1280;
    this.height = 720;
    this.frameRate = 15;
    this.depthMode = 'disabled';
    this.alignDepth = false;
    this.autoExposure = true;
    this.exposure = 3000;
    this.gain = 32;
    this.statusMessage = 'Awaiting stream status updates…';
    this.statusTone = 'info';
    this.captureFeedback = '';
    this.captureHistory = [];
  }

  render() {
    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Stream preview</h3>
          <p class="surface-status" data-variant="${this.statusTone}">${this.statusMessage}</p>
          <div class="preview" role="img" aria-label="Live video preview placeholder">
            <div>
              <p>Preview stream placeholder</p>
              <p>${this.width}×${this.height} @ ${this.frameRate} FPS</p>
            </div>
          </div>
          <div class="surface-actions">
            <button type="button" class="surface-button" @click=${this.markLive}>Mark feed live</button>
            <button type="button" class="surface-button surface-button--ghost" @click=${this.markIdle}>
              Mark idle
            </button>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Capture settings</h3>
          <form @submit=${this.handleSettingsSubmit}>
            <div class="form-row form-row--split">
              <label>
                Width
                <input
                  type="number"
                  name="width"
                  min="320"
                  max="1920"
                  step="16"
                  .value=${String(this.width)}
                  @input=${(event) => (this.width = Number(event.target.value))}
                />
              </label>
              <label>
                Height
                <input
                  type="number"
                  name="height"
                  min="240"
                  max="1080"
                  step="16"
                  .value=${String(this.height)}
                  @input=${(event) => (this.height = Number(event.target.value))}
                />
              </label>
              <label>
                Frame rate
                <input
                  type="number"
                  name="frameRate"
                  min="5"
                  max="60"
                  .value=${String(this.frameRate)}
                  @input=${(event) => (this.frameRate = Number(event.target.value))}
                />
              </label>
            </div>
            <div class="form-row form-row--split">
              <label>
                Depth mode
                <select
                  name="depthMode"
                  .value=${this.depthMode}
                  @change=${(event) => (this.depthMode = normalizeDepthMode(event.target.value))}
                >
                  <option value="disabled">Colour only</option>
                  <option value="depth">Depth stream</option>
                  <option value="aligned_depth">Aligned depth</option>
                </select>
              </label>
              <label>
                Align depth
                <select
                  name="alignDepth"
                  .value=${this.alignDepth ? 'true' : 'false'}
                  @change=${(event) => (this.alignDepth = event.target.value === 'true')}
                >
                  <option value="false">Disabled</option>
                  <option value="true">Enabled</option>
                </select>
              </label>
              <label>
                Auto exposure
                <select
                  name="autoExposure"
                  .value=${this.autoExposure ? 'true' : 'false'}
                  @change=${(event) => (this.autoExposure = event.target.value === 'true')}
                >
                  <option value="true">Enabled</option>
                  <option value="false">Manual</option>
                </select>
              </label>
            </div>
            <div class="form-row form-row--split" ?hidden=${this.autoExposure}>
              <label>
                Exposure (µs)
                <input
                  type="number"
                  name="exposure"
                  min="10"
                  max="33000"
                  .value=${String(this.exposure)}
                  @input=${(event) => (this.exposure = Number(event.target.value))}
                />
              </label>
              <label>
                Gain
                <input
                  type="number"
                  name="gain"
                  min="1"
                  max="128"
                  .value=${String(this.gain)}
                  @input=${(event) => (this.gain = Number(event.target.value))}
                />
              </label>
            </div>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Apply settings</button>
            </div>
          </form>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Frame capture</h3>
          ${this.captureFeedback
            ? html`<p class="surface-status" data-variant="error">${this.captureFeedback}</p>`
            : ''}
          <div class="surface-actions">
            <button type="button" class="surface-button" @click=${() => this.handleCapture('colour')}>
              Capture colour frame
            </button>
            <button type="button" class="surface-button surface-button--ghost" @click=${() => this.handleCapture('depth')}>
              Capture depth frame
            </button>
          </div>
          ${this.captureHistory.length
            ? html`<ol class="capture-history">
                ${this.captureHistory.map(
                  (entry) => html`<li class="capture-entry">
                    <div class="capture-entry__meta">
                      <span>${entry.timestamp}</span>
                      <span>${entry.mode}</span>
                    </div>
                    <p class="capture-entry__note">${entry.note}</p>
                  </li>`
                )}
              </ol>`
            : html`<p class="surface-empty">No captures recorded yet.</p>`}
        </article>
      </div>
    `;
  }

  handleSettingsSubmit(event) {
    event.preventDefault();
    const payload = buildEyeSettingsPayload({
      width: this.width,
      height: this.height,
      frameRate: this.frameRate,
      depthMode: this.depthMode,
      alignDepth: this.alignDepth,
      autoExposure: this.autoExposure,
      exposure: this.exposure,
      gain: this.gain,
    });
    if (!payload.ok) {
      this.statusMessage = payload.error;
      this.statusTone = 'error';
      return;
    }
    this.dispatchEvent(
      new CustomEvent('eye-settings-request', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = 'Settings submitted for device reconciliation.';
    this.statusTone = 'success';
  }

  handleCapture(mode) {
    this.captureFeedback = '';
    this.dispatchEvent(
      new CustomEvent('eye-capture-request', {
        detail: { mode },
        bubbles: true,
        composed: true,
      }),
    );
    this.captureHistory = [
      {
        id: makeId('capture'),
        mode,
        timestamp: new Date().toLocaleTimeString(),
        note: mode === 'depth' ? 'Depth frame queued for export.' : 'Colour frame queued for export.',
      },
      ...this.captureHistory,
    ].slice(0, 20);
  }

  markLive() {
    this.statusMessage = 'Live stream detected.';
    this.statusTone = 'success';
  }

  markIdle() {
    this.statusMessage = 'Awaiting stream status updates…';
    this.statusTone = 'info';
  }
}

customElements.define('eye-dashboard', EyeDashboard);
