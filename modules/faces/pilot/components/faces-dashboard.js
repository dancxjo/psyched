import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { surfaceStyles } from '/components/pilot-style.js';
import { buildFacesSettingsPayload } from './faces-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

/**
 * Dashboard for the Faces module giving pilots quick tuning controls.
 *
 * Events emitted:
 * - ``faces-settings-request`` → `{ detail: FacesSettingsPayload }`
 * - ``faces-tag-request`` → `{ detail: { faceId: string, label: string } }`
 * - ``faces-database-reset`` → `{ detail: { scope: 'embeddings' } }`
 */
class FacesDashboard extends LitElement {
  static properties = {
    threshold: { state: true },
    window: { state: true },
    publishCrops: { state: true },
    publishEmbeddings: { state: true },
    statusMessage: { state: true },
    statusTone: { state: true },
    tagFaceId: { state: true },
    tagLabel: { state: true },
    tagFeedback: { state: true },
    detectionLog: { state: true },
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
        grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
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

      .detection-log {
        list-style: none;
        padding: 0;
        margin: 0;
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
        max-height: 260px;
        overflow-y: auto;
      }

      .detection-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.25rem;
      }

      .detection-entry__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }
    `,
  ];

  constructor() {
    super();
    this.threshold = 0.6;
    this.window = 15;
    this.publishCrops = true;
    this.publishEmbeddings = true;
    this.statusMessage = 'Awaiting detector telemetry…';
    this.statusTone = 'info';
    this.tagFaceId = '';
    this.tagLabel = '';
    this.tagFeedback = '';
    this.detectionLog = [];
  }

  render() {
    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card">
          <h3 class="surface-card__title">Detector tuning</h3>
          <p class="surface-status" data-variant="${this.statusTone}">${this.statusMessage}</p>
          <form @submit=${this.handleSettingsSubmit}>
            <div class="form-row form-row--split">
              <label>
                Detection threshold
                <input
                  type="number"
                  step="0.05"
                  min="0.1"
                  max="1"
                  .value=${String(this.threshold)}
                  @input=${(event) => (this.threshold = Number(event.target.value))}
                />
              </label>
              <label>
                Smoothing window (frames)
                <input
                  type="number"
                  min="1"
                  max="120"
                  .value=${String(this.window)}
                  @input=${(event) => (this.window = Number(event.target.value))}
                />
              </label>
            </div>
            <div class="form-row form-row--split">
              <label>
                Publish crops
                <select
                  name="publishCrops"
                  .value=${this.publishCrops ? 'true' : 'false'}
                  @change=${(event) => (this.publishCrops = event.target.value === 'true')}
                >
                  <option value="true">Enabled</option>
                  <option value="false">Disabled</option>
                </select>
              </label>
              <label>
                Publish embeddings
                <select
                  name="publishEmbeddings"
                  .value=${this.publishEmbeddings ? 'true' : 'false'}
                  @change=${(event) => (this.publishEmbeddings = event.target.value === 'true')}
                >
                  <option value="true">Enabled</option>
                  <option value="false">Disabled</option>
                </select>
              </label>
            </div>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Apply tuning</button>
              <button type="button" class="surface-button surface-button--ghost" @click=${this.resetEmbeddings}>
                Reset embedding cache
              </button>
            </div>
          </form>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Latest detections</h3>
          ${this.detectionLog.length
            ? html`<ol class="detection-log">
                ${this.detectionLog.map(
                  (entry) => html`<li class="detection-entry">
                    <div class="detection-entry__meta">
                      <span>${entry.timestamp}</span>
                      <span>${entry.label}</span>
                      <span>${(entry.confidence * 100).toFixed(1)}%</span>
                    </div>
                    <p class="detection-entry__note">ROI: ${entry.roi}</p>
                  </li>`
                )}
              </ol>`
            : html`<p class="surface-empty">No detections recorded yet.</p>`}
          <div class="surface-actions">
            <button type="button" class="surface-button surface-button--ghost" @click=${this.simulateDetection}>
              Simulate detection
            </button>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Manual labelling</h3>
          ${this.tagFeedback
            ? html`<p class="surface-status" data-variant="error">${this.tagFeedback}</p>`
            : ''}
          <form @submit=${this.handleTagSubmit}>
            <label>
              Face identifier
              <input
                type="text"
                name="faceId"
                .value=${this.tagFaceId}
                @input=${(event) => (this.tagFaceId = event.target.value)}
              />
            </label>
            <label>
              Label
              <input
                type="text"
                name="label"
                .value=${this.tagLabel}
                @input=${(event) => (this.tagLabel = event.target.value)}
              />
            </label>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Submit label</button>
              <button type="button" class="surface-button surface-button--ghost" @click=${this.clearTagForm}>
                Clear
              </button>
            </div>
          </form>
        </article>
      </div>
    `;
  }

  handleSettingsSubmit(event) {
    event.preventDefault();
    const payload = buildFacesSettingsPayload({
      threshold: this.threshold,
      window: this.window,
      publishCrops: this.publishCrops,
      publishEmbeddings: this.publishEmbeddings,
    });
    if (!payload.ok) {
      this.statusMessage = payload.error;
      this.statusTone = 'error';
      return;
    }
    this.dispatchEvent(
      new CustomEvent('faces-settings-request', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = 'Detector settings submitted for synchronisation.';
    this.statusTone = 'success';
  }

  handleTagSubmit(event) {
    event.preventDefault();
    const faceId = this.tagFaceId.trim();
    const label = this.tagLabel.trim();
    if (!faceId || !label) {
      this.tagFeedback = 'Face identifier and label are required.';
      return;
    }
    this.dispatchEvent(
      new CustomEvent('faces-tag-request', {
        detail: { faceId, label },
        bubbles: true,
        composed: true,
      }),
    );
    this.tagFeedback = '';
    this.clearTagForm();
  }

  clearTagForm() {
    this.tagFaceId = '';
    this.tagLabel = '';
  }

  resetEmbeddings() {
    this.dispatchEvent(
      new CustomEvent('faces-database-reset', {
        detail: { scope: 'embeddings' },
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = 'Embedding cache reset request queued.';
    this.statusTone = 'warning';
  }

  simulateDetection() {
    const entry = {
      id: makeId('face'),
      timestamp: new Date().toLocaleTimeString(),
      label: 'unknown',
      confidence: Math.random() * 0.4 + this.threshold,
      roi: `${Math.floor(Math.random() * 640)},${Math.floor(Math.random() * 480)} ${
        128 + Math.floor(Math.random() * 64)
      }×${128 + Math.floor(Math.random() * 64)}`,
    };
    this.detectionLog = [entry, ...this.detectionLog].slice(0, 40);
  }
}

customElements.define('faces-dashboard', FacesDashboard);
