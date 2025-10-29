import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';
import { buildFacesSettingsPayload, parseFaceTriggerPayload } from './faces-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

/**
 * Dashboard for the Faces module giving cockpits quick tuning controls.
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

      .detection-entry__note {
        margin: 0;
        font-size: 0.8rem;
        color: var(--lcars-text);
      }

      .detection-entry__value {
        font-family: var(--metric-value-font);
        word-break: break-all;
      }

      .detection-entry__note--muted {
        color: var(--lcars-muted);
      }

      .detection-entry__raw {
        margin: 0;
        font-size: 0.7rem;
        color: var(--lcars-muted);
      }

      .detection-entry__raw code {
        font-family: var(--metric-value-font);
        word-break: break-word;
        white-space: pre-wrap;
      }
    `,
  ];

  constructor() {
    super();
    this.threshold = 0.6;
    this.window = 15;
    this.publishCrops = true;
    this.publishEmbeddings = true;
    this.statusMessage = 'Connecting to recognition stream…';
    this.statusTone = 'info';
    this.tagFaceId = '';
    this.tagLabel = '';
    this.tagFeedback = '';
    this.detectionLog = [];
    this.sockets = [];
  }

  connectedCallback() {
    super.connectedCallback();
    if (!this.sockets.length) {
      this.connectTriggerStream();
    }
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    for (const socket of this.sockets) {
      try {
        socket.close();
      } catch (_error) {
        // ignore teardown issues
      }
    }
    this.sockets.length = 0;
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
          <h3 class="surface-card__title">Recognition events</h3>
          ${this.detectionLog.length
            ? html`<ol class="detection-log">
                ${this.detectionLog.map(
                  (entry) => html`<li class="detection-entry">
                    <div class="detection-entry__meta">
                      <span>${entry.timestamp}</span>
                      <span>${entry.name}</span>
                      ${entry.collection ? html`<span>${entry.collection}</span>` : ''}
                    </div>
                    <p class="detection-entry__note">
                      Memory:
                      <span class="detection-entry__value">${entry.memoryId || '—'}</span>
                    </p>
                    <p class="detection-entry__note">
                      Vector:
                      <span class="detection-entry__value">${entry.vectorId || '—'}</span>
                    </p>
                    ${entry.note
                      ? html`<p class="detection-entry__note detection-entry__note--muted">${entry.note}</p>`
                      : ''}
                    ${entry.raw
                      ? html`<p class="detection-entry__raw"><code>${entry.raw}</code></p>`
                      : ''}
                  </li>`
                )}
              </ol>`
            : html`<p class="surface-empty">No recognition events recorded yet.</p>`}
          <div class="surface-actions">
            <button type="button" class="surface-button surface-button--ghost" @click=${this.simulateDetection}>
              Simulate recognition
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

  connectTriggerStream() {
    const socket = createTopicSocket({
      module: 'faces',
      action: 'face_trigger_stream',
      type: 'std_msgs/msg/String',
      role: 'subscribe',
    });
    socket.addEventListener('open', () => {
      this.statusMessage = 'Recognition stream connected.';
      this.statusTone = 'success';
    });
    socket.addEventListener('message', (event) => {
      const payload = this.parseStreamEnvelope(event);
      if (!payload) {
        return;
      }
      if (payload.event === 'status') {
        const state = payload?.data?.state;
        if (state === 'ready') {
          this.statusMessage = 'Recognition stream ready.';
          this.statusTone = 'success';
        } else if (state === 'closed') {
          this.statusMessage = 'Recognition stream closed.';
          this.statusTone = 'warning';
        }
        return;
      }
      if (payload.event !== 'message') {
        return;
      }
      const result = parseFaceTriggerPayload(payload.data);
      if (!result.ok) {
        this.statusMessage = result.error;
        this.statusTone = 'error';
        this.recordRecognitionEvent({
          name: 'Unparsed trigger',
          note: result.error,
          raw: this.formatRawPayload(payload.data),
          collection: '',
          memoryId: '',
          vectorId: '',
        });
        return;
      }
      const eventData = result.value;
      this.statusMessage = `Recognition event: ${eventData.name}`;
      this.statusTone = 'success';
      this.recordRecognitionEvent(eventData);
    });
    socket.addEventListener('error', () => {
      this.statusMessage = 'Recognition stream error.';
      this.statusTone = 'error';
    });
    socket.addEventListener('close', () => {
      this.statusMessage = 'Recognition stream disconnected.';
      this.statusTone = 'warning';
    });
    this.sockets.push(socket);
  }

  parseStreamEnvelope(event) {
    if (!event || typeof event.data !== 'string') {
      return null;
    }
    try {
      return JSON.parse(event.data);
    } catch (_error) {
      return null;
    }
  }

  recordRecognitionEvent(details) {
    const entry = {
      id: makeId('face'),
      timestamp: new Date().toLocaleTimeString(),
      name: details?.name ?? 'Unknown',
      memoryId: details?.memoryId ?? '',
      vectorId: details?.vectorId ?? '',
      collection: details?.collection ?? '',
      note: details?.note ?? '',
      raw: details?.raw ?? '',
    };
    this.detectionLog = [entry, ...this.detectionLog].slice(0, 40);
  }

  formatRawPayload(data) {
    if (typeof data === 'string') {
      return data;
    }
    try {
      return JSON.stringify(data);
    } catch (_error) {
      return '';
    }
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
