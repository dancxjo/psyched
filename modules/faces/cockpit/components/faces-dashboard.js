import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket, callRosService } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';
import { buildFacesSettingsPayload, parseFaceTriggerPayload } from './faces-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

const ROUTER_SOURCE_OPTIONS = [
  ['kinect', 'Kinect'],
  ['usb', 'USB camera'],
  ['auto', 'Auto'],
  ['off', 'Disabled'],
];

const PARAMETER_TYPE_BOOL = 1;
const PARAMETER_TYPE_STRING = 4;

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
    routerStatus: { state: true },
    routerTone: { state: true },
    routerSource: { state: true },
    routerFallback: { state: true },
    kinectEnabled: { state: true },
    usbEnabled: { state: true },
    routerTopics: { state: true },
    routerBusy: { state: true },
    routerAvailable: { state: true },
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

      .router-topics {
        margin: 0;
        display: grid;
        gap: 0.25rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .router-topics__entry {
        display: flex;
        flex-direction: column;
        gap: 0.1rem;
      }

      .router-topics__label {
        text-transform: uppercase;
        letter-spacing: 0.05em;
      }

      .router-topics__value {
        font-family: var(--metric-value-font);
        color: var(--lcars-text);
        word-break: break-word;
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
    this.routerStatus = 'Querying faces router state…';
    this.routerTone = 'info';
    this.routerSource = 'kinect';
    this.routerFallback = 'auto';
    this.kinectEnabled = true;
    this.usbEnabled = false;
    this.routerTopics = {
      kinectImage: '/camera/color/image_raw',
      kinectInfo: '/camera/color/camera_info',
      usbImage: '/eye/usb/image_raw',
      usbInfo: '/eye/usb/camera_info',
      outputImage: '/faces/camera/image_raw',
      outputInfo: '/faces/camera/camera_info',
    };
    this.routerBusy = false;
    this.routerAvailable = false;
    this.tagFaceId = '';
    this.tagLabel = '';
    this.tagFeedback = '';
    this.detectionLog = [];
    this.sockets = [];
    this._routerRetryHandle = null;
    this._routerRetryAttempt = 0;
    this._routerRetryLimit = 5;
    this._routerRetryDelayMs = 3000;
  }

  connectedCallback() {
    super.connectedCallback();
    if (!this.sockets.length) {
      this.connectTriggerStream();
    }
    void this.refreshRouterState();
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
    if (this._routerRetryHandle) {
      clearTimeout(this._routerRetryHandle);
      this._routerRetryHandle = null;
    }
  }

  render() {
    const routerControlsDisabled = this.routerBusy || !this.routerAvailable;
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

        <article class="surface-card">
          <h3 class="surface-card__title">Faces routing</h3>
          <p class="surface-status" data-variant="${this.routerTone}">${this.routerStatus}</p>
          <form @submit=${this.applyRouterSettings}>
            <div class="form-row form-row--split">
              <label>
                Active source
                <select
                  .value=${this.routerSource}
                  @change=${(event) => (this.routerSource = event.target.value)}
                  ?disabled=${routerControlsDisabled}
                >
                  ${ROUTER_SOURCE_OPTIONS.map(
                    ([value, label]) => html`<option value=${value}>${label}</option>`,
                  )}
                </select>
              </label>
              <label>
                Fallback source
                <select
                  .value=${this.routerFallback}
                  @change=${(event) => (this.routerFallback = event.target.value)}
                  ?disabled=${routerControlsDisabled}
                >
                  ${ROUTER_SOURCE_OPTIONS.map(
                    ([value, label]) => html`<option value=${value}>${label}</option>`,
                  )}
                </select>
              </label>
            </div>
            <div class="form-row form-row--split">
              <label>
                Kinect feed enabled
                <select
                  .value=${this.kinectEnabled ? 'true' : 'false'}
                  @change=${(event) => (this.kinectEnabled = event.target.value === 'true')}
                  ?disabled=${routerControlsDisabled}
                >
                  <option value="true">Enabled</option>
                  <option value="false">Disabled</option>
                </select>
              </label>
              <label>
                USB feed enabled
                <select
                  .value=${this.usbEnabled ? 'true' : 'false'}
                  @change=${(event) => (this.usbEnabled = event.target.value === 'true')}
                  ?disabled=${routerControlsDisabled}
                >
                  <option value="true">Enabled</option>
                  <option value="false">Disabled</option>
                </select>
              </label>
            </div>
            <div class="router-topics">
              <div class="router-topics__entry">
                <span class="router-topics__label">Kinect image</span>
                <span class="router-topics__value">${this.routerTopics.kinectImage || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">Kinect info</span>
                <span class="router-topics__value">${this.routerTopics.kinectInfo || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">USB image</span>
                <span class="router-topics__value">${this.routerTopics.usbImage || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">USB info</span>
                <span class="router-topics__value">${this.routerTopics.usbInfo || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">Faces output image</span>
                <span class="router-topics__value">${this.routerTopics.outputImage || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">Faces output info</span>
                <span class="router-topics__value">${this.routerTopics.outputInfo || '—'}</span>
              </div>
            </div>
            <div class="surface-actions">
              <button type="submit" class="surface-button" ?disabled=${routerControlsDisabled}>
                ${this.routerBusy && this.routerAvailable ? 'Applying…' : 'Apply routing'}
              </button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click=${() => this.refreshRouterState({ manual: true })}
                ?disabled=${this.routerBusy}
              >
                Refresh state
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

  async refreshRouterState(options = {}) {
    const manual = Boolean(options.manual);
    if (manual) {
      this._routerRetryAttempt = 0;
    }
    if (this._routerRetryHandle) {
      clearTimeout(this._routerRetryHandle);
      this._routerRetryHandle = null;
    }
    const previousBusy = this.routerBusy;
    this.routerBusy = true;
    this.routerStatus = 'Querying faces router state…';
    this.routerTone = 'info';

    const parameterNames = [
      'faces_source',
      'fallback_source',
      'kinect_enabled',
      'usb_enabled',
      'kinect_image_topic',
      'kinect_camera_info_topic',
      'usb_image_topic',
      'usb_camera_info_topic',
      'output_image_topic',
      'output_camera_info_topic',
    ];

    try {
      const response = await callRosService({
        module: 'eye',
        service: '/psyched_faces_router/get_parameters',
        type: 'rcl_interfaces/srv/GetParameters',
        args: { names: parameterNames },
        timeoutMs: 4000,
      });

      const values = Array.isArray(response?.values) ? response.values : [];
      const lookup = new Map();
      for (let index = 0; index < parameterNames.length; index += 1) {
        lookup.set(parameterNames[index], values[index] ?? null);
      }

      this.routerSource = this._extractString(lookup.get('faces_source'), this.routerSource);
      this.routerFallback = this._extractString(lookup.get('fallback_source'), this.routerFallback);
      this.kinectEnabled = this._extractBool(lookup.get('kinect_enabled'), this.kinectEnabled);
      this.usbEnabled = this._extractBool(lookup.get('usb_enabled'), this.usbEnabled);
      this.routerTopics = {
        kinectImage: this._extractString(lookup.get('kinect_image_topic'), this.routerTopics.kinectImage),
        kinectInfo: this._extractString(lookup.get('kinect_camera_info_topic'), this.routerTopics.kinectInfo),
        usbImage: this._extractString(lookup.get('usb_image_topic'), this.routerTopics.usbImage),
        usbInfo: this._extractString(lookup.get('usb_camera_info_topic'), this.routerTopics.usbInfo),
        outputImage: this._extractString(lookup.get('output_image_topic'), this.routerTopics.outputImage),
        outputInfo: this._extractString(lookup.get('output_camera_info_topic'), this.routerTopics.outputInfo),
      };

      this.routerStatus = 'Faces router state synchronised.';
      this.routerTone = 'success';
      this.routerAvailable = true;
      this._routerRetryAttempt = 0;
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      const unavailable = typeof message === 'string' && message.toLowerCase().includes('not available');
      if (unavailable && this._routerRetryAttempt < this._routerRetryLimit) {
        this._routerRetryAttempt += 1;
        const delaySeconds = Math.max(this._routerRetryDelayMs / 1000, 0.1);
        this.routerStatus = `Faces router is starting up (waiting for eye module); retrying in ${delaySeconds.toFixed(1)}s…`;
        this.routerTone = 'warning';
        this.routerAvailable = false;
        this._routerRetryHandle = setTimeout(() => {
          this._routerRetryHandle = null;
          void this.refreshRouterState();
        }, this._routerRetryDelayMs);
      } else {
        this.routerStatus = `Faces router unavailable: ${message}. Ensure the eye module is running.`;
        this.routerTone = 'error';
        this.routerAvailable = false;
      }
    } finally {
      this.routerBusy = previousBusy;
    }
  }

  async applyRouterSettings(event) {
    event.preventDefault();
    if (this.routerBusy) {
      return;
    }
    if (!this.routerAvailable) {
      this.routerStatus = 'Faces router service is unavailable.';
      this.routerTone = 'error';
      return;
    }

    this.routerBusy = true;
    this.routerStatus = 'Applying routing updates…';
    this.routerTone = 'info';

    const parameters = [
      this._buildStringParameter('faces_source', this.routerSource),
      this._buildStringParameter('fallback_source', this.routerFallback),
      this._buildBoolParameter('kinect_enabled', this.kinectEnabled),
      this._buildBoolParameter('usb_enabled', this.usbEnabled),
    ].filter(Boolean);

    if (!parameters.length) {
      this.routerBusy = false;
      this.routerStatus = 'No routing changes detected.';
      this.routerTone = 'warning';
      return;
    }

    try {
      await callRosService({
        module: 'eye',
        service: '/psyched_faces_router/set_parameters',
        type: 'rcl_interfaces/srv/SetParameters',
        args: { parameters },
        timeoutMs: 5000,
      });
      await this.refreshRouterState({ manual: true });
      if (this.routerAvailable) {
        this.routerStatus = 'Faces routing updated.';
        this.routerTone = 'success';
      }
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.routerStatus = `Failed to update faces routing: ${message}`;
      this.routerTone = 'error';
    } finally {
      this.routerBusy = false;
    }
  }

  _extractString(parameterValue, fallback = '') {
    if (!parameterValue || typeof parameterValue !== 'object') {
      return fallback;
    }
    const text = typeof parameterValue.string_value === 'string' ? parameterValue.string_value.trim() : '';
    if (text) {
      return text;
    }
    if (Array.isArray(parameterValue.string_array_value) && parameterValue.string_array_value.length > 0) {
      const first = parameterValue.string_array_value[0];
      if (typeof first === 'string' && first.trim()) {
        return first.trim();
      }
    }
    return fallback;
  }

  _extractBool(parameterValue, fallback = false) {
    if (!parameterValue || typeof parameterValue !== 'object') {
      return fallback;
    }
    if (typeof parameterValue.bool_value === 'boolean') {
      return parameterValue.bool_value;
    }
    if (typeof parameterValue.integer_value === 'number') {
      return parameterValue.integer_value !== 0;
    }
    return fallback;
  }

  _buildStringParameter(name, value) {
    if (typeof value !== 'string' || !value.trim()) {
      return null;
    }
    return {
      name,
      value: {
        type: PARAMETER_TYPE_STRING,
        string_value: value.trim(),
      },
    };
  }

  _buildBoolParameter(name, value) {
    if (typeof value !== 'boolean') {
      return null;
    }
    return {
      name,
      value: {
        type: PARAMETER_TYPE_BOOL,
        bool_value: value,
      },
    };
  }

  connectTriggerStream() {
    const socket = createTopicSocket({
      module: 'faces',
      action: 'face_trigger_stream',
      type: 'psyched_msgs/msg/SensationStamped',
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
        if (result.reason === 'ignored' || result.reason === 'empty') {
          return;
        }
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
    const memoryId = `mem-${Math.random().toString(16).slice(2, 10)}`;
    const vectorId = `vec-${Math.random().toString(16).slice(2, 10)}`;
    const payload = {
      name: 'Simulated face',
      memoryId,
      vectorId,
      collection: 'faces',
      note: 'Simulated recognition event.',
      raw: JSON.stringify({
        name: 'Simulated face',
        memory_id: memoryId,
        vector_id: vectorId,
        collection: 'faces',
      }),
    };
    this.recordRecognitionEvent(payload);
    this.statusMessage = 'Simulated recognition event recorded.';
    this.statusTone = 'info';
  }
}

customElements.define('faces-dashboard', FacesDashboard);
