
import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';
import {
  buildEyeSettingsPayload,
  normalizeDepthMode,
} from './eye-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

const DEFAULT_PREVIEW_STREAMS = [
  ['/camera/color/image_raw/compressed', 'sensor_msgs/msg/CompressedImage'],
  ['/camera/color/image_raw', 'sensor_msgs/msg/Image'],
  ['/kinect_ros2/image_raw/compressed', 'sensor_msgs/msg/CompressedImage'],
  ['/kinect_ros2/image_raw', 'sensor_msgs/msg/Image'],
  ['/image_raw/compressed', 'sensor_msgs/msg/CompressedImage'],
  ['/image_raw', 'sensor_msgs/msg/Image'],
];

const PREVIEW_TIMEOUT_MS = 6000;
const PREVIEW_RECONNECT_DELAY_MS = 1500;

/**
 * Dashboard for the Eye module exposing Kinect stream controls.
 *
 * Custom events:
 * - ``eye-settings-request`` → `{ detail: EyeSettingsPayload }`
 * - ``eye-capture-request`` → `{ detail: { mode: 'color' | 'depth' } }`
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
    previewMode: { state: true },
    previewFrameUrl: { state: true },
    previewFrameReady: { state: true },
    previewSource: { state: true },
    previewEncoding: { state: true },
    previewWidth: { state: true },
    previewHeight: { state: true },
    previewTimestamp: { state: true },
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
        padding: 0.5rem;
        position: relative;
        overflow: hidden;
      }

      .preview__frame {
        display: block;
        width: 100%;
        height: auto;
        border-radius: 0.65rem;
        box-shadow: 0 0 18px rgba(0, 0, 0, 0.4);
      }

      .preview__placeholder {
        display: grid;
        gap: 0.25rem;
      }

      .preview__details {
        margin: 0.5rem 0 0;
        font-size: 0.7rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--lcars-muted);
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
    this.previewMode = 'idle';
    this.previewFrameUrl = '';
    this.previewFrameReady = false;
    this.previewSource = '';
    this.previewEncoding = '';
    this.previewWidth = this.width;
    this.previewHeight = this.height;
    this.previewTimestamp = '';

    this._previewCandidates = [];
    this._previewCandidateIndex = 0;
    this._activePreviewCandidate = null;
    this._previewSocket = null;
    this._previewTimeoutId = null;
    this._reconnectTimer = null;
    this._latestRawFrame = null;
    this._latestCompressedFrame = null;
    this._frameRenderScheduled = false;
    this._compressedRenderScheduled = false;
    this._imageDataCache = null;
    this._connected = false;

    this._handlePreviewEvent = this._handlePreviewEvent.bind(this);
    this._handlePreviewError = this._handlePreviewError.bind(this);
    this._handlePreviewClose = this._handlePreviewClose.bind(this);
  }

  connectedCallback() {
    super.connectedCallback();
    this._connected = true;
    this._previewCandidates = this._resolvePreviewCandidates();
    this._previewCandidateIndex = 0;
    this._startPreviewStream();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._connected = false;
    this._teardownPreview();
  }

  render() {
    const hasFrame = this.previewFrameReady && (this.previewMode === 'image' ? Boolean(this.previewFrameUrl) : this.previewMode === 'canvas');
    const details = hasFrame
      ? [
        this.previewSource ? `Topic: ${this.previewSource}` : '',
        this.previewEncoding ? `Encoding: ${this.previewEncoding}` : '',
        this.previewWidth && this.previewHeight ? `${this.previewWidth}×${this.previewHeight}` : '',
        this.previewTimestamp ? `Last frame: ${this.previewTimestamp}` : '',
      ].filter(Boolean).join(' • ')
      : '';

    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Stream preview</h3>
          <p class="surface-status" data-variant="${this.statusTone}">${this.statusMessage}</p>
          <div class="preview" role="img" aria-label="Live video preview">
            ${this.previewMode === 'image' && this.previewFrameUrl
        ? html`<img class="preview__frame" src="${this.previewFrameUrl}" alt="Eye color stream preview" />`
        : ''}
            ${this.previewMode === 'canvas'
        ? html`<canvas
                  class="preview__frame preview__canvas"
                  width="${this.previewWidth || this.width}"
                  height="${this.previewHeight || this.height}"
                ></canvas>`
        : ''}
            ${!hasFrame
        ? html`<div class="preview__placeholder">
                  <p>${this.previewSource ? `Waiting for ${this.previewSource}` : 'Initialising preview stream…'}</p>
                  <p>${this.width}×${this.height} @ ${this.frameRate} FPS</p>
                </div>`
        : ''}
          </div>
          ${details ? html`<p class="preview__details">${details}</p>` : ''}
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
                  <option value="disabled">Color only</option>
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
            <button type="button" class="surface-button" @click=${() => this.handleCapture('color')}>
              Capture color frame
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
        note: mode === 'depth' ? 'Depth frame queued for export.' : 'Color frame queued for export.',
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

  _resolvePreviewCandidates() {
    const attribute = this.getAttribute('data-preview-topics') || this.dataset.previewTopics || '';
    const parsed = this._parsePreviewAttribute(attribute);
    const defaults = DEFAULT_PREVIEW_STREAMS.map(([topic, messageType]) => ({ topic, messageType }));
    const seen = new Set();
    return [...parsed, ...defaults].filter((candidate) => {
      if (!candidate || typeof candidate.topic !== 'string' || typeof candidate.messageType !== 'string') {
        return false;
      }
      const topic = candidate.topic.trim();
      const messageType = candidate.messageType.trim();
      if (!topic || !messageType) {
        return false;
      }
      const key = `${topic}|${messageType}`;
      if (seen.has(key)) {
        return false;
      }
      seen.add(key);
      return true;
    });
  }

  _parsePreviewAttribute(value) {
    if (!value || typeof value !== 'string') {
      return [];
    }
    return value
      .split(',')
      .map((entry) => entry.trim())
      .filter(Boolean)
      .map((entry) => {
        const [topicPart, typePart] = entry.split('|').map((chunk) => chunk.trim());
        if (!topicPart) {
          return null;
        }
        const messageType = typePart || (topicPart.endsWith('/compressed') ? 'sensor_msgs/msg/CompressedImage' : 'sensor_msgs/msg/Image');
        return { topic: topicPart, messageType };
      })
      .filter(Boolean);
  }

  _startPreviewStream() {
    if (!this._connected) {
      return;
    }
    if (this._previewCandidateIndex >= this._previewCandidates.length) {
      this.statusMessage = 'No preview streams configured.';
      this.statusTone = 'error';
      return;
    }
    this._clearPreviewSocket();
    if (this._previewTimeoutId) {
      clearTimeout(this._previewTimeoutId);
      this._previewTimeoutId = null;
    }
    if (this._reconnectTimer) {
      clearTimeout(this._reconnectTimer);
      this._reconnectTimer = null;
    }

    const candidate = this._previewCandidates[this._previewCandidateIndex];
    this._activePreviewCandidate = candidate;
    this.previewSource = candidate.topic;
    this.statusMessage = `Connecting to ${candidate.topic}…`;
    this.statusTone = 'info';
    this._resetPreviewState();

    try {
      const socket = createTopicSocket({
        module: 'eye',
        topic: candidate.topic,
        type: candidate.messageType,
        role: 'subscribe',
        queueLength: 2,
      });
      this._previewSocket = socket;
      socket.addEventListener('message', this._handlePreviewEvent);
      socket.addEventListener('error', this._handlePreviewError);
      socket.addEventListener('close', this._handlePreviewClose);
      socket.ready.catch((error) => this._handlePreviewError(error));
      this._previewTimeoutId = setTimeout(() => this._handlePreviewTimeout(), PREVIEW_TIMEOUT_MS);
    } catch (error) {
      console.error('Failed to initialise preview stream', error);
      this._advancePreviewCandidate();
    }
  }

  _teardownPreview() {
    if (this._previewTimeoutId) {
      clearTimeout(this._previewTimeoutId);
      this._previewTimeoutId = null;
    }
    if (this._reconnectTimer) {
      clearTimeout(this._reconnectTimer);
      this._reconnectTimer = null;
    }
    this._latestRawFrame = null;
    this._latestCompressedFrame = null;
    this._frameRenderScheduled = false;
    this._compressedRenderScheduled = false;
    this._imageDataCache = null;
    this._activePreviewCandidate = null;
    this._clearPreviewSocket();
    this._resetPreviewState();
  }

  _resetPreviewState() {
    this.previewMode = 'idle';
    this.previewFrameUrl = '';
    this.previewFrameReady = false;
    this.previewEncoding = '';
    this.previewTimestamp = '';
    this.previewWidth = this.width;
    this.previewHeight = this.height;
    this._latestRawFrame = null;
    this._latestCompressedFrame = null;
    this._frameRenderScheduled = false;
    this._compressedRenderScheduled = false;
  }

  _clearPreviewSocket() {
    if (!this._previewSocket) {
      return;
    }
    this._previewSocket.removeEventListener('message', this._handlePreviewEvent);
    this._previewSocket.removeEventListener('error', this._handlePreviewError);
    this._previewSocket.removeEventListener('close', this._handlePreviewClose);
    try {
      this._previewSocket.close();
    } catch (_error) {
      // Ignore shutdown errors.
    }
    this._previewSocket = null;
  }

  _handlePreviewEvent(event) {
    if (!event || typeof event.data !== 'string') {
      return;
    }
    let payload;
    try {
      payload = JSON.parse(event.data);
    } catch (_error) {
      return;
    }
    if (!payload || typeof payload !== 'object') {
      return;
    }
    if (payload.event === 'status') {
      this._handlePreviewStatus(payload.data || {});
      return;
    }
    if (payload.event !== 'message') {
      return;
    }
    const message = payload.data || {};
    if (message && typeof message === 'object' && Array.isArray(message.data) && typeof message.format === 'string') {
      if (this._renderCompressedFrame(message)) {
        this._onPreviewFrameReceived();
      }
      return;
    }
    if (message && typeof message === 'object' && 'encoding' in message && message.data) {
      if (this._renderRawFrame(message)) {
        this._onPreviewFrameReceived();
      }
    }
  }

  _handlePreviewStatus(status) {
    const state = typeof status?.state === 'string' ? status.state.toLowerCase() : '';
    if (state === 'ready' && !this.previewFrameReady) {
      this.statusMessage = 'Connected. Waiting for frames…';
      this.statusTone = 'info';
    } else if (state === 'closed') {
      this.previewFrameReady = false;
      this.statusMessage = 'Stream closed by backend. Reconnecting…';
      this.statusTone = 'warning';
      this._scheduleReconnect();
    }
  }

  _handlePreviewError(error) {
    if (!this._connected) {
      return;
    }
    console.warn('Eye preview socket error', error);
    if (!this.previewFrameReady) {
      this.statusMessage = 'Preview stream unavailable, trying fallback…';
      this.statusTone = 'warning';
      this._advancePreviewCandidate();
    } else {
      this.statusMessage = 'Stream error encountered. Reconnecting…';
      this.statusTone = 'warning';
      this.previewFrameReady = false;
      this._scheduleReconnect();
    }
  }

  _handlePreviewClose() {
    if (!this._connected) {
      return;
    }
    if (!this.previewFrameReady) {
      this._advancePreviewCandidate();
    } else {
      this.statusMessage = 'Stream disconnected. Attempting reconnect…';
      this.statusTone = 'warning';
      this.previewFrameReady = false;
      this._scheduleReconnect();
    }
  }

  _handlePreviewTimeout() {
    this._previewTimeoutId = null;
    if (this.previewFrameReady) {
      return;
    }
    this.statusMessage = 'No frames received; checking alternate stream…';
    this.statusTone = 'warning';
    this._advancePreviewCandidate();
  }

  _advancePreviewCandidate() {
    this._clearPreviewSocket();
    if (this._previewTimeoutId) {
      clearTimeout(this._previewTimeoutId);
      this._previewTimeoutId = null;
    }
    this.previewFrameReady = false;
    this._previewCandidateIndex += 1;
    if (this._previewCandidateIndex >= this._previewCandidates.length) {
      this.statusMessage = 'Unable to open preview stream.';
      this.statusTone = 'error';
      return;
    }
    this._startPreviewStream();
  }

  _scheduleReconnect() {
    if (this._reconnectTimer || !this._connected) {
      return;
    }
    this._reconnectTimer = setTimeout(() => {
      this._reconnectTimer = null;
      if (!this._connected) {
        return;
      }
      this._startPreviewStream();
    }, PREVIEW_RECONNECT_DELAY_MS);
  }

  _onPreviewFrameReceived() {
    if (this._previewTimeoutId) {
      clearTimeout(this._previewTimeoutId);
      this._previewTimeoutId = null;
    }
    if (!this.previewFrameReady) {
      this.statusMessage = 'Live stream detected.';
      this.statusTone = 'success';
    }
    this.previewFrameReady = true;
    this.previewTimestamp = new Date().toLocaleTimeString();
  }

  _renderCompressedFrame(message) {
    const data = Array.isArray(message.data) ? Uint8Array.from(message.data) : message.data;
    if (!(data instanceof Uint8Array) || data.length === 0) {
      return false;
    }
    const mimeType = this._resolveMimeType(message.format);
    const blob = new Blob([data], { type: mimeType });
    this.previewMode = 'canvas';
    this.previewEncoding = `compressed (${mimeType.replace('image/', '')})`;
    if (typeof message.width === 'number' && message.width > 0) {
      this.previewWidth = message.width;
    }
    if (typeof message.height === 'number' && message.height > 0) {
      this.previewHeight = message.height;
    }
    this.previewFrameUrl = '';
    this._latestCompressedFrame = {
      blob,
      width: this.previewWidth,
      height: this.previewHeight,
    };
    this._scheduleCompressedFrameDraw();
    return true;
  }

  _scheduleCompressedFrameDraw() {
    if (this._compressedRenderScheduled) {
      return;
    }
    this._compressedRenderScheduled = true;
    requestAnimationFrame(() => this._flushCompressedFrame());
  }

  async _flushCompressedFrame() {
    this._compressedRenderScheduled = false;
    const frame = this._latestCompressedFrame;
    if (!frame || !this._connected) {
      return;
    }
    await this.updateComplete;
    if (!this._connected) {
      return;
    }
    const canvas = this.renderRoot?.querySelector('.preview__canvas');
    if (!(canvas instanceof HTMLCanvasElement)) {
      return;
    }
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      return;
    }

    let bitmap;
    try {
      bitmap = await this._decodeCompressedFrame(frame.blob);
    } catch (error) {
      console.warn('Failed to decode compressed eye frame', error);
      this.previewFrameReady = false;
      return;
    }

    const width = frame.width || bitmap.width || this.previewWidth;
    const height = frame.height || bitmap.height || this.previewHeight;

    if (width && width > 0) {
      this.previewWidth = width;
      if (canvas.width !== width) {
        canvas.width = width;
      }
    }
    if (height && height > 0) {
      this.previewHeight = height;
      if (canvas.height !== height) {
        canvas.height = height;
      }
    }

    try {
      ctx.drawImage(bitmap, 0, 0, canvas.width, canvas.height);
    } catch (error) {
      console.warn('Failed to draw compressed eye frame', error);
      this.previewFrameReady = false;
    } finally {
      if (bitmap && typeof bitmap.close === 'function') {
        bitmap.close();
      }
    }
  }

  async _decodeCompressedFrame(blob) {
    if (typeof createImageBitmap === 'function') {
      return createImageBitmap(blob);
    }
    const dataUrl = await new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.onload = () => resolve(reader.result);
      reader.onerror = () => reject(new Error('Failed to read compressed frame payload'));
      reader.readAsDataURL(blob);
    });
    if (typeof dataUrl !== 'string') {
      throw new Error('Compressed frame did not contain valid image data');
    }
    const image = new Image();
    image.decoding = 'async';
    await new Promise((resolve, reject) => {
      image.onload = () => resolve();
      image.onerror = () => reject(new Error('Failed to load compressed frame image'));
      image.src = dataUrl;
    });
    return image;
  }

  _renderRawFrame(message) {
    const encoding = typeof message.encoding === 'string' ? message.encoding.toLowerCase() : '';
    const width = Number(message.width) || 0;
    const height = Number(message.height) || 0;
    if (!encoding || !width || !height) {
      return false;
    }
    const buffer = this._toUint8Array(message.data);
    if (!buffer.length) {
      return false;
    }
    this.previewMode = 'canvas';
    this.previewEncoding = encoding;
    this.previewWidth = width;
    this.previewHeight = height;
    this._latestRawFrame = { message, buffer };
    this._scheduleRawFrameDraw();
    return true;
  }

  _scheduleRawFrameDraw() {
    if (this._frameRenderScheduled) {
      return;
    }
    this._frameRenderScheduled = true;
    requestAnimationFrame(() => this._flushRawFrame());
  }

  async _flushRawFrame() {
    this._frameRenderScheduled = false;
    const frame = this._latestRawFrame;
    if (!frame || !this._connected) {
      return;
    }
    await this.updateComplete;
    if (!this._connected) {
      return;
    }
    const canvas = this.renderRoot?.querySelector('.preview__canvas');
    if (!(canvas instanceof HTMLCanvasElement)) {
      return;
    }
    if (canvas.width !== this.previewWidth) {
      canvas.width = this.previewWidth;
    }
    if (canvas.height !== this.previewHeight) {
      canvas.height = this.previewHeight;
    }
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      return;
    }
    const drawn = this._drawImageData(ctx, frame.message, frame.buffer);
    if (!drawn) {
      this.previewFrameReady = false;
    }
  }

  _drawImageData(ctx, message, buffer) {
    const encoding = typeof message.encoding === 'string' ? message.encoding.toLowerCase() : '';
    const width = Number(message.width) || 0;
    const height = Number(message.height) || 0;
    const bytesPerPixel = this._bytesPerPixel(encoding);
    if (!bytesPerPixel || !width || !height) {
      return false;
    }
    const rowStride = Number(message.step) || width * bytesPerPixel;
    if (buffer.length < rowStride * height) {
      return false;
    }
    if (!this._imageDataCache || this._imageDataCache.width !== width || this._imageDataCache.height !== height) {
      this._imageDataCache = ctx.createImageData(width, height);
    }
    const target = this._imageDataCache.data;
    const bigEndian = Boolean(message.is_bigendian);
    for (let y = 0; y < height; y += 1) {
      const srcRow = y * rowStride;
      const destRow = y * width * 4;
      for (let x = 0; x < width; x += 1) {
        const srcIndex = srcRow + x * bytesPerPixel;
        const destIndex = destRow + x * 4;
        if (destIndex + 3 >= target.length || srcIndex >= buffer.length) {
          continue;
        }
        if (encoding === 'rgb8') {
          target[destIndex] = buffer[srcIndex];
          target[destIndex + 1] = buffer[srcIndex + 1];
          target[destIndex + 2] = buffer[srcIndex + 2];
          target[destIndex + 3] = 255;
        } else if (encoding === 'bgr8') {
          target[destIndex] = buffer[srcIndex + 2];
          target[destIndex + 1] = buffer[srcIndex + 1];
          target[destIndex + 2] = buffer[srcIndex];
          target[destIndex + 3] = 255;
        } else if (encoding === 'mono8') {
          const value = buffer[srcIndex];
          target[destIndex] = value;
          target[destIndex + 1] = value;
          target[destIndex + 2] = value;
          target[destIndex + 3] = 255;
        } else if (encoding === 'mono16') {
          const byteA = buffer[srcIndex + (bigEndian ? 0 : 1)] ?? 0;
          const byteB = buffer[srcIndex + (bigEndian ? 1 : 0)] ?? 0;
          const value = (byteA << 8) | byteB;
          const normalized = Math.min(255, Math.max(0, value >> 8));
          target[destIndex] = normalized;
          target[destIndex + 1] = normalized;
          target[destIndex + 2] = normalized;
          target[destIndex + 3] = 255;
        }
      }
    }
    ctx.putImageData(this._imageDataCache, 0, 0);
    return true;
  }

  _resolveMimeType(format) {
    const value = typeof format === 'string' ? format.toLowerCase() : '';
    if (value.includes('png')) {
      return 'image/png';
    }
    if (value.includes('bmp')) {
      return 'image/bmp';
    }
    return 'image/jpeg';
  }

  _bytesPerPixel(encoding) {
    switch (encoding) {
      case 'rgb8':
      case 'bgr8':
        return 3;
      case 'mono8':
        return 1;
      case 'mono16':
        return 2;
      default:
        return 0;
    }
  }

  _toUint8Array(data) {
    if (data instanceof Uint8Array) {
      return data;
    }
    if (Array.isArray(data)) {
      return Uint8Array.from(data);
    }
    if (data && typeof data === 'object' && typeof data.length === 'number') {
      try {
        return Uint8Array.from(data);
      } catch (_error) {
        return new Uint8Array();
      }
    }
    return new Uint8Array();
  }
}

customElements.define('eye-dashboard', EyeDashboard);
