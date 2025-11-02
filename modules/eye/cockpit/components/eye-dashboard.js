
import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket, callModuleAction, callRosService } from '/js/cockpit.js';
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
const PARAMETER_TYPE_BOOL = 1;
const PARAMETER_TYPE_STRING = 4;
const ROUTER_SOURCE_OPTIONS = [
  ['auto', 'Auto'],
  ['kinect', 'Kinect'],
  ['usb', 'USB camera'],
  ['off', 'Disabled'],
];

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
    routerStatus: { state: true },
    routerTone: { state: true },
    routerBusy: { state: true },
    routerAvailable: { state: true },
    routerSource: { state: true },
    routerFallback: { state: true },
    kinectEnabled: { state: true },
    usbEnabled: { state: true },
    routerTopics: { state: true },
    videoDevices: { state: true },
    usbDevicePath: { state: true },
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

      .router-topics {
        margin: 0;
        padding: 0;
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

      .device-list {
        list-style: none;
        padding: 0;
        margin: 0;
        display: grid;
        gap: 0.4rem;
      }

      .device-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.3rem;
      }

      .device-entry__label {
        font-family: var(--metric-value-font);
        color: var(--lcars-text);
      }

      .device-entry__path {
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .device-entry__flags {
        font-size: 0.7rem;
        color: var(--lcars-muted);
        letter-spacing: 0.05em;
        text-transform: uppercase;
      }

      .device-entry__actions {
        display: flex;
        gap: 0.5rem;
        align-items: center;
        font-size: 0.7rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--lcars-muted);
      }

      .device-entry__badge {
        display: inline-flex;
        align-items: center;
        padding: 0.25rem 0.55rem;
        border-radius: 999px;
        background: rgba(255, 255, 255, 0.12);
        color: var(--lcars-text);
        font-family: var(--metric-value-font);
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

    this.routerStatus = 'Checking faces router…';
    this.routerTone = 'info';
    this.routerBusy = false;
    this.routerAvailable = false;
    this.routerSource = 'auto';
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
    this.videoDevices = [];
    this.usbDevicePath = '';

    this._previewCandidates = [];
    this._previewCandidateIndex = 0;
    this._activePreviewCandidate = null;
    this._previewSocket = null;
    this._previewTimeoutId = null;
    this._reconnectTimer = null;
    this._frameObjectUrl = '';
    this._latestRawFrame = null;
    this._frameRenderScheduled = false;
    this._imageDataCache = null;
    this._connected = false;

    this._routerRetryHandle = null;
    this._routerRetryAttempt = 0;
    this._routerRetryLimit = 5;
    this._routerRetryDelayMs = 3000;

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
    void this.refreshRouterState();
    void this.refreshUsbCameraState();
    void this.refreshVideoDevices();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._connected = false;
    this._teardownPreview();
    this._clearRouterRetry();
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
    const routerControlsDisabled = this.routerBusy || !this.routerAvailable;
    const previewLiveAction = { icon: '📡', label: 'Mark feed live' };
    const previewIdleAction = { icon: '🛑', label: 'Mark idle' };
    const routerApplyAction = this.routerBusy && this.routerAvailable
      ? { icon: '⚙️', label: 'Applying…' }
      : { icon: '🧭', label: 'Apply routing' };
    const routerRefreshAction = { icon: '🔄', label: 'Refresh state' };
    const rescanDevicesAction = { icon: '🔍', label: 'Rescan devices' };
    const settingsApplyAction = { icon: '🛠️', label: 'Apply settings' };
    const captureColorAction = { icon: '📸', label: 'Capture color frame' };
    const captureDepthAction = { icon: '🌊', label: 'Capture depth frame' };

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
            <button
              type="button"
              class="surface-button"
              @click=${this.markLive}
              aria-label="${previewLiveAction.label}"
              title="${previewLiveAction.label}"
            >
              <span class="surface-action__icon" aria-hidden="true">${previewLiveAction.icon}</span>
              <span class="surface-action__label" aria-hidden="true">${previewLiveAction.label}</span>
            </button>
            <button
              type="button"
              class="surface-button surface-button--ghost"
              @click=${this.markIdle}
              aria-label="${previewIdleAction.label}"
              title="${previewIdleAction.label}"
            >
              <span class="surface-action__icon" aria-hidden="true">${previewIdleAction.icon}</span>
              <span class="surface-action__label" aria-hidden="true">${previewIdleAction.label}</span>
            </button>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Faces router</h3>
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
                <span class="router-topics__value">${this.routerTopics?.kinectImage || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">Kinect info</span>
                <span class="router-topics__value">${this.routerTopics?.kinectInfo || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">USB image</span>
                <span class="router-topics__value">${this.routerTopics?.usbImage || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">USB info</span>
                <span class="router-topics__value">${this.routerTopics?.usbInfo || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">Faces output image</span>
                <span class="router-topics__value">${this.routerTopics?.outputImage || '—'}</span>
              </div>
              <div class="router-topics__entry">
                <span class="router-topics__label">Faces output info</span>
                <span class="router-topics__value">${this.routerTopics?.outputInfo || '—'}</span>
              </div>
            </div>
            <div class="surface-actions">
              <button
                type="submit"
                class="surface-button"
                ?disabled=${routerControlsDisabled}
                aria-label="${routerApplyAction.label}"
                title="${routerApplyAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true">${routerApplyAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true">${routerApplyAction.label}</span>
              </button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click=${() => this.refreshRouterState({ manual: true })}
                ?disabled=${this.routerBusy}
                aria-label="${routerRefreshAction.label}"
                title="${routerRefreshAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true">${routerRefreshAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true">${routerRefreshAction.label}</span>
              </button>
            </div>
          </form>
          <div>
            <h4 class="surface-card__subtitle">Detected video devices</h4>
            ${this.videoDevices.length
              ? html`<ul class="device-list">
                  ${this.videoDevices.map(
                    (device) => html`<li class="device-entry">
                      <span class="device-entry__label">${device.label || device.path}</span>
                      <span class="device-entry__path">${device.path}</span>
                      <span class="device-entry__flags">
                        ${device.readable ? 'readable' : 'no read access'} ·
                        ${device.writable ? 'writable' : 'read-only'}
                      </span>
                      <div class="device-entry__actions">
                        ${device.active
                          ? html`<span class="device-entry__badge">Active</span>`
                          : html`<button
                              type="button"
                              class="surface-button surface-button--ghost"
                              @click=${() => this.selectVideoDevice(device)}
                              ?disabled=${this.routerBusy || !device.readable}
                              title=${device.readable
                                ? `Switch router to ${device.path}`
                                : 'Device is not readable by the eye module'}
                              aria-label=${device.readable
                                ? `Use camera ${device.path}`
                                : 'Camera not readable'}
                            >
                              <span class="surface-action__icon" aria-hidden="true">🎥</span>
                              <span class="surface-action__label" aria-hidden="true">Use this camera</span>
                            </button>`}
                      </div>
                    </li>`,
                  )}
                </ul>`
              : html`<p class="surface-empty">No video devices detected.</p>`}
            <div class="surface-actions">
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click=${() => this.refreshVideoDevices()}
                aria-label="${rescanDevicesAction.label}"
                title="${rescanDevicesAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true">${rescanDevicesAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true">${rescanDevicesAction.label}</span>
              </button>
            </div>
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
                  <option value="disabled">color only</option>
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
              <button
                type="submit"
                class="surface-button"
                aria-label="${settingsApplyAction.label}"
                title="${settingsApplyAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true">${settingsApplyAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true">${settingsApplyAction.label}</span>
              </button>
            </div>
          </form>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Frame capture</h3>
          ${this.captureFeedback
            ? html`<p class="surface-status" data-variant="error">${this.captureFeedback}</p>`
            : ''}
          <div class="surface-actions">
            <button
              type="button"
              class="surface-button"
              @click=${() => this.handleCapture('color')}
              aria-label="${captureColorAction.label}"
              title="${captureColorAction.label}"
            >
              <span class="surface-action__icon" aria-hidden="true">${captureColorAction.icon}</span>
              <span class="surface-action__label" aria-hidden="true">${captureColorAction.label}</span>
            </button>
            <button
              type="button"
              class="surface-button surface-button--ghost"
              @click=${() => this.handleCapture('depth')}
              aria-label="${captureDepthAction.label}"
              title="${captureDepthAction.label}"
            >
              <span class="surface-action__icon" aria-hidden="true">${captureDepthAction.icon}</span>
              <span class="surface-action__label" aria-hidden="true">${captureDepthAction.label}</span>
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
        note: mode === 'depth' ? 'Depth frame queued for export.' : 'color frame queued for export.',
      },
      ...this.captureHistory,
    ].slice(0, 20);
  }

  async refreshVideoDevices() {
    try {
      const payload = await callModuleAction('eye', 'list_video_devices', {});
      const devicesRaw = Array.isArray(payload?.devices) ? payload.devices : [];
      const devices = devicesRaw
        .map((device) => {
          const path = typeof device?.path === 'string' ? device.path : '';
          if (!path) {
            return null;
          }
          const label = typeof device?.label === 'string' ? device.label : '';
          return {
            id: typeof device?.id === 'string' && device.id ? device.id : path,
            path,
            label,
            readable: Boolean(device?.readable),
            writable: Boolean(device?.writable),
          };
        })
        .filter(Boolean);
      const normalized = this._withActiveDevice(
        /** @type {Array<{ id: string, path: string, label: string, readable: boolean, writable: boolean }>} */ (devices),
        this.usbDevicePath,
      );
      this.videoDevices = normalized;
    } catch (error) {
      console.error('Failed to enumerate video devices', error);
      this.videoDevices = [];
    }
  }

  async refreshUsbCameraState() {
    try {
      const response = await callRosService({
        module: 'eye',
        service: '/psyched_usb_camera/get_parameters',
        type: 'rcl_interfaces/srv/GetParameters',
        args: { names: ['device'] },
        timeoutMs: 3000,
      });
      const values = Array.isArray(response?.values) ? response.values : [];
      const deviceValue = values.length ? values[0] : null;
      this.usbDevicePath = this._extractString(deviceValue, this.usbDevicePath);
    } catch (error) {
      console.error('Failed to query USB camera parameters', error);
    } finally {
      this.videoDevices = this._withActiveDevice(this.videoDevices, this.usbDevicePath);
    }
  }

  _withActiveDevice(devices, activePath) {
    const target = typeof activePath === 'string' ? activePath.trim() : '';
    return devices.map((device) => ({
      ...device,
      active: Boolean(target && device.path === target),
    }));
  }

  async selectVideoDevice(device) {
    const path =
      typeof device === 'string'
        ? device
        : device && typeof device.path === 'string'
        ? device.path
        : '';
    const label =
      device && typeof device === 'object' && typeof device.label === 'string' && device.label
        ? device.label
        : '';
    const targetPath = path.trim();
    if (!targetPath || this.routerBusy) {
      return;
    }

    const statusLabel = label || targetPath;
    this.routerBusy = true;
    this.routerStatus = `Switching camera to ${statusLabel}…`;
    this.routerTone = 'info';

    try {
      await this._setUsbCameraDevice(targetPath);
      this.usbDevicePath = targetPath;
      this.videoDevices = this._withActiveDevice(this.videoDevices, targetPath);

      try {
        await this._setFacesRouterSourceUsb();
      } catch (routerError) {
        const message = routerError instanceof Error ? routerError.message : String(routerError);
        this.routerStatus = `Camera switched, but router update failed: ${message}`;
        this.routerTone = 'error';
        return;
      }

      await this.refreshRouterState({ manual: true });
      this.routerStatus = `Faces router using USB feed (${statusLabel}).`;
      this.routerTone = 'success';
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.routerStatus = `Failed to switch camera: ${message}`;
      this.routerTone = 'error';
    } finally {
      this.routerBusy = false;
    }
  }

  async _setUsbCameraDevice(path) {
    const parameters = [this._buildStringParameter('device', path)].filter(Boolean);
    if (!parameters.length) {
      throw new Error('Missing camera device path.');
    }
    const response = await callRosService({
      module: 'eye',
      service: '/psyched_usb_camera/set_parameters',
      type: 'rcl_interfaces/srv/SetParameters',
      args: { parameters },
      timeoutMs: 4000,
    });

    const results = Array.isArray(response?.results) ? response.results : [];
    for (const result of results) {
      if (result && result.successful === false) {
        const reason =
          typeof result.reason === 'string' && result.reason ? result.reason : 'device update rejected';
        throw new Error(reason);
      }
    }
  }

  async _setFacesRouterSourceUsb() {
    const parameters = [
      this._buildBoolParameter('usb_enabled', true),
      this._buildStringParameter('faces_source', 'usb'),
    ].filter(Boolean);
    if (!parameters.length) {
      return;
    }

    const response = await callRosService({
      module: 'eye',
      service: '/psyched_faces_router/set_parameters',
      type: 'rcl_interfaces/srv/SetParameters',
      args: { parameters },
      timeoutMs: 5000,
    });

    const results = Array.isArray(response?.results) ? response.results : [];
    for (const result of results) {
      if (result && result.successful === false) {
        const reason =
          typeof result.reason === 'string' && result.reason ? result.reason : 'faces router rejected update';
        throw new Error(reason);
      }
    }
  }

  async refreshRouterState(options = {}) {
    const manual = Boolean(options.manual);
    if (manual) {
      this._routerRetryAttempt = 0;
    }
    this._clearRouterRetry();
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
        kinectImage: this._extractString(lookup.get('kinect_image_topic'), this.routerTopics?.kinectImage),
        kinectInfo: this._extractString(lookup.get('kinect_camera_info_topic'), this.routerTopics?.kinectInfo),
        usbImage: this._extractString(lookup.get('usb_image_topic'), this.routerTopics?.usbImage),
        usbInfo: this._extractString(lookup.get('usb_camera_info_topic'), this.routerTopics?.usbInfo),
        outputImage: this._extractString(lookup.get('output_image_topic'), this.routerTopics?.outputImage),
        outputInfo: this._extractString(lookup.get('output_camera_info_topic'), this.routerTopics?.outputInfo),
      };

      this.routerStatus = 'Faces router ready.';
      this.routerTone = 'success';
      this.routerAvailable = true;
      this._routerRetryAttempt = 0;
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      const unavailable = typeof message === 'string' && message.toLowerCase().includes('not available');
      if (unavailable && this._routerRetryAttempt < this._routerRetryLimit) {
        this._routerRetryAttempt += 1;
        const delaySeconds = Math.max(this._routerRetryDelayMs / 1000, 0.1);
        this.routerStatus = `Faces router starting (waiting for eye); retrying in ${delaySeconds.toFixed(1)}s…`;
        this.routerTone = 'warning';
        this.routerAvailable = false;
        this._routerRetryHandle = setTimeout(() => {
          this._routerRetryHandle = null;
          void this.refreshRouterState();
        }, this._routerRetryDelayMs);
      } else {
        this.routerStatus = `Faces router unavailable: ${message}`;
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

  markLive() {
    this.statusMessage = 'Live stream detected.';
    this.statusTone = 'success';
  }

  markIdle() {
    this.statusMessage = 'Awaiting stream status updates…';
    this.statusTone = 'info';
  }

  _clearRouterRetry() {
    if (this._routerRetryHandle) {
      clearTimeout(this._routerRetryHandle);
      this._routerRetryHandle = null;
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
    this._frameRenderScheduled = false;
    this._imageDataCache = null;
    this._activePreviewCandidate = null;
    this._clearPreviewSocket();
    this._revokeObjectUrl();
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
    this._revokeObjectUrl();
    this._frameObjectUrl = URL.createObjectURL(blob);
    this.previewFrameUrl = this._frameObjectUrl;
    this.previewMode = 'image';
    this.previewEncoding = `compressed (${mimeType.replace('image/', '')})`;
    if (typeof message.width === 'number' && message.width > 0) {
      this.previewWidth = message.width;
    }
    if (typeof message.height === 'number' && message.height > 0) {
      this.previewHeight = message.height;
    }
    return true;
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

  _revokeObjectUrl() {
    if (this._frameObjectUrl) {
      URL.revokeObjectURL(this._frameObjectUrl);
      this._frameObjectUrl = '';
    }
  }
}

customElements.define('eye-dashboard', EyeDashboard);
