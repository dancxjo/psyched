import {
  css,
  html,
  LitElement,
} from "https://unpkg.com/lit@3.1.4/index.js?module";
import { createTopicSocket } from "/js/cockpit.js";
import { surfaceStyles } from "/components/cockpit-style.js";
import {
  buildFacesSettingsPayload,
  parseFaceTriggerPayload,
} from "./faces-dashboard.helpers.js";

function makeId(prefix) {
  return crypto.randomUUID
    ? crypto.randomUUID()
    : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

const MAX_CROP_HISTORY = 12;

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
    cropStatusMessage: { state: true },
    cropStatusTone: { state: true },
    faceCrops: { state: true },
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

      .crop-gallery {
        display: grid;
        gap: 0.75rem;
        grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      }

      .crop-card {
        display: grid;
        gap: 0.5rem;
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
      }

      .crop-card__image {
        width: 100%;
        border-radius: 0.4rem;
        display: block;
        object-fit: cover;
        background: rgba(0, 0, 0, 0.25);
      }

      .crop-card__meta {
        display: flex;
        flex-direction: column;
        gap: 0.25rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .crop-card__meta span {
        color: var(--lcars-text);
        font-family: var(--metric-value-font);
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
    this.statusMessage = "Connecting to recognition stream…";
    this.statusTone = "info";
    this.cropStatusMessage = "Connecting to detections stream…";
    this.cropStatusTone = "info";
    this.faceCrops = [];
    this.tagFaceId = "";
    this.tagLabel = "";
    this.tagFeedback = "";
    this.detectionLog = [];
    this.sockets = [];
    this._cropCanvas = null;
    this._cropContext = null;
  }

  connectedCallback() {
    super.connectedCallback();
    if (!this.sockets.length) {
      this.connectTriggerStream();
      this.connectDetectionsStream();
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
    const hasCrops = this.faceCrops.length > 0;
    const tuningApplyAction = { icon: "🎛️", label: "Apply tuning" };
    const resetEmbeddingsAction = {
      icon: "♻️",
      label: "Reset embedding cache",
    };
    const simulateRecognitionAction = {
      icon: "🎭",
      label: "Simulate recognition",
    };
    const submitLabelAction = { icon: "🏷️", label: "Submit label" };
    const clearLabelAction = { icon: "🧹", label: "Clear" };
    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card">
          <h3 class="surface-card__title">Detector tuning</h3>
          <p class="surface-status" data-variant="${this.statusTone}">${this
            .statusMessage}</p>
          <form @submit="${this.handleSettingsSubmit}">
            <div class="form-row form-row--split">
              <label>
                Detection threshold
                <input
                  type="number"
                  step="0.05"
                  min="0.1"
                  max="1"
                  .value="${String(this.threshold)}"
                  @input="${(
                    event,
                  ) => (this.threshold = Number(event.target.value))}"
                />
              </label>
              <label>
                Smoothing window (frames)
                <input
                  type="number"
                  min="1"
                  max="120"
                  .value="${String(this.window)}"
                  @input="${(
                    event,
                  ) => (this.window = Number(event.target.value))}"
                />
              </label>
            </div>
            <div class="form-row form-row--split">
              <label>
                Publish crops
                <select
                  name="publishCrops"
                  .value="${this.publishCrops ? "true" : "false"}"
                  @change="${(
                    event,
                  ) => (this.publishCrops = event.target.value === "true")}"
                >
                  <option value="true">Enabled</option>
                  <option value="false">Disabled</option>
                </select>
              </label>
              <label>
                Publish embeddings
                <select
                  name="publishEmbeddings"
                  .value="${this.publishEmbeddings ? "true" : "false"}"
                  @change="${(
                    event,
                  ) => (this.publishEmbeddings =
                    event.target.value === "true")}"
                >
                  <option value="true">Enabled</option>
                  <option value="false">Disabled</option>
                </select>
              </label>
            </div>
            <div class="surface-actions">
              <button
                type="submit"
                class="surface-button"
                aria-label="${tuningApplyAction.label}"
                title="${tuningApplyAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true"
                >${tuningApplyAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true"
                >${tuningApplyAction.label}</span>
              </button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click="${this.resetEmbeddings}"
                aria-label="${resetEmbeddingsAction.label}"
                title="${resetEmbeddingsAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true"
                >${resetEmbeddingsAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true"
                >${resetEmbeddingsAction.label}</span>
              </button>
            </div>
          </form>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Recent face crops</h3>
          <p class="surface-status" data-variant="${this.cropStatusTone}">${this
            .cropStatusMessage}</p>
          ${hasCrops
            ? html`
              <div class="crop-gallery">
                ${this.faceCrops.map(
                  (crop) =>
                    html`
                      <div class="crop-card" data-crop-id="${crop.id}">
                        <img class="crop-card__image" src="${crop
                          .url}" alt="Face crop preview" />
                        <div class="crop-card__meta">
                          <div>
                            Captured <span>${crop.timestamp}</span>
                          </div>
                          <div>
                            Confidence
                            <span>${typeof crop.confidence === "number"
                              ? `${(crop.confidence * 100).toFixed(1)}%`
                              : "—"}</span>
                          </div>
                          <div>
                            Size <span>${crop.width}×${crop.height}</span>
                          </div>
                        </div>
                      </div>
                    `,
                )}
              </div>
            `
            : html`
              <p class="surface-empty">No face crops received yet.</p>
            `}
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Recognition events</h3>
          ${this.detectionLog.length
            ? html`
              <ol class="detection-log">
                ${this.detectionLog.map(
                  (entry) =>
                    html`
                      <li class="detection-entry">
                        <div class="detection-entry__meta">
                          <span>${entry.timestamp}</span>
                          <span>${entry.name}</span>
                          ${entry.collection
                            ? html`
                              <span>${entry.collection}</span>
                            `
                            : ""}
                        </div>
                        ${entry.identity
                          ? html`
                            <p class="detection-entry__note">
                              Identity:
                              <span class="detection-entry__value">${entry.identity.name ||
                                entry.identity.id || "—"}</span>
                              ${typeof entry.identityConfidence === "number"
                                ? html`
                                  <span class="detection-entry__note--muted">
                                    ${(entry.identityConfidence * 100).toFixed(1)}%
                                  </span>
                                `
                                : ""}
                            </p>
                          `
                          : ""}
                        <p class="detection-entry__note">
                          Memory:
                          <span class="detection-entry__value">${entry
                            .memoryId || "—"}</span>
                        </p>
                        <p class="detection-entry__note">
                          Vector:
                          <span class="detection-entry__value">${entry
                            .vectorId || "—"}</span>
                        </p>
                        ${entry.vectorPreview?.length
                          ? html`
                            <p class="detection-entry__note detection-entry__note--muted">
                              Vector preview:
                              <code>${entry.vectorPreview
                                .map((value) => Number(value).toFixed(3))
                                .join(", ")}</code>
                            </p>
                          `
                          : ""} ${entry.matches?.length
                          ? html`
                            <p class="detection-entry__note detection-entry__note--muted">
                              Matches:
                              ${this.formatMatches(entry.matches)}
                            </p>
                          `
                          : ""} ${entry.cropTopic
                          ? html`
                            <p class="detection-entry__note">
                              Crops topic:
                              <span class="detection-entry__value">${entry
                                .cropTopic}</span>
                            </p>
                          `
                          : ""} ${entry.note
                          ? html`
                            <p class="detection-entry__note detection-entry__note--muted">${entry
                              .note}</p>
                          `
                          : ""} ${entry.raw
                          ? html`
                            <p class="detection-entry__raw"><code>${entry
                              .raw}</code></p>
                          `
                          : ""}
                      </li>
                    `,
                )}
              </ol>
            `
            : html`
              <p class="surface-empty">No recognition events recorded yet.</p>
            `}
          <div class="surface-actions">
            <button
              type="button"
              class="surface-button surface-button--ghost"
              @click="${this.simulateDetection}"
              aria-label="${simulateRecognitionAction.label}"
              title="${simulateRecognitionAction.label}"
            >
              <span class="surface-action__icon" aria-hidden="true">${simulateRecognitionAction
                .icon}</span>
              <span class="surface-action__label" aria-hidden="true"
              >${simulateRecognitionAction.label}</span>
            </button>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Manual labelling</h3>
          ${this.tagFeedback
            ? html`
              <p class="surface-status" data-variant="error">${this
                .tagFeedback}</p>
            `
            : ""}
          <form @submit="${this.handleTagSubmit}">
            <label>
              Face identifier
              <input
                type="text"
                name="faceId"
                .value="${this.tagFaceId}"
                @input="${(event) => (this.tagFaceId = event.target.value)}"
              />
            </label>
            <label>
              Label
              <input
                type="text"
                name="label"
                .value="${this.tagLabel}"
                @input="${(event) => (this.tagLabel = event.target.value)}"
              />
            </label>
            <div class="surface-actions">
              <button
                type="submit"
                class="surface-button"
                aria-label="${submitLabelAction.label}"
                title="${submitLabelAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true"
                >${submitLabelAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true"
                >${submitLabelAction.label}</span>
              </button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click="${this.clearTagForm}"
                aria-label="${clearLabelAction.label}"
                title="${clearLabelAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true"
                >${clearLabelAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true"
                >${clearLabelAction.label}</span>
              </button>
            </div>
          </form>
        </article>
      </div>
    `;
  }

  connectDetectionsStream() {
    const socket = createTopicSocket({
      module: "faces",
      action: "face_detections_stream",
      type: "faces_msgs/msg/FaceDetections",
      role: "subscribe",
      queueLength: 2,
    });
    socket.addEventListener("open", () => {
      this.cropStatusMessage = "Detections stream connected.";
      this.cropStatusTone = "success";
    });
    socket.addEventListener("message", (event) => {
      const payload = this.parseStreamEnvelope(event);
      if (!payload) {
        return;
      }
      if (payload.event === "status") {
        const state = payload?.data?.state;
        if (state === "ready") {
          this.cropStatusMessage = "Detections stream ready.";
          this.cropStatusTone = "success";
        } else if (state === "closed") {
          this.cropStatusMessage = "Detections stream closed.";
          this.cropStatusTone = "warning";
        }
        return;
      }
      if (payload.event !== "message") {
        return;
      }
      this._processDetectionsPayload(payload.data);
    });
    socket.addEventListener("error", () => {
      this.cropStatusMessage = "Detections stream error.";
      this.cropStatusTone = "error";
    });
    socket.addEventListener("close", () => {
      this.cropStatusMessage = "Detections stream disconnected.";
      this.cropStatusTone = "warning";
    });
    this.sockets.push(socket);
  }

  _processDetectionsPayload(message) {
    if (!message || typeof message !== "object") {
      return;
    }
    const faces = Array.isArray(message.faces) ? message.faces : [];
    if (!faces.length) {
      return;
    }
    const timestamp = new Date().toLocaleTimeString();
    const previews = [];
    for (const face of faces) {
      const preview = this._renderFaceCrop(face);
      if (preview) {
        previews.push({
          id: makeId("crop"),
          timestamp,
          ...preview,
        });
      }
    }
    if (!previews.length) {
      return;
    }
    this.faceCrops = [...previews, ...this.faceCrops].slice(
      0,
      MAX_CROP_HISTORY,
    );
  }

  _renderFaceCrop(face) {
    if (!face || typeof face !== "object") {
      return null;
    }
    const preview = this._imageMessageToDataUrl(face.crop);
    if (!preview) {
      return null;
    }
    const confidenceValue = Number(face.confidence);
    return {
      url: preview.url,
      width: preview.width,
      height: preview.height,
      confidence: Number.isFinite(confidenceValue) ? confidenceValue : null,
    };
  }

  _imageMessageToDataUrl(image) {
    if (!image || typeof image !== "object") {
      return null;
    }
    const width = Number(image.width) || 0;
    const height = Number(image.height) || 0;
    const encoding = typeof image.encoding === "string"
      ? image.encoding.toLowerCase()
      : "";
    const data = this._toUint8Array(image.data);
    if (!width || !height || !encoding || !data.length) {
      return null;
    }

    const context = this._ensureCropContext(width, height);
    if (!context || !this._cropCanvas) {
      return null;
    }

    let rgba;
    if (encoding === "rgb8" || encoding === "bgr8") {
      if (data.length < width * height * 3) {
        return null;
      }
      rgba = new Uint8ClampedArray(width * height * 4);
      for (
        let source = 0, target = 0;
        source < width * height * 3;
        source += 3, target += 4
      ) {
        const r = encoding === "bgr8" ? data[source + 2] : data[source];
        const g = data[source + 1];
        const b = encoding === "bgr8" ? data[source] : data[source + 2];
        rgba[target] = r;
        rgba[target + 1] = g;
        rgba[target + 2] = b;
        rgba[target + 3] = 255;
      }
    } else if (encoding === "rgba8" || encoding === "bgra8") {
      if (data.length < width * height * 4) {
        return null;
      }
      rgba = new Uint8ClampedArray(width * height * 4);
      for (let source = 0; source < width * height * 4; source += 4) {
        const r = encoding === "bgra8" ? data[source + 2] : data[source];
        const g = data[source + 1];
        const b = encoding === "bgra8" ? data[source] : data[source + 2];
        const a = data[source + 3];
        rgba[source] = r;
        rgba[source + 1] = g;
        rgba[source + 2] = b;
        rgba[source + 3] = a > 0 ? a : 255;
      }
    } else if (encoding.startsWith("mono")) {
      if (data.length < width * height) {
        return null;
      }
      rgba = new Uint8ClampedArray(width * height * 4);
      for (
        let index = 0, target = 0;
        index < width * height;
        index += 1, target += 4
      ) {
        const value = data[index];
        rgba[target] = value;
        rgba[target + 1] = value;
        rgba[target + 2] = value;
        rgba[target + 3] = 255;
      }
    } else {
      return null;
    }

    const imageData = new ImageData(rgba, width, height);
    context.putImageData(imageData, 0, 0);
    try {
      return {
        url: this._cropCanvas.toDataURL("image/png"),
        width,
        height,
      };
    } catch (_error) {
      return null;
    }
  }

  _ensureCropContext(width, height) {
    if (!this._cropCanvas) {
      this._cropCanvas = typeof document !== "undefined"
        ? document.createElement("canvas")
        : null;
      this._cropContext = this._cropCanvas
        ? this._cropCanvas.getContext("2d", { willReadFrequently: true })
        : null;
    }
    if (!this._cropCanvas || !this._cropContext) {
      return null;
    }
    if (
      this._cropCanvas.width !== width || this._cropCanvas.height !== height
    ) {
      this._cropCanvas.width = width;
      this._cropCanvas.height = height;
    }
    return this._cropContext;
  }

  connectTriggerStream() {
    const socket = createTopicSocket({
      module: "faces",
      action: "face_trigger_stream",
      type: "psyched_msgs/msg/SensationStamped",
      role: "subscribe",
    });
    socket.addEventListener("open", () => {
      this.statusMessage = "Recognition stream connected.";
      this.statusTone = "success";
    });
    socket.addEventListener("message", (event) => {
      const payload = this.parseStreamEnvelope(event);
      if (!payload) {
        return;
      }
      if (payload.event === "status") {
        const state = payload?.data?.state;
        if (state === "ready") {
          this.statusMessage = "Recognition stream ready.";
          this.statusTone = "success";
        } else if (state === "closed") {
          this.statusMessage = "Recognition stream closed.";
          this.statusTone = "warning";
        }
        return;
      }
      if (payload.event !== "message") {
        return;
      }
      const result = parseFaceTriggerPayload(payload.data);
      if (!result.ok) {
        if (result.reason === "ignored" || result.reason === "empty") {
          return;
        }
        this.statusMessage = result.error;
        this.statusTone = "error";
        this.recordRecognitionEvent({
          name: "Unparsed trigger",
          note: result.error,
          raw: this.formatRawPayload(payload.data),
          collection: "",
          memoryId: "",
          vectorId: "",
        });
        return;
      }
      const eventData = result.value;
      this.statusMessage = `Recognition event: ${eventData.name}`;
      this.statusTone = "success";
      this.recordRecognitionEvent(eventData);
    });
    socket.addEventListener("error", () => {
      this.statusMessage = "Recognition stream error.";
      this.statusTone = "error";
    });
    socket.addEventListener("close", () => {
      this.statusMessage = "Recognition stream disconnected.";
      this.statusTone = "warning";
    });
    this.sockets.push(socket);
  }

  parseStreamEnvelope(event) {
    if (!event || typeof event.data !== "string") {
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
      id: makeId("face"),
      timestamp: new Date().toLocaleTimeString(),
      name: details?.name ?? "Unknown",
      memoryId: details?.memoryId ?? "",
      vectorId: details?.vectorId ?? "",
      memoryHint: details?.memoryHint ?? "",
      vectorHint: details?.vectorHint ?? "",
      signature: details?.signature ?? "",
      vectorPreview: Array.isArray(details?.vectorPreview)
        ? details.vectorPreview
        : [],
      collection: details?.collection ?? "",
      cropTopic: details?.cropTopic ?? "",
      note: details?.note ?? "",
      raw: details?.raw ?? "",
      identity: details?.identity ?? null,
      identityConfidence: typeof details?.identityConfidence === "number"
        ? details.identityConfidence
        : (details?.identity && typeof details.identity.confidence === "number"
          ? details.identity.confidence
          : null),
      matches: Array.isArray(details?.matches) ? details.matches : [],
    };
    this.detectionLog = [entry, ...this.detectionLog].slice(0, 40);
  }

  formatMatches(matches) {
    if (!Array.isArray(matches) || !matches.length) {
      return "";
    }
    return matches
      .map((match) => {
        const identity = match && typeof match === "object"
          ? match.identity
          : null;
        const label = identity && (identity.name || identity.id)
          ? identity.name || identity.id
          : (() => {
            const memoryId = match && typeof match === "object" ? match.memoryId : "";
            return typeof memoryId === "string" && memoryId.trim() ? memoryId.trim() : "";
          })();
        const score = match && typeof match === "object" && typeof match.score === "number"
          ? match.score.toFixed(2)
          : "";
        if (label && score) {
          return `${label} (${score})`;
        }
        return label || score || "match";
      })
      .join(" · ");
  }

  formatRawPayload(data) {
    if (typeof data === "string") {
      return data;
    }
    try {
      return JSON.stringify(data);
    } catch (_error) {
      return "";
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
      this.statusTone = "error";
      return;
    }
    this.dispatchEvent(
      new CustomEvent("faces-settings-request", {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = "Detector settings submitted for synchronisation.";
    this.statusTone = "success";
  }

  handleTagSubmit(event) {
    event.preventDefault();
    const faceId = this.tagFaceId.trim();
    const label = this.tagLabel.trim();
    if (!faceId || !label) {
      this.tagFeedback = "Face identifier and label are required.";
      return;
    }
    this.dispatchEvent(
      new CustomEvent("faces-tag-request", {
        detail: { faceId, label },
        bubbles: true,
        composed: true,
      }),
    );
    this.tagFeedback = "";
    this.clearTagForm();
  }

  clearTagForm() {
    this.tagFaceId = "";
    this.tagLabel = "";
  }

  resetEmbeddings() {
    this.dispatchEvent(
      new CustomEvent("faces-database-reset", {
        detail: { scope: "embeddings" },
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = "Embedding cache reset request queued.";
    this.statusTone = "warning";
  }

  _toUint8Array(data) {
    if (data instanceof Uint8Array) {
      return data;
    }
    if (Array.isArray(data)) {
      return Uint8Array.from(data);
    }
    if (data && typeof data === "object" && typeof data.length === "number") {
      try {
        return Uint8Array.from(data);
      } catch (_error) {
        return new Uint8Array();
      }
    }
    return new Uint8Array();
  }

  simulateDetection() {
    const memoryId = `mem-${Math.random().toString(16).slice(2, 10)}`;
    const vectorId = `vec-${Math.random().toString(16).slice(2, 10)}`;
    const payload = {
      name: "Simulated face",
      memoryId,
      vectorId,
      collection: "faces",
      note: "Simulated recognition event.",
      raw: JSON.stringify({
        name: "Simulated face",
        memory_id: memoryId,
        vector_id: vectorId,
        collection: "faces",
      }),
    };
    this.recordRecognitionEvent(payload);
    this.statusMessage = "Simulated recognition event recorded.";
    this.statusTone = "info";
  }
}

customElements.define("faces-dashboard", FacesDashboard);
