import {
  css,
  html,
  LitElement,
} from "https://unpkg.com/lit@3.1.4/index.js?module";
import { classMap } from "https://unpkg.com/lit@3.1.4/directives/class-map.js?module";
import { surfaceStyles } from "/components/cockpit-style.js";
import { createCollapsedCardManager } from "/components/dashboard-collapse.js";
import { createTopicSocket } from "/js/cockpit.js";
import {
  blobToBase64,
  formatByteSize,
  normaliseDebugSnapshot,
  normaliseFeelingIntent,
} from "./pilot-dashboard.helpers.js";

function makeId(prefix) {
  return crypto.randomUUID
    ? crypto.randomUUID()
    : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

/**
 * Dashboard for monitoring the pilot planner loop.
 *
 * Displays:
 * - Live FeelingIntent messages coming from the pilot node.
 * - Debug snapshots summarising context topics, sensations and script runs.
 * - The most recent LLM response cached by the pilot.
 */
class PilotDashboard extends LitElement {
  static properties = {
    moduleStatus: { state: true },
    connected: { state: true },
    intentConnected: { state: true },
    lastHeartbeat: { state: true },
    config: { state: true },
    recentSensations: { state: true },
    scriptRuns: { state: true },
    lastPrompt: { state: true },
    lastLLM: { state: true },
    logsCount: { state: true },
    errorsCount: { state: true },
    intentFeed: { state: true },
    collapsedCards: { state: true },
    earTextInput: { state: true },
    earFeedback: { state: true },
    earFeedbackTone: { state: true },
    eyeRecordingState: { state: true },
    eyeStatusMessage: { state: true },
    eyeFeedback: { state: true },
    eyeFeedbackTone: { state: true },
    eyeClipSummary: { state: true },
    eyeDurationSeconds: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      .status-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
        gap: 0.75rem;
        margin: 1rem 0;
      }

      .status-grid dt {
        margin: 0;
        font-size: 0.7rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--metric-label-color);
      }

      .status-grid dd {
        margin: 0.25rem 0 0 0;
        font-size: 1.05rem;
        font-weight: 500;
      }

      .section-note {
        margin: 0;
        color: var(--lcars-muted);
        font-size: 0.85rem;
      }

      .harness-grid {
        display: grid;
        gap: 1rem;
        grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      }

      .harness-panel {
        border: 1px solid var(--control-surface-border);
        border-radius: 0.75rem;
        padding: 0.85rem;
        display: grid;
        gap: 0.65rem;
        background: rgba(255, 255, 255, 0.02);
      }

      .harness-panel textarea,
      .harness-panel input[type="number"] {
        width: 100%;
        font: inherit;
        padding: 0.65rem 0.75rem;
        border-radius: 0.5rem;
        border: 1px solid var(--control-surface-border);
        background: rgba(0, 0, 0, 0.4);
        color: var(--lcars-text);
        resize: vertical;
        min-height: 3.2rem;
      }

      .harness-meta {
        margin: 0;
        font-size: 0.8rem;
        color: var(--lcars-muted);
      }

      .chip-list {
        list-style: none;
        padding: 0;
        margin: 0.6rem 0 0 0;
        display: flex;
        flex-wrap: wrap;
        gap: 0.35rem;
      }

      .chip-list .chip {
        background: rgba(255, 255, 255, 0.08);
        border-radius: 999px;
        padding: 0.15rem 0.6rem;
        font-size: 0.75rem;
        letter-spacing: 0.04em;
      }

      .empty-placeholder {
        margin: 0;
        color: var(--lcars-muted);
        font-style: italic;
      }

      .intent-feed {
        list-style: none;
        padding: 0;
        margin: 1rem 0 0 0;
        display: flex;
        flex-direction: column;
        gap: 0.75rem;
      }

      .intent-entry {
        border: 1px solid var(--control-surface-border);
        border-radius: 0.6rem;
        background: rgba(0, 0, 0, 0.35);
        padding: 0.9rem;
        display: grid;
        gap: 0.5rem;
      }

      .intent-entry__meta {
        display: flex;
        flex-wrap: wrap;
        gap: 0.6rem;
        align-items: center;
        font-size: 0.85rem;
        color: var(--lcars-muted);
      }

      .intent-entry__emoji {
        font-size: 1.4rem;
      }

      .intent-entry__script,
      .llm-output {
        margin: 0;
        padding: 0.75rem;
        background: rgba(0, 0, 0, 0.45);
        border-radius: 0.5rem;
        border: 1px solid var(--control-surface-border);
        font-family: var(--font-mono, "Fira Code", monospace);
        font-size: 0.82rem;
        white-space: pre-wrap;
      }

      .intent-entry__goals,
      .intent-entry__topics {
        margin: 0;
        font-size: 0.85rem;
        color: var(--lcars-muted);
      }

      .sensations-list,
      .scripts-list {
        list-style: none;
        padding: 0;
        margin: 1rem 0 0 0;
        display: flex;
        flex-direction: column;
        gap: 0.6rem;
      }

      .sensations-entry,
      .script-entry {
        border: 1px solid var(--control-surface-border);
        border-radius: 0.6rem;
        padding: 0.75rem;
        background: rgba(0, 0, 0, 0.3);
      }

      .sensations-entry__meta,
      .script-entry__meta {
        margin: 0 0 0.35rem 0;
        font-size: 0.85rem;
        color: var(--lcars-muted);
      }

      .script-entry__status {
        font-weight: 600;
        text-transform: capitalize;
      }

      .script-entry__actions {
        margin: 0.35rem 0 0 0;
        padding-left: 1.2rem;
        font-size: 0.85rem;
      }

      .llm-output {
        margin-top: 1rem;
      }

      .pilot-dashboard__intent-prompt {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
        gap: 0.85rem;
        grid-column: 1 / -1;
        align-items: start;
      }

      @media (max-width: 720px) {
        .pilot-dashboard__intent-prompt {
          grid-template-columns: minmax(0, 1fr);
        }
      }
    `,
  ];

  constructor() {
    super();
    this.moduleStatus = "initialising";
    this.connected = false;
    this.intentConnected = false;
    this.lastHeartbeat = null;
    this.config = {
      debounceSeconds: null,
      windowSeconds: null,
      contextTopics: [],
      sensationTopics: [],
    };
    this.recentSensations = [];
    this.scriptRuns = [];
    this.lastPrompt = "";
    this.lastLLM = "";
    this.logsCount = 0;
    this.errorsCount = 0;
    this.intentFeed = [];
    this.earTextInput = "";
    this.earFeedback = "";
    this.earFeedbackTone = "";
    this.eyeRecordingState = "idle";
    this.eyeStatusMessage = "Awaiting recording request.";
    this.eyeFeedback = "";
    this.eyeFeedbackTone = "";
    this.eyeClipSummary = "";
    this.eyeDurationSeconds = 5;
    this._cardManager = createCollapsedCardManager("module-pilot");
    this.collapsedCards = this._cardManager.getCollapsedIds();

    this._debugSocket = null;
    this._intentSocket = null;
    this._earPublisher = null;
    this._eyePublisher = null;
    this._eyeMediaStream = null;
    this._eyeRecorder = null;
    this._eyeChunks = [];
    this._eyeStopTimer = null;
    this._earFeedbackTimer = null;
    this._eyeFeedbackTimer = null;
  }

  connectedCallback() {
    super.connectedCallback();
    this._connectDebugStream();
    this._connectIntentStream();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._shutdownSocket(this._debugSocket);
    this._shutdownSocket(this._intentSocket);
    this._shutdownSocket(this._earPublisher);
    this._shutdownSocket(this._eyePublisher);
    this._teardownEyeRecorder();
    if (this._earFeedbackTimer) {
      clearTimeout(this._earFeedbackTimer);
      this._earFeedbackTimer = null;
    }
    if (this._eyeFeedbackTimer) {
      clearTimeout(this._eyeFeedbackTimer);
      this._eyeFeedbackTimer = null;
    }
    this._debugSocket = null;
    this._intentSocket = null;
    this._earPublisher = null;
    this._eyePublisher = null;
  }

  isCardCollapsed(id) {
    if (!id) {
      return false;
    }
    if (this._cardManager) {
      return this._cardManager.isCollapsed(id);
    }
    return this.collapsedCards instanceof Set ? this.collapsedCards.has(id) : false;
  }

  setCardCollapsed(id, collapsed) {
    if (!id) {
      return;
    }
    if (this._cardManager) {
      this.collapsedCards = this._cardManager.setCollapsed(id, collapsed);
      return;
    }
    const next = new Set(this.collapsedCards instanceof Set ? this.collapsedCards : []);
    if (collapsed) {
      next.add(id);
    } else {
      next.delete(id);
    }
    this.collapsedCards = next;
  }

  toggleCardCollapsed(id) {
    if (!id) {
      return;
    }
    if (this._cardManager) {
      this.collapsedCards = this._cardManager.toggle(id);
      return;
    }
    const collapsed = this.isCardCollapsed(id);
    this.setCardCollapsed(id, !collapsed);
  }

  render() {
    const heartbeatDisplay = this.lastHeartbeat
      ? this._formatTimestamp(this.lastHeartbeat)
      : "No heartbeat yet";
    const debounceDisplay = this.config.debounceSeconds != null
      ? `${this.config.debounceSeconds.toFixed(1)} s`
      : "—";
    const windowDisplay = this.config.windowSeconds != null
      ? `${this.config.windowSeconds.toFixed(1)} s`
      : "—";
    const statusTone = this.connected ? "success" : "warning";
    const statusMessage = this.connected
      ? "Receiving pilot debug telemetry."
      : "Waiting for pilot debug stream…";
    const intentTone = this.intentConnected ? "success" : "warning";
    const intentMessage = this.intentConnected
      ? "FeelingIntent stream active."
      : "No FeelingIntent messages yet.";

    const statusCard = this.renderSurfaceCard({
      id: "pilot-status",
      title: "Pilot status",
      wide: true,
      content: html`
        <p class="surface-status" data-variant="${statusTone}">${statusMessage}</p>
        <dl class="status-grid">
          <div>
            <dt>Planner state</dt>
            <dd>${this.moduleStatus}</dd>
          </div>
          <div>
            <dt>Heartbeat</dt>
            <dd>${heartbeatDisplay}</dd>
          </div>
          <div>
            <dt>Debounce window</dt>
            <dd>${debounceDisplay}</dd>
          </div>
          <div>
            <dt>Context horizon</dt>
            <dd>${windowDisplay}</dd>
          </div>
          <div>
            <dt>Debug logs cached</dt>
            <dd>${this.logsCount}</dd>
          </div>
          <div>
            <dt>Error entries</dt>
            <dd>${this.errorsCount}</dd>
          </div>
        </dl>
        <section>
          <h4>Context topics</h4>
          <p class="section-note">
            Topics the pilot is currently including in the prompt context.
          </p>
          ${this.config.contextTopics.length
          ? html`
              <ul class="chip-list">
                ${this.config.contextTopics.map((topic) =>
            html`
                    <li class="chip">${topic}</li>
                  `
          )}
              </ul>
            `
          : html`
              <p class="empty-placeholder">No context topics observed yet.</p>
            `}
        </section>
        <section>
          <h4>Sensation inputs</h4>
          <p class="section-note">
            Recent sensation topics contributing to short-term memory.
          </p>
          ${this.config.sensationTopics.length
          ? html`
              <ul class="chip-list">
                ${this.config.sensationTopics.map((topic) =>
            html`
                    <li class="chip">${topic}</li>
                  `
          )}
              </ul>
            `
          : html`
              <p class="empty-placeholder">No sensation streams recorded yet.</p>
            `}
        </section>
      `,
    });

    const intentCard = this.renderSurfaceCard({
      id: "pilot-intent",
      title: "FeelingIntent feed",
      wide: true,
      content: html`
        <p class="surface-status" data-variant="${intentTone}">${intentMessage}</p>
        ${this.intentFeed.length
          ? html`
            <ul class="intent-feed">
              ${this.intentFeed.map(
            (entry) =>
              html`
                    <li class="intent-entry">
                      <div class="intent-entry__meta">
                        <span class="intent-entry__emoji">${entry
                  .attitudeEmoji || "🤖"}</span>
                        <span>${entry.displayTime}</span>
                        ${entry.episodeId
                  ? html`
                            <span>Episode ${entry.episodeId}</span>
                          `
                  : null} ${entry.situationId
                    ? html`
                            <span>Situation ${entry.situationId}</span>
                          `
                    : null}
                      </div>
                      ${entry.spokenSentence
                  ? html`
                          <p class="surface-note">"${entry
                      .spokenSentence}"</p>
                        `
                  : null} ${entry.thoughtSentence
                    ? html`
                          <p class="section-note">${entry.thoughtSentence}</p>
                        `
                    : null} ${entry.goals.length
                      ? html`
                          <p class="intent-entry__goals">
                            Goals: ${entry.goals.join(", ")}
                          </p>
                        `
                      : null} ${entry.commandScript
                        ? html`
                          <pre class="intent-entry__script">${entry
                            .commandScript}</pre>
                        `
                        : null} ${entry.sourceTopics.length
                          ? html`
                          <p class="intent-entry__topics">
                            Sources: ${entry.sourceTopics.join(", ")}
                          </p>
                        `
                          : null} ${entry.moodDelta
                            ? html`
                          <p class="section-note">Mood delta: ${entry
                                .moodDelta}</p>
                        `
                            : null}
                      ${entry.memory && entry.memory.text
                  ? html`
                          <p class="section-note">
                            Memory: ${entry.memory.emoji
                      ? `${entry.memory.emoji} `
                      : ""}${entry.memory.text}
                          </p>
                        `
                  : null}
                    </li>
                  `,
          )}
            </ul>
          `
          : html`
            <p class="empty-placeholder">
              No FeelingIntent messages received from the pilot yet.
            </p>
          `}
      `,
    });

    const promptCard = this.renderSurfaceCard({
      id: "pilot-latest-prompt",
      title: "Latest prompt",
      wide: true,
      content: this.lastPrompt
        ? html`
          <p class="section-note">
            Snapshot of the text sent to the language model for the most recent planning cycle.
          </p>
          <pre class="llm-output">${this.lastPrompt}</pre>
        `
        : html`
          <p class="section-note">
            Prompt history will appear here once the planner publishes telemetry.
          </p>
          <p class="empty-placeholder">No prompt captured yet.</p>
        `,
    });

    const sensationsCard = this.renderSurfaceCard({
      id: "pilot-sensations",
      title: "Recent sensations",
      content: html`
          <p class="section-note">
            Sliding window of sensation summaries captured over the last ${this
          .config.windowSeconds != null
          ? this.config.windowSeconds.toFixed(1)
          : "—"}
            seconds.
          </p>
          ${this.recentSensations.length
          ? html`
              <ul class="sensations-list">
                ${this.recentSensations.slice(0, 8).map(
            (sensation) =>
              html`
                      <li class="sensations-entry">
                        <p class="sensations-entry__meta">
                          <strong>${sensation.kind ||
                "unknown"}</strong> via ${sensation
                  .topic} ${sensation.hint
                    ? html`
                              <span>(${sensation.hint})</span>
                            `
                    : null}
                        </p>
                        ${sensation.jsonPayload
                  ? html`
                            <pre class="intent-entry__script">${sensation
                      .jsonPayload}</pre>
                          `
                  : html`
                            <p class="section-note">No JSON payload supplied.</p>
                          `}
                        <p class="section-note">Vector length: ${sensation
                  .vectorLength}</p>
                      </li>
                    `,
          )}
              </ul>
            `
          : html`
              <p class="empty-placeholder">No sensations recorded in the current window.</p>
            `}
        `,
    });

    const scriptsCard = this.renderSurfaceCard({
      id: "pilot-scripts",
      title: "Command scripts",
      content: html`
        <p class="section-note">
          Scripts dispatched by the pilot after validating the LLM output and action
          catalogue.
        </p>
        ${this.scriptRuns.length
          ? html`
            <ul class="scripts-list">
              ${this.scriptRuns.slice(0, 6).map(
            (script) =>
              html`
                    <li class="script-entry">
                      <p class="script-entry__meta">
                        <span class="script-entry__status">${script
                  .status}</span>
                        ${script.source
                  ? html`
                            <span>• ${script.source}</span>
                          `
                  : null} ${script.startedAt
                    ? html`
                            <span>• ${script.startedAt}</span>
                          `
                    : null} ${script.finishedAt
                      ? html`
                            <span>→ ${script.finishedAt}</span>
                          `
                      : null}
                      </p>
                      ${script.error
                  ? html`
                          <p class="surface-status" data-variant="error">${script
                      .error}</p>
                        `
                  : null} ${script.usedActions.length
                    ? html`
                          <p class="section-note">
                            Actions: ${script.usedActions.join(", ")}
                          </p>
                        `
                    : null} ${script.actions.length
                      ? html`
                          <ul class="script-entry__actions">
                            ${script.actions.map(
                        (action) =>
                          html`
                                  <li>
                                    ${action.action || "unknown"} – ${action
                              .status} ${action.timestamp
                                ? html`
                                        <span>(${action.timestamp})</span>
                                      `
                                : null}
                                  </li>
                                `,
                      )}
                          </ul>
                        `
                      : null}
                    </li>
                  `,
          )}
            </ul>
          `
          : html`
            <p class="empty-placeholder">No command scripts executed yet.</p>
          `}
      `,
    });

    const lastLLMCard = this.lastLLM
      ? this.renderSurfaceCard({
        id: "pilot-latest-llm",
        title: "Latest LLM response",
        wide: true,
        content: html`
            <p class="section-note">
              Trimmed copy of the model output that informed the most recent FeelingIntent
              message.
            </p>
            <pre class="llm-output">${this.lastLLM}</pre>
          `,
      })
      : null;

    const harnessCard = this.renderHarnessCard();

    const cards = [
      harnessCard,
      statusCard,
      sensationsCard,
      scriptsCard,
      lastLLMCard,
      html`
        <div class="pilot-dashboard__intent-prompt">
          ${intentCard}
          ${promptCard}
        </div>
      `,
    ].filter(Boolean);

    return html`
      <div class="surface-grid surface-grid--wide">
        ${cards}
      </div>
    `;
  }

  renderHarnessCard() {
    const recording = this.eyeRecordingState === "recording";
    const uploading = this.eyeRecordingState === "uploading";

    return this.renderSurfaceCard({
      id: "pilot-browser-hooks",
      title: "Browser hooks",
      wide: true,
      content: html`
        <div class="harness-grid">
          <section class="harness-panel">
            <h4>Eye video capture</h4>
            <p class="section-note">
              Record a short WebM clip using your browser camera and publish it to
              <code>/eye/browser_clip</code> for ingestion by the Eye module.
            </p>
            <label>
              Clip length (seconds)
              <input
                type="number"
                min="1"
                max="30"
                step="1"
                inputmode="numeric"
                .value=${this.eyeDurationSeconds}
                @input=${(event) => this._handleEyeDurationInput(event)}
              />
            </label>
            <div class="surface-actions">
              <button
                type="button"
                class="surface-button"
                @click=${() => this.startEyeRecording()}
                ?disabled=${recording || uploading}
              >
                🎥 Start recording
              </button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click=${() => this.stopEyeRecording()}
                ?disabled=${!recording}
              >
                ⏹ Stop
              </button>
            </div>
            <p class="harness-meta">Status: ${this.eyeStatusMessage}</p>
            ${this.eyeClipSummary
              ? html`<p class="surface-note surface-mono">${this.eyeClipSummary}</p>`
              : null}
            ${this.eyeFeedback
              ? html`<p class="surface-status" data-variant=${this.eyeFeedbackTone || ""}>
                  ${this.eyeFeedback}
                </p>`
              : null}
          </section>

          <section class="harness-panel">
            <h4>Ear transcript injector</h4>
            <p class="section-note">
              Send transcript lines through the ASR harness to feed the ear module directly.
            </p>
            <form class="harness-form" @submit=${(event) => this.sendEarTranscript(event)}>
              <label>
                Transcript text
                <textarea
                  placeholder="e.g. Observed a person waving near the doorway"
                  .value=${this.earTextInput}
                  @input=${(event) => {
                    this.earTextInput = event.target.value;
                  }}
                ></textarea>
              </label>
              <div class="surface-actions">
                <button
                  type="submit"
                  class="surface-button"
                  ?disabled=${this.earTextInput.trim().length === 0}
                >
                  🗣️ Send to ear
                </button>
                <button
                  type="button"
                  class="surface-button surface-button--ghost"
                  @click=${() => this._clearEarInput()}
                  ?disabled=${this.earTextInput.trim().length === 0}
                >
                  🧼 Clear
                </button>
              </div>
            </form>
            ${this.earFeedback
              ? html`<p class="surface-status" data-variant=${this.earFeedbackTone || ""}>
                  ${this.earFeedback}
                </p>`
              : null}
          </section>
        </div>
      `,
    });
  }

  renderSurfaceCard({ id, title, content, wide = false }) {
    const cardId = typeof id === "string" ? id.trim() : "";
    const collapsed = cardId ? this.isCardCollapsed(cardId) : false;
    const contentId = cardId ? `${cardId}-content` : undefined;
    const classes = classMap({
      "surface-card": true,
      "surface-card--wide": Boolean(wide),
      "surface-card--collapsible": true,
      "surface-card--collapsed": collapsed,
    });

    return html`
      <article class=${classes} data-card-id=${cardId || undefined}>
        <header class="surface-card__header">
          <h3 class="surface-card__title">${title}</h3>
          <button
            type="button"
            class="surface-card__toggle"
            aria-expanded=${collapsed ? "false" : "true"}
            aria-controls=${contentId || undefined}
            aria-label=${collapsed ? `Expand ${title}` : `Collapse ${title}`}
            @click=${() => cardId && this.toggleCardCollapsed(cardId)}
          >
            <span class="surface-card__toggleIcon" aria-hidden="true">${collapsed ? "▸" : "▾"}</span>
            <span class="surface-card__toggleLabel" aria-hidden="true">${collapsed ? "Expand" : "Collapse"}</span>
            <span class="surface-card__toggleText">${collapsed ? `Expand ${title}` : `Collapse ${title}`}</span>
          </button>
        </header>
        <div
          id=${contentId || undefined}
          class="surface-card__content"
          ?hidden=${collapsed}
        >
          ${content}
        </div>
      </article>
    `;
  }

  _handleEyeDurationInput(event) {
    const next = Number(event?.target?.value);
    if (Number.isFinite(next) && next > 0) {
      this.eyeDurationSeconds = Math.min(Math.max(Math.floor(next), 1), 60);
      return;
    }
    this.eyeDurationSeconds = 5;
  }

  _selectRecorderOptions() {
    if (typeof MediaRecorder === "undefined") {
      return {};
    }
    const candidates = [
      "video/webm;codecs=vp9,opus",
      "video/webm;codecs=vp8,opus",
      "video/webm",
    ];
    for (const mimeType of candidates) {
      if (MediaRecorder.isTypeSupported(mimeType)) {
        return { mimeType };
      }
    }
    return {};
  }

  async startEyeRecording() {
    if (this.eyeRecordingState === "recording" || this.eyeRecordingState === "uploading") {
      return;
    }
    if (
      typeof navigator === "undefined" || !navigator.mediaDevices ||
      typeof navigator.mediaDevices.getUserMedia !== "function"
    ) {
      this._setEyeFeedback("Browser camera access is unavailable.", "error");
      return;
    }
    if (typeof MediaRecorder === "undefined") {
      this._setEyeFeedback("MediaRecorder is not supported in this browser.", "error");
      return;
    }

    try {
      this.eyeRecordingState = "recording";
      this.eyeStatusMessage = "Requesting camera access…";
      const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: true });
      this._eyeMediaStream = stream;

      const options = this._selectRecorderOptions();
      this._eyeChunks = [];
      const recorder = new MediaRecorder(stream, options);
      recorder.addEventListener("dataavailable", (event) => {
        if (event?.data && event.data.size > 0) {
          this._eyeChunks.push(event.data);
        }
      });
      recorder.addEventListener("stop", () => {
        void this._publishEyeClip(recorder.mimeType);
      });
      recorder.addEventListener("error", (event) => {
        const message = event?.error?.message || "Recorder error";
        this._setEyeFeedback(message, "error");
        this._teardownEyeRecorder();
      });

      const durationMs = Math.max(1, Number(this.eyeDurationSeconds)) * 1000;
      recorder.start();
      this.eyeStatusMessage = "Recording…";
      this._eyeRecorder = recorder;
      this._eyeStopTimer = setTimeout(() => this.stopEyeRecording(), durationMs);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.eyeRecordingState = "idle";
      this._setEyeFeedback(`Unable to start recording: ${message}`, "error");
      this._teardownEyeRecorder();
    }
  }

  stopEyeRecording() {
    if (this._eyeStopTimer) {
      clearTimeout(this._eyeStopTimer);
      this._eyeStopTimer = null;
    }
    if (this._eyeRecorder && this._eyeRecorder.state !== "inactive") {
      this.eyeStatusMessage = "Stopping recorder…";
      try {
        this._eyeRecorder.stop();
      } catch (_error) {
        this._teardownEyeRecorder();
      }
      return;
    }
    this._teardownEyeRecorder();
  }

  async _publishEyeClip(mimeTypeHint) {
    const chunks = Array.isArray(this._eyeChunks) ? [...this._eyeChunks] : [];
    this._eyeChunks = [];
    const recorder = this._eyeRecorder;
    this._eyeRecorder = null;
    if (this._eyeStopTimer) {
      clearTimeout(this._eyeStopTimer);
      this._eyeStopTimer = null;
    }
    if (this._eyeMediaStream) {
      try {
        this._eyeMediaStream.getTracks().forEach((track) => track.stop());
      } catch (_error) {
        // Ignore stream teardown issues.
      }
    }
    this._eyeMediaStream = null;

    if (chunks.length === 0) {
      this.eyeRecordingState = "idle";
      this.eyeStatusMessage = "No frames captured.";
      return;
    }

    this.eyeRecordingState = "uploading";
    this.eyeStatusMessage = "Encoding clip…";
    try {
      const blob = new Blob(chunks, {
        type: mimeTypeHint || (recorder ? recorder.mimeType : "video/webm") || "video/webm",
      });
      const base64 = await blobToBase64(blob);
      const payload = {
        mime_type: blob.type || "video/webm",
        bytes: blob.size,
        captured_at: new Date().toISOString(),
        source: "pilot-cockpit",
        encoding: "base64",
        data: base64,
      };
      const publisher = this._ensureEyePublisher();
      if (!publisher) {
        throw new Error("Unable to open Eye publisher");
      }
      await publisher.ready.catch(() => {});
      publisher.send({ data: JSON.stringify(payload) });
      const sizeLabel = formatByteSize(blob.size);
      this.eyeClipSummary = `${sizeLabel} ${blob.type || "video"} clip sent at ${new Date().toLocaleTimeString()}`;
      this.eyeStatusMessage = "Clip published to Eye.";
      this._setEyeFeedback("Browser recording forwarded to the Eye module.", "success", true);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this._setEyeFeedback(`Failed to publish clip: ${message}`);
      this.eyeStatusMessage = "Clip failed to send.";
    } finally {
      this.eyeRecordingState = "idle";
    }
  }

  _teardownEyeRecorder() {
    if (this._eyeStopTimer) {
      clearTimeout(this._eyeStopTimer);
      this._eyeStopTimer = null;
    }
    if (this._eyeRecorder && this._eyeRecorder.state !== "inactive") {
      try {
        this._eyeRecorder.stop();
      } catch (_error) {
        // ignore recorder shutdown issues
      }
    }
    this._eyeRecorder = null;
    if (this._eyeMediaStream) {
      try {
        this._eyeMediaStream.getTracks().forEach((track) => track.stop());
      } catch (_error) {
        // ignore stream teardown
      }
    }
    this._eyeMediaStream = null;
    this._eyeChunks = [];
    if (this.eyeRecordingState !== "idle") {
      this.eyeRecordingState = "idle";
      this.eyeStatusMessage = "Recording cancelled.";
    }
  }

  _ensureEyePublisher() {
    if (this._eyePublisher) {
      return this._eyePublisher;
    }
    try {
      const socket = createTopicSocket({
        module: "eye",
        topic: "/eye/browser_clip",
        type: "std_msgs/msg/String",
        role: "publish",
      });
      socket.addEventListener("close", () => {
        if (this._eyePublisher === socket) {
          this._eyePublisher = null;
        }
      });
      socket.addEventListener("error", () => {
        if (this._eyePublisher === socket) {
          this._eyePublisher = null;
        }
      });
      this._eyePublisher = socket;
      return socket;
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this._setEyeFeedback(`Unable to open Eye publisher: ${message}`);
      return null;
    }
  }

  _setEyeFeedback(message, tone = "error", autoClear = false) {
    this.eyeFeedback = message;
    this.eyeFeedbackTone = tone;
    if (this._eyeFeedbackTimer) {
      clearTimeout(this._eyeFeedbackTimer);
      this._eyeFeedbackTimer = null;
    }
    if (autoClear && message) {
      this._eyeFeedbackTimer = setTimeout(() => {
        this.eyeFeedback = "";
        this.eyeFeedbackTone = "";
        this._eyeFeedbackTimer = null;
      }, 3200);
    }
  }

  _ensureEarPublisher() {
    if (this._earPublisher) {
      return this._earPublisher;
    }
    try {
      const socket = createTopicSocket({
        module: "ear",
        topic: "/ear/hole",
        type: "std_msgs/msg/String",
        role: "publish",
      });
      socket.addEventListener("close", () => {
        if (this._earPublisher === socket) {
          this._earPublisher = null;
        }
      });
      socket.addEventListener("error", () => {
        if (this._earPublisher === socket) {
          this._earPublisher = null;
        }
      });
      this._earPublisher = socket;
      return socket;
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this._setEarFeedback(`Unable to reach ear topic: ${message}`);
      return null;
    }
  }

  _clearEarInput() {
    this.earTextInput = "";
    this._setEarFeedback("", "");
  }

  sendEarTranscript(event) {
    event?.preventDefault();
    const text = this.earTextInput.trim();
    if (!text) {
      this._setEarFeedback("Enter text to inject into the ear transcript stream.", "warning", true);
      return;
    }
    const publisher = this._ensureEarPublisher();
    if (!publisher) {
      this._setEarFeedback("Ear publisher unavailable.", "error");
      return;
    }
    try {
      publisher.send({ data: text });
      this.earTextInput = "";
      this._setEarFeedback("Transcript delivered to the ear module.", "success", true);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this._setEarFeedback(`Failed to send transcript: ${message}`, "error");
    }
  }

  _setEarFeedback(message, tone = "info", autoClear = false) {
    this.earFeedback = message;
    this.earFeedbackTone = tone;
    if (this._earFeedbackTimer) {
      clearTimeout(this._earFeedbackTimer);
      this._earFeedbackTimer = null;
    }
    if (autoClear && message) {
      this._earFeedbackTimer = setTimeout(() => {
        this.earFeedback = "";
        this.earFeedbackTone = "";
        this._earFeedbackTimer = null;
      }, 2400);
    }
  }

  _connectDebugStream() {
    try {
      const socket = createTopicSocket({
        module: "pilot",
        action: "debug_stream",
      });
      socket.addEventListener("open", () => {
        this.connected = true;
      });
      socket.addEventListener("close", () => {
        this.connected = false;
      });
      socket.addEventListener("error", () => {
        this.connected = false;
      });
      socket.addEventListener(
        "message",
        (event) => this._handleDebugMessage(event),
      );
      this._debugSocket = socket;
    } catch (_error) {
      this.connected = false;
    }
  }

  _connectIntentStream() {
    try {
      const socket = createTopicSocket({
        module: "pilot",
        action: "intent_stream",
      });
      socket.addEventListener("open", () => {
        this.intentConnected = true;
      });
      socket.addEventListener("close", () => {
        this.intentConnected = false;
      });
      socket.addEventListener("error", () => {
        this.intentConnected = false;
      });
      socket.addEventListener(
        "message",
        (event) => this._handleIntentMessage(event),
      );
      this._intentSocket = socket;
    } catch (_error) {
      this.intentConnected = false;
    }
  }

  _shutdownSocket(socket) {
    if (socket && typeof socket.close === "function") {
      try {
        socket.close();
      } catch (_error) {
        // Ignore shutdown errors to keep teardown resilient.
      }
    }
  }

  _handleDebugMessage(event) {
    try {
      if (typeof event.data !== "string") {
        return;
      }
      const payload = JSON.parse(event.data);
      if (payload.event !== "message") {
        return;
      }
      let snapshotPayload = payload.data;
      if (
        snapshotPayload && typeof snapshotPayload === "object" &&
        typeof snapshotPayload.data === "string"
      ) {
        snapshotPayload = snapshotPayload.data;
      }
      const snapshot = typeof snapshotPayload === "string"
        ? JSON.parse(snapshotPayload)
        : typeof snapshotPayload === "object" && snapshotPayload !== null
          ? snapshotPayload
          : null;
      const normalised = normaliseDebugSnapshot(snapshot);
      this.moduleStatus = normalised.status;
      this.lastHeartbeat = normalised.heartbeat;
      this.config = normalised.config;
      this.recentSensations = normalised.recentSensations;
      this.scriptRuns = normalised.scripts;
      this.lastPrompt = normalised.prompt;
      this.lastLLM = normalised.lastLLM;
      this.logsCount = normalised.logs.length;
      this.errorsCount = normalised.errors.length;
      this.connected = true;
    } catch (_error) {
      // Ignore malformed payloads but keep the stream alive.
    }
  }

  _handleIntentMessage(event) {
    try {
      if (typeof event.data !== "string") {
        return;
      }
      const payload = JSON.parse(event.data);
      if (payload.event !== "message") {
        return;
      }
      const message = payload.data && typeof payload.data === "object"
        ? payload.data
        : null;
      const normalised = normaliseFeelingIntent(message);
      if (!normalised) {
        return;
      }
      const stamp = normalised.stamp;
      const displayTime = stamp ? this._formatTimestamp(stamp) : "Unknown time";
      const entry = {
        id: normalised.id || makeId("intent"),
        stamp,
        displayTime,
        attitudeEmoji: normalised.attitudeEmoji,
        spokenSentence: normalised.spokenSentence,
        thoughtSentence: normalised.thoughtSentence,
        commandScript: normalised.commandScript,
        goals: normalised.goals,
        moodDelta: normalised.moodDelta,
        sourceTopics: normalised.sourceTopics,
        memory: normalised.memory,
        episodeId: normalised.episodeId,
        situationId: normalised.situationId,
      };
      const filtered = this.intentFeed.filter((existing) =>
        existing.id !== entry.id
      );
      this.intentFeed = [entry, ...filtered].slice(0, 10);
      this.intentConnected = true;
    } catch (_error) {
      // Ignore malformed payloads.
    }
  }

  _formatTimestamp(date) {
    try {
      return date.toLocaleTimeString();
    } catch (_error) {
      return date instanceof Date ? date.toISOString() : "—";
    }
  }
}

customElements.define("pilot-dashboard", PilotDashboard);
