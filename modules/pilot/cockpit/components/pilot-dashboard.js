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
    this._cardManager = createCollapsedCardManager("module-pilot");
    this.collapsedCards = this._cardManager.getCollapsedIds();

    this._debugSocket = null;
    this._intentSocket = null;
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
    this._debugSocket = null;
    this._intentSocket = null;
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

    const cards = [
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
            ${collapsed ? "Expand" : "Collapse"}
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
