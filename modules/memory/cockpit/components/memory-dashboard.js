import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { surfaceStyles } from '/components/cockpit-style.js';
import { createTopicSocket, callModuleAction } from '/js/cockpit.js';
import {
  buildMemoryQueryPayload,
  buildMemoryStorePayload,
  normaliseRecallResults,
} from './memory-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

/**
 * Memory module dashboard for querying and seeding semantic memories.
 *
 * Events emitted:
 * - ``memory-query-request`` → `{ detail: MemoryQueryPayload }`
 * - ``memory-store-request`` → `{ detail: MemoryStorePayload }`
 */
class MemoryDashboard extends LitElement {
  static properties = {
    statusMessage: { state: true },
    statusTone: { state: true },
    queryText: { state: true },
    queryTopK: { state: true },
    queryFeedback: { state: true },
    queryResults: { state: true },
    storeTitle: { state: true },
    storeBody: { state: true },
    storeTags: { state: true },
    storeFeedback: { state: true },
    memoryLog: { state: true },
    memoryLogStatus: { state: true },
    memoryLogTone: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      .result-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.35rem;
      }

      .result-entry__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .surface-card__header {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.5rem;
      }

      .memory-log__meta {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .memory-log__title {
        margin: 0.3rem 0 0.4rem;
        font-size: 1rem;
      }

      .memory-log__summary {
        margin: 0 0 0.5rem;
        color: var(--lcars-text);
      }

      .memory-log__details {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .memory-log__tags {
        font-style: italic;
      }

      .memory-log__metadata {
        margin-top: 0.5rem;
      }

      .memory-log__metadata pre {
        background: rgba(0, 0, 0, 0.35);
        border-radius: 0.5rem;
        padding: 0.5rem;
        white-space: pre-wrap;
        word-break: break-word;
        font-family: var(--cockpit-mono, monospace);
        font-size: 0.75rem;
        overflow-x: auto;
      }
    `,
  ];

  constructor() {
    super();
    this.statusMessage = 'Query the knowledge base or persist new insights.';
    this.statusTone = 'info';
    this.queryText = '';
    this.queryTopK = 5;
    this.queryFeedback = '';
    this.queryResults = [];
    this.storeTitle = '';
    this.storeBody = '';
    this.storeTags = '';
    this.storeFeedback = '';
    this.memoryLog = [];
    this.memoryLogStatus = 'Connecting to pilot memory feed…';
    this.memoryLogTone = 'info';
    this._memoryFeedSocket = null;
    this._memoryFeedReconnectHandle = 0;
    this._memoryFeedMessageListener = (event) => this._processMemoryFeedEvent(event);
    this._memoryFeedCloseListener = () => this._handleMemoryFeedClosure();
    this._memoryFeedErrorListener = () => this._handleMemoryFeedError();
  }

  connectedCallback() {
    super.connectedCallback();
    this._openMemoryFeed();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._closeMemoryFeed();
  }

  render() {
    const runQueryAction = { icon: '🔍', label: 'Run query' };
    const clearQueryAction = { icon: '🧹', label: 'Clear' };
    const persistMemoryAction = { icon: '💾', label: 'Persist memory' };
    const clearStoreAction = { icon: '🧼', label: 'Clear form' };
    const simulateIngestAction = { icon: '🧪', label: 'Simulate ingest' };
    const clearLogAction = { icon: '🧹', label: 'Clear log' };

    return html`
      <div class="surface-grid surface-grid--wide surface-grid--dense">
        <article class="surface-card surface-card--wide surface-card--compact">
          <h3 class="surface-card__title">Memory search</h3>
          <p class="surface-status" data-variant="${this.statusTone}">${this.statusMessage}</p>
          <form class="surface-form surface-form--compact" @submit=${this.handleQuerySubmit}>
            <label class="surface-field">
              <span class="surface-label">Query text</span>
              <textarea
                class="surface-textarea"
                name="query"
                placeholder="e.g. What did Pete observe during the afternoon patrol?"
                .value=${this.queryText}
                @input=${(event) => (this.queryText = event.target.value)}
              ></textarea>
            </label>
            <div class="surface-grid surface-grid--dense surface-grid--narrow">
              <label class="surface-field">
                <span class="surface-label">Results</span>
                <input
                  class="surface-input surface-input--small"
                  type="number"
                  min="1"
                  max="20"
                  .value=${String(this.queryTopK)}
                  @input=${(event) => (this.queryTopK = Number(event.target.value))}
                />
              </label>
            </div>
            ${this.queryFeedback
              ? html`<p class="surface-status" data-variant="error">${this.queryFeedback}</p>`
              : ''}
            <div class="surface-actions">
              <button
                type="submit"
                class="surface-button"
                aria-label="${runQueryAction.label}"
                title="${runQueryAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true">${runQueryAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true">${runQueryAction.label}</span>
              </button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click=${this.clearQuery}
                aria-label="${clearQueryAction.label}"
                title="${clearQueryAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true">${clearQueryAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true">${clearQueryAction.label}</span>
              </button>
            </div>
          </form>
          ${this.queryResults.length
            ? html`<ol class="surface-list surface-list--scrollable">
                ${this.queryResults.map(
                  (entry) => html`<li class="result-entry">
                    <div class="result-entry__meta">
                      <span>${entry.timestamp}</span>
                      <span>${entry.score.toFixed(2)}</span>
                      ${entry.tags.length
                        ? html`<span>${entry.tags.map((tag) => `#${tag}`).join(' ')}</span>`
                        : ''}
                    </div>
                    <h4 class="result-entry__title">${entry.title}</h4>
                    <p class="result-entry__body">${entry.body}</p>
                  </li>`
                )}
              </ol>`
            : html`<p class="surface-empty">No query results yet.</p>`}
        </article>

        <article class="surface-card surface-card--wide surface-card--compact">
          <h3 class="surface-card__title">Store new memory</h3>
          ${this.storeFeedback
            ? html`<p class="surface-status" data-variant="error">${this.storeFeedback}</p>`
            : ''}
          <form class="surface-form surface-form--compact" @submit=${this.handleStoreSubmit}>
            <label class="surface-field">
              <span class="surface-label">Title</span>
              <input
                class="surface-input"
                type="text"
                name="title"
                .value=${this.storeTitle}
                @input=${(event) => (this.storeTitle = event.target.value)}
              />
            </label>
            <label class="surface-field">
              <span class="surface-label">Details</span>
              <textarea
                class="surface-textarea"
                name="body"
                .value=${this.storeBody}
                @input=${(event) => (this.storeBody = event.target.value)}
              ></textarea>
            </label>
            <label class="surface-field">
              <span class="surface-label">Tags</span>
              <input
                class="surface-input"
                type="text"
                name="tags"
                placeholder="comma,separated,tags"
                .value=${this.storeTags}
                @input=${(event) => (this.storeTags = event.target.value)}
              />
            </label>
            <div class="surface-actions">
              <button
                type="submit"
                class="surface-button"
                aria-label="${persistMemoryAction.label}"
                title="${persistMemoryAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true">${persistMemoryAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true">${persistMemoryAction.label}</span>
              </button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click=${this.clearStoreForm}
                aria-label="${clearStoreAction.label}"
                title="${clearStoreAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true">${clearStoreAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true">${clearStoreAction.label}</span>
              </button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click=${this.simulateIngest}
                aria-label="${simulateIngestAction.label}"
                title="${simulateIngestAction.label}"
              >
                <span class="surface-action__icon" aria-hidden="true">${simulateIngestAction.icon}</span>
                <span class="surface-action__label" aria-hidden="true">${simulateIngestAction.label}</span>
              </button>
            </div>
          </form>
        </article>

        <article class="surface-card surface-card--wide surface-card--compact">
          <header class="surface-card__header">
            <h3 class="surface-card__title">Pilot memory feed</h3>
            <button
              type="button"
              class="surface-action"
              ?disabled=${!this.memoryLog.length}
              @click=${this.clearMemoryLog}
              aria-label="${clearLogAction.label}"
              title="${clearLogAction.label}"
            >
              <span class="surface-action__icon" aria-hidden="true">${clearLogAction.icon}</span>
              <span class="surface-action__label" aria-hidden="true">${clearLogAction.label}</span>
            </button>
          </header>
          <p class="surface-status" data-variant=${this.memoryLogTone}>${this.memoryLogStatus}</p>
          <ul class="surface-log">
            ${this.memoryLog.length
              ? this.memoryLog.map((entry) => this.renderMemoryLogEntry(entry))
              : html`<li class="surface-log__entry">No memory events yet.</li>`}
          </ul>
        </article>
      </div>
    `;
  }

  renderMemoryLogEntry(entry) {
    return html`<li class="surface-log__entry" key=${entry.id}>
      <div class="memory-log__meta">
        <span>${entry.displayTime}</span>
        ${entry.kind ? html`<span>${entry.kind}</span>` : ''}
        ${entry.source ? html`<span>${entry.source}</span>` : ''}
        ${entry.frameId ? html`<span>${entry.frameId}</span>` : ''}
      </div>
      <h4 class="memory-log__title">${entry.title}</h4>
      ${entry.summary ? html`<p class="memory-log__summary">${entry.summary}</p>` : ''}
      <div class="memory-log__details">
        ${entry.tags.length ? html`<span class="memory-log__tags">${entry.tags.map((tag) => `#${tag}`).join(' ')}</span>` : ''}
        ${entry.memoryId ? html`<span class="memory-log__id"><code>${entry.memoryId}</code></span>` : ''}
        ${entry.vectorId ? html`<span class="memory-log__id"><code>${entry.vectorId}</code></span>` : ''}
      </div>
      ${entry.metadataJson
        ? html`<details class="memory-log__metadata">
            <summary>Metadata</summary>
            <pre>${entry.metadataJson}</pre>
          </details>`
        : ''}
    </li>`;
  }

  async handleQuerySubmit(event) {
    event.preventDefault();
    const payload = buildMemoryQueryPayload({ query: this.queryText, topK: this.queryTopK });
    if (!payload.ok) {
      this.queryFeedback = payload.error;
      return;
    }
    this.queryFeedback = '';
    this.dispatchEvent(
      new CustomEvent('memory-query-request', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = 'Searching stored memories…';
    this.statusTone = 'info';
    try {
      const response = await callModuleAction('memory', 'recall_text', {
        query: payload.value.query,
        k: payload.value.top_k,
      });
      const results = normaliseRecallResults(response.result?.results);
      const entries = results
        .slice(0, 30)
        .map((result) => this._buildResultEntry(result));
      this.queryResults = entries;
      if (entries.length === 0) {
        this.statusMessage = 'No memories matched the query.';
        this.statusTone = 'warning';
      } else {
        const count = entries.length;
        this.statusMessage = `Received ${count} matching ${count === 1 ? 'memory' : 'memories'}.`;
        this.statusTone = 'success';
      }
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.queryFeedback = message;
      this.statusMessage = 'Memory recall failed.';
      this.statusTone = 'error';
    }
  }

  handleStoreSubmit(event) {
    event.preventDefault();
    const payload = buildMemoryStorePayload({
      title: this.storeTitle,
      body: this.storeBody,
      tags: this.storeTags,
    });
    if (!payload.ok) {
      this.storeFeedback = payload.error;
      return;
    }
    this.storeFeedback = '';
    this.dispatchEvent(
      new CustomEvent('memory-store-request', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = 'Memory submitted for persistence.';
    this.statusTone = 'success';
    this.clearStoreForm();
  }

  clearQuery() {
    this.queryText = '';
    this.queryTopK = 5;
    this.queryFeedback = '';
  }

  clearStoreForm() {
    this.storeTitle = '';
    this.storeBody = '';
    this.storeTags = '';
  }

  clearMemoryLog() {
    this.memoryLog = [];
    this.memoryLogStatus = 'Log cleared. Awaiting new entries…';
    this.memoryLogTone = 'info';
  }

  simulateIngest() {
    this.recordResult({
      title: this.storeTitle || 'Synthetic event',
      body: this.storeBody || 'Generated sample memory entry.',
      score: 0.42,
      tags: this.storeTags ? this.storeTags.split(',').map((tag) => tag.trim()).filter(Boolean) : [],
    });
  }

  _openMemoryFeed() {
    if (this._memoryFeedSocket) {
      return;
    }
    this.memoryLogStatus = 'Connecting to pilot memory feed…';
    this.memoryLogTone = 'info';
    try {
      const socket = createTopicSocket({
        module: 'memory',
        topic: '/memory/pilot_feed',
        type: 'std_msgs/msg/String',
        role: 'subscribe',
        queueLength: 50,
      });
      socket.addEventListener('message', this._memoryFeedMessageListener);
      socket.addEventListener('close', this._memoryFeedCloseListener);
      socket.addEventListener('error', this._memoryFeedErrorListener);
      this._memoryFeedSocket = socket;
    } catch (error) {
      this.memoryLogStatus = error instanceof Error ? error.message : String(error);
      this.memoryLogTone = 'error';
      this._scheduleMemoryFeedReconnect();
    }
  }

  _closeMemoryFeed() {
    this._cancelMemoryFeedReconnect();
    const socket = this._memoryFeedSocket;
    if (!socket) {
      return;
    }
    socket.removeEventListener('message', this._memoryFeedMessageListener);
    socket.removeEventListener('close', this._memoryFeedCloseListener);
    socket.removeEventListener('error', this._memoryFeedErrorListener);
    try {
      socket.close();
    } catch (_error) {
      // ignore teardown failures
    }
    this._memoryFeedSocket = null;
  }

  _cancelMemoryFeedReconnect() {
    if (this._memoryFeedReconnectHandle) {
      globalThis.clearTimeout(this._memoryFeedReconnectHandle);
      this._memoryFeedReconnectHandle = 0;
    }
  }

  _processMemoryFeedEvent(event) {
    let payload;
    try {
      payload = JSON.parse(event.data);
    } catch (_error) {
      return;
    }
    const kind = payload && typeof payload === 'object' ? payload.event : '';
    if (kind === 'status') {
      const state = payload.data && typeof payload.data === 'object' ? payload.data.state : '';
      if (state === 'ready') {
        this.memoryLogStatus = 'Pilot memory feed connected.';
        this.memoryLogTone = 'success';
        this._cancelMemoryFeedReconnect();
      } else if (state === 'closed') {
        this.memoryLogStatus = 'Pilot memory feed closed.';
        this.memoryLogTone = 'warning';
      }
      return;
    }
    if (kind !== 'message') {
      return;
    }
    const message = payload.data && typeof payload.data === 'object' ? payload.data : null;
    const raw = message && typeof message.data === 'string' ? message.data : '';
    if (!raw) {
      return;
    }
    let parsed;
    try {
      parsed = JSON.parse(raw);
    } catch (_error) {
      this.memoryLogStatus = 'Received malformed memory log entry.';
      this.memoryLogTone = 'error';
      return;
    }
    const entry = this._normaliseMemoryLog(parsed);
    if (!entry) {
      return;
    }
    this._appendMemoryLog(entry);
    this.memoryLogStatus = 'Receiving pilot memory writes.';
    this.memoryLogTone = 'success';
  }

  _handleMemoryFeedClosure() {
    this._memoryFeedSocket = null;
    this.memoryLogStatus = 'Memory feed disconnected. Attempting to reconnect…';
    this.memoryLogTone = 'warning';
    this._scheduleMemoryFeedReconnect();
  }

  _handleMemoryFeedError() {
    this.memoryLogStatus = 'Memory feed error. Attempting to reconnect…';
    this.memoryLogTone = 'error';
    this._scheduleMemoryFeedReconnect();
  }

  _scheduleMemoryFeedReconnect() {
    if (!this.isConnected) {
      return;
    }
    if (this._memoryFeedReconnectHandle) {
      return;
    }
    this._memoryFeedReconnectHandle = globalThis.setTimeout(() => {
      this._memoryFeedReconnectHandle = 0;
      this._openMemoryFeed();
    }, 3000);
  }

  _appendMemoryLog(entry) {
    const identifier = entry.memoryId;
    const existing = identifier
      ? this.memoryLog.filter((item) => item.memoryId !== identifier)
      : this.memoryLog.slice();
    this.memoryLog = [entry, ...existing].slice(0, 40);
  }

  _normaliseMemoryLog(message) {
    if (!message || typeof message !== 'object') {
      return null;
    }
    const memoryId = this._safeString(message.memory_id);
    const vectorId = this._safeString(message.vector_id);
    const kind = this._safeString(message.kind);
    const source = this._safeString(message.source);
    const frameId = this._safeString(message.frame_id);
    const timestamp = this._toDate(message.timestamp);
    const metadata =
      message.metadata && typeof message.metadata === 'object' && !Array.isArray(message.metadata)
        ? message.metadata
        : {};

    let summary = this._safeString(message.summary);
    if (!summary) {
      summary =
        this._safeString(metadata.body) ||
        this._safeString(metadata.spoken_sentence) ||
        this._safeString(metadata.thought_sentence) ||
        this._safeString(metadata.situation_overview);
    }

    let title =
      this._safeString(metadata.title) ||
      this._safeString(metadata.memory_tag) ||
      this._safeString(metadata.topic) ||
      this._safeString(metadata.kind);
    if (!title && summary) {
      title = summary;
      summary = '';
    }
    if (!title) {
      title = kind || 'Memory entry';
    }

    if (summary.length > 280) {
      summary = `${summary.slice(0, 280)}…`;
    }
    if (title.length > 120) {
      title = `${title.slice(0, 120)}…`;
    }

    const tags = new Set();
    const labels = Array.isArray(message.labels) ? message.labels : [];
    for (const label of labels) {
      const text = this._safeString(label);
      if (text) {
        tags.add(text);
      }
    }
    const metadataTags = Array.isArray(metadata.tags) ? metadata.tags : [];
    for (const tag of metadataTags) {
      const text = this._safeString(tag);
      if (text) {
        tags.add(text);
      }
    }
    const sources = Array.isArray(metadata.source_topics) ? metadata.source_topics : [];
    for (const topic of sources) {
      const text = this._safeString(topic);
      if (text) {
        tags.add(text);
      }
    }
    const collection = this._safeString(metadata.memory_collection_text);
    if (collection) {
      tags.add(collection);
    }
    const memoryTag = this._safeString(metadata.memory_tag);
    if (memoryTag) {
      tags.add(memoryTag);
    }

    let metadataJson = '';
    try {
      metadataJson = JSON.stringify(metadata, null, 2);
      if (metadataJson.length > 2000) {
        metadataJson = `${metadataJson.slice(0, 2000)}…`;
      }
    } catch (_error) {
      metadataJson = '';
    }

    const fallbackTime = this._formatTimestamp(new Date());
    const displayTime = this._formatTimestamp(timestamp) || fallbackTime || '';

    return {
      id: memoryId || makeId('memory-log'),
      displayTime,
      timestamp,
      memoryId,
      vectorId,
      kind,
      source,
      frameId,
      title,
      summary,
      tags: Array.from(tags),
      metadataJson,
    };
  }

  _safeString(value) {
    if (typeof value !== 'string') {
      return '';
    }
    const trimmed = value.trim();
    return trimmed;
  }

  _toDate(value) {
    if (!value) {
      return null;
    }
    if (value instanceof Date) {
      return Number.isNaN(value.getTime()) ? null : value;
    }
    const date = new Date(value);
    return Number.isNaN(date.getTime()) ? null : date;
  }

  _formatTimestamp(date) {
    if (!(date instanceof Date) || Number.isNaN(date.getTime())) {
      return '';
    }
    try {
      return date.toLocaleTimeString();
    } catch (_error) {
      return date.toISOString();
    }
  }

  recordResult(result) {
    const entry = this._buildResultEntry(result);
    this.queryResults = [entry, ...this.queryResults].slice(0, 30);
  }

  _buildResultEntry(result) {
    const memoryId = typeof result.memoryId === 'string' && result.memoryId.trim() ? result.memoryId.trim() : '';
    const id = memoryId || makeId('memory');
    const numericScore = Number(result.score);
    const score = Number.isFinite(numericScore) ? numericScore : 0;
    const tags = Array.isArray(result.tags)
      ? result.tags.map((tag) => this._safeString(tag)).filter(Boolean)
      : [];
    const rawTimestamp = typeof result.timestamp === 'string' || result.timestamp instanceof Date
      ? result.timestamp
      : '';
    const displayTimestamp = this._formatResultTimestamp(rawTimestamp);
    const fallbackTime = this._formatTimestamp(new Date());
    const timestamp = displayTimestamp || fallbackTime || '';
    const title = this._safeString(result.title) || 'Memory entry';
    const body = this._safeString(result.body);
    return {
      id,
      timestamp,
      title,
      body,
      score,
      tags,
    };
  }

  _formatResultTimestamp(value) {
    if (value instanceof Date) {
      return this._formatTimestamp(value);
    }
    const parsed = this._toDate(value);
    if (parsed) {
      return this._formatTimestamp(parsed);
    }
    if (typeof value === 'string') {
      const trimmed = value.trim();
      if (trimmed) {
        return trimmed;
      }
    }
    return '';
  }
}

customElements.define('memory-dashboard', MemoryDashboard);
