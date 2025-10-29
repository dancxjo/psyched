import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';
import {
  extractThreadIdFromStream,
  parseConversationSnapshot,
} from './conversation-helpers.mjs';

/**
 * Dashboard for monitoring and nudging Pete's Conversant module.
 */
class ConversantDashboard extends LitElement {
  static properties = {
    topic: { state: true },
    topicStatus: { state: true },
    topicFeedback: { state: true },
    conversations: { state: true },
    conversationFeedback: { state: true },
    selectedThreadId: { state: true },
    directMessage: { state: true },
    directFeedback: { state: true },
    threadHint: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      .surface-card__title {
        margin: 0;
      }

      .surface-status {
        margin: 0 0 0.5rem;
      }

      .surface-actions {
        display: flex;
        gap: 0.5rem;
      }

      .surface-textarea {
        min-height: 5rem;
        resize: vertical;
      }

      .surface-field + .surface-field {
        margin-top: 0.75rem;
      }

      .conversation-stream {
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
      }

      .conversation-stream__heading {
        margin: 0;
        font-size: 0.8rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--lcars-muted);
      }

      .conversation-stream__log {
        background: var(--control-surface-bg);
        border: 1px solid var(--control-surface-border);
        border-radius: var(--control-surface-radius);
        box-shadow: var(--control-surface-shadow);
        padding: 0.75rem;
        max-height: 320px;
        overflow-y: auto;
        display: flex;
        flex-direction: column;
        gap: 0.6rem;
      }

      .conversation-stream__empty {
        margin: 0;
        font-size: 0.8rem;
        color: var(--lcars-muted);
      }

      .conversation-stream__entry {
        border-left: 3px solid rgba(92, 209, 132, 0.5);
        padding: 0 0 0 0.6rem;
        display: flex;
        flex-direction: column;
        gap: 0.25rem;
      }

      .conversation-stream__entry[data-role='user'] {
        border-left-color: rgba(248, 128, 60, 0.5);
      }

      .conversation-stream__entry header {
        display: flex;
        align-items: center;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .conversation-stream__entry pre {
        margin: 0;
        white-space: pre-wrap;
        word-break: break-word;
        font-family: 'Source Code Pro', monospace;
        font-size: 0.85rem;
        color: var(--lcars-text);
      }
    `,
  ];

  constructor() {
    super();
    this.topic = '';
    this.topicStatus = 'Connecting…';
    this.topicFeedback = '';
    this.conversations = [];
    this.conversationFeedback = '';
    this.selectedThreadId = '';
    this.directMessage = '';
    this.directFeedback = '';
    this.threadHint = '';
    this._topicSocket = null;
    this._concernPublisher = null;
    this._sockets = [];
    this._conversationSockets = new Map();
    this._conversationsByThread = new Map();
  }

  updated(changedProperties) {
    if (changedProperties.has('conversations') || changedProperties.has('selectedThreadId')) {
      const streamLog = this.renderRoot?.querySelector('.conversation-stream__log');
      if (streamLog) {
        streamLog.scrollTop = streamLog.scrollHeight;
      }
    }
  }

  connectedCallback() {
    super.connectedCallback();
    this._openTopicFeed();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    for (const socket of this._sockets) {
      try {
        socket.close();
      } catch (_error) {
        // Intentionally ignore shutdown errors.
      }
    }
    this._sockets = [];
    this._topicSocket = null;
    this._concernPublisher = null;
    this._conversationSockets.clear();
    this._conversationsByThread.clear();
  }

  _openTopicFeed() {
    const socket = createTopicSocket({
      module: 'conversant',
      topic: '/conversant/topic',
      type: 'std_msgs/msg/String',
      role: 'subscribe',
      qos: { durability: 'transient_local' },
    });
    socket.addEventListener('open', () => {
      this.topicStatus = 'Live';
      this.topicFeedback = '';
    });
    socket.addEventListener('close', () => {
      this.topicStatus = 'Disconnected';
    });
    socket.addEventListener('error', () => {
      this.topicStatus = 'Error';
      this.topicFeedback = 'Unable to subscribe to /conversant/topic.';
    });
    socket.addEventListener('message', (event) => {
      const payload = this._safeParse(event);
      if (!payload || payload.event !== 'message') {
        return;
      }
      const data = this._extractDataField(payload);
      this.topic = String(data || '');
      const threadId = extractThreadIdFromStream(this.topic, this.selectedThreadId);
      if (threadId) {
        this._selectThread(threadId);
      }
      this._subscribeToConversationStream(this.topic);
    });
    this._topicSocket = socket;
    this._sockets.push(socket);
  }

  _safeParse(event) {
    if (!event || typeof event.data !== 'string') {
      return null;
    }
    try {
      return JSON.parse(event.data);
    } catch (_error) {
      return null;
    }
  }

  _extractDataField(payload) {
    if (!payload || typeof payload !== 'object') {
      return '';
    }
    const data = payload.data;
    if (data && typeof data === 'object' && 'data' in data) {
      return typeof data.data === 'string' ? data.data : String(data.data ?? '');
    }
    if (typeof data === 'string') {
      return data;
    }
    return '';
  }

  _formatTimestamp(raw) {
    const date = new Date(raw);
    if (Number.isNaN(date.getTime())) {
      return raw;
    }
    return date.toLocaleString();
  }

  _subscribeToConversationStream(streamName) {
    const topicName = typeof streamName === 'string' ? streamName.trim() : '';
    if (!topicName) {
      return null;
    }
    if (this._conversationSockets.has(topicName)) {
      return this._conversationSockets.get(topicName);
    }
    const socket = createTopicSocket({
      module: 'conversant',
      topic: topicName,
      type: 'std_msgs/msg/String',
      role: 'subscribe',
      qos: { durability: 'transient_local' },
    });
    socket.addEventListener('error', () => {
      this.conversationFeedback = `Unable to subscribe to ${topicName}.`;
    });
    socket.addEventListener('close', () => {
      this._conversationSockets.delete(topicName);
    });
    socket.addEventListener('message', (event) => {
      const payload = this._safeParse(event);
      if (!payload || payload.event !== 'message') {
        return;
      }
      const data = this._extractDataField(payload);
      const snapshot = parseConversationSnapshot(data);
      if (!snapshot) {
        return;
      }
      this._updateConversation(topicName, snapshot);
      if (this.conversationFeedback) {
        this.conversationFeedback = '';
      }
    });
    this._conversationSockets.set(topicName, socket);
    this._sockets.push(socket);
    return socket;
  }

  _updateConversation(topicName, snapshot) {
    const { threadId, userId, messages } = snapshot;
    const lastMessage = messages[messages.length - 1];
    this._conversationsByThread.set(threadId, {
      threadId,
      userId,
      topic: topicName,
      messages,
      lastTimestamp: lastMessage ? lastMessage.timestamp : '',
    });
    const ordered = Array.from(this._conversationsByThread.values()).sort((a, b) => {
      if (!a.lastTimestamp && !b.lastTimestamp) {
        return a.threadId.localeCompare(b.threadId);
      }
      if (!a.lastTimestamp) {
        return 1;
      }
      if (!b.lastTimestamp) {
        return -1;
      }
      return b.lastTimestamp.localeCompare(a.lastTimestamp);
    });
    this.conversations = ordered;
    const activeThreadFromTopic = extractThreadIdFromStream(this.topic, threadId);
    if (
      !this.selectedThreadId ||
      this.selectedThreadId === threadId ||
      activeThreadFromTopic === threadId
    ) {
      this._selectThread(threadId);
    }
  }

  _selectThread(threadId) {
    const cleaned = typeof threadId === 'string' ? threadId.trim() : '';
    if (cleaned === this.selectedThreadId) {
      if (cleaned && this.threadHint !== cleaned) {
        this.threadHint = cleaned;
      }
      return;
    }
    this.selectedThreadId = cleaned;
    if (cleaned) {
      this.threadHint = cleaned;
    }
  }

  _handleThreadSelect(event) {
    const value = event && event.target && typeof event.target.value === 'string'
      ? event.target.value
      : '';
    this._selectThread(value);
  }

  _ensureConcernPublisher() {
    if (this._concernPublisher) {
      return this._concernPublisher;
    }
    const socket = createTopicSocket({
      module: 'conversant',
      topic: '/conversant/concern',
      type: 'std_msgs/msg/String',
      role: 'publish',
    });
    socket.addEventListener('error', () => {
      this.directFeedback = 'Unable to publish to /conversant/concern.';
    });
    this._concernPublisher = socket;
    this._sockets.push(socket);
    return socket;
  }

  _handleDirectMessageSubmit(event) {
    event.preventDefault();
    const text = this.directMessage.trim();
    if (!text) {
      this.directFeedback = 'Message text is required.';
      return;
    }
    const threadId = this.threadHint.trim() || this.selectedThreadId;
    const payload = threadId
      ? JSON.stringify({ concern: text, thread_id: threadId })
      : text;
    try {
      const publisher = this._ensureConcernPublisher();
      publisher.send(JSON.stringify({ data: payload }));
      this.directFeedback = 'Direct message queued for Conversant.';
      this.directMessage = '';
    } catch (error) {
      this.directFeedback = error instanceof Error ? error.message : String(error);
    }
  }

  _clearDirectForm() {
    this.directMessage = '';
    this.threadHint = this.selectedThreadId;
    this.directFeedback = '';
  }

  render() {
    const topicVariant = this.topicStatus === 'Live' ? 'success' : this.topicStatus === 'Error' ? 'error' : '';
    const activeConversation = this.conversations.find((entry) => entry.threadId === this.selectedThreadId)
      || this.conversations[0]
      || null;
    const messages = activeConversation ? activeConversation.messages : [];
    const threadSelection = this.conversations.length
      ? html`
          <label class="surface-field">
            <span class="surface-label">Conversation thread</span>
            <select class="surface-input" @change=${(event) => this._handleThreadSelect(event)}>
              ${this.conversations.map(
                (entry) => html`<option value=${entry.threadId} ?selected=${entry.threadId === this.selectedThreadId}>
                  ${entry.threadId} · ${entry.userId}
                </option>`,
              )}
            </select>
          </label>
        `
      : html`<p class="surface-status">Waiting for conversation snapshots…</p>`;
    return html`
      <div class="surface-grid surface-grid--wide surface-grid--dense">
        <article class="surface-card surface-card--compact">
          <h3 class="surface-card__title">Conversation stream</h3>
          <p class="surface-status" data-variant=${topicVariant}>Stream feed: ${this.topicStatus}</p>
          <label class="surface-field">
            <span class="surface-label">Current stream</span>
            <input class="surface-input" type="text" .value=${this.topic} readonly />
          </label>
          <div class="conversation-stream">
            <p class="conversation-stream__heading">Live transcript</p>
            ${messages.length
              ? html`
                  <div class="conversation-stream__log">
                    ${messages.map(
                      (message) => html`
                        <article class="conversation-stream__entry" data-role=${message.role}>
                          <header>
                            <span>${message.role}</span>
                            <span>${this._formatTimestamp(message.timestamp)}</span>
                            ${message.intent ? html`<span>Intent</span>` : ''}
                          </header>
                          <pre>${message.content}</pre>
                        </article>
                      `,
                    )}
                  </div>
                `
              : html`<p class="conversation-stream__empty">Waiting for messages…</p>`}
          </div>
          ${this.topicFeedback
            ? html`<p class="surface-status" data-variant="error">${this.topicFeedback}</p>`
            : ''}
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Conversation log</h3>
          ${threadSelection}
          ${this.conversationFeedback
            ? html`<p class="surface-status" data-variant="error">${this.conversationFeedback}</p>`
            : ''}
          ${activeConversation
            ? html`
                <p class="surface-status">
                  Active thread: ${activeConversation.threadId} • User: ${activeConversation.userId}
                </p>
              `
            : ''}
          <div class="conversation-console">
            <div class="conversation-log">
              <h5>Messages</h5>
              ${messages.length
                ? html`
                    <ul>
                      ${messages.map(
                        (message) => html`
                          <li class="conversation-entry ${message.role === 'assistant' ? 'assistant' : 'user'}">
                            <header>
                              <span class="badge role">${message.role}</span>
                              <span class="badge">${this._formatTimestamp(message.timestamp)}</span>
                              ${message.intent
                                ? html`<span class="badge">Intent</span>`
                                : ''}
                            </header>
                            <pre>${message.content}</pre>
                          </li>
                        `,
                      )}
                    </ul>
                  `
                : html`<p class="surface-status">No messages recorded yet.</p>`}
            </div>
          </div>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Direct message</h3>
          <p class="surface-status">
            Send a note straight to the active conversation thread. JSON payloads are accepted when
            advanced routing is needed.
          </p>
          <form class="surface-form" @submit=${(event) => this._handleDirectMessageSubmit(event)}>
            <label class="surface-field">
              <span class="surface-label">Message</span>
              <input
                class="surface-input"
                name="message"
                type="text"
                .value=${this.directMessage}
                placeholder="e.g. Please acknowledge the battery check"
                @input=${(event) => (this.directMessage = event.target.value)}
              />
            </label>
            <label class="surface-field">
              <span class="surface-label">Thread (optional)</span>
              <input
                class="surface-input"
                type="text"
                name="thread"
                .value=${this.threadHint}
                placeholder="Leave blank for latest thread"
                @input=${(event) => (this.threadHint = event.target.value)}
              />
            </label>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Send message</button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click=${() => this._clearDirectForm()}
              >
                Clear
              </button>
            </div>
          </form>
          ${this.directFeedback
            ? html`<p class="surface-status" data-variant="info">${this.directFeedback}</p>`
            : ''}
        </article>
      </div>
    `;
  }
}

customElements.define('conversant-dashboard', ConversantDashboard);
