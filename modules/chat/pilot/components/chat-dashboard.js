import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';
import { surfaceStyles } from '/components/pilot-style.js';

function generateId() {
    if (typeof crypto !== 'undefined' && crypto.randomUUID) {
        return crypto.randomUUID();
    }
    return `msg-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

class ChatDashboard extends LitElement {
    static properties = {
        status: { state: true },
        voiceStatus: { state: true },
        formFeedback: { state: true },
        voiceFeedback: { state: true },
        messages: { state: true },
        voiceLast: { state: true },
        composerRole: { state: true },
        composerSpeaker: { state: true },
        composerConfidence: { state: true },
        composerContent: { state: true },
        voiceCommand: { state: true },
    };

    static styles = [
        surfaceStyles,
        css`
      .conversation-log {
        max-height: 300px;
      }
      .conversation-entry {
        background: rgba(0, 0, 0, 0.3);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: flex;
        flex-direction: column;
        gap: 0.35rem;
      }
      .conversation-entry__meta {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }
      .conversation-entry__role {
        font-weight: 600;
        color: var(--lcars-accent-secondary);
      }
      .conversation-entry__content {
        margin: 0;
        white-space: pre-wrap;
        line-height: 1.35;
        font-size: 0.85rem;
      }
      .conversation-empty {
        color: var(--lcars-muted);
        font-style: italic;
        text-align: center;
        font-size: 0.85rem;
      }
      .voice-log {
        min-height: 3rem;
        overflow-wrap: anywhere;
      }
    `,
    ];

    constructor() {
        super();
        this.status = 'Connecting…';
        this.voiceStatus = 'Connecting…';
        this.formFeedback = '';
        this.voiceFeedback = '';
        this.conversationPublisher = null;
        this.voicePublisher = null;
        this.messages = [];
        this.voiceLast = '';
        this.composerRole = 'user';
        this.composerSpeaker = 'pilot';
        this.composerConfidence = 1.0;
        this.composerContent = '';
        this.voiceCommand = '';
        this.sockets = [];
    }

    connectedCallback() {
        super.connectedCallback();
        this.connectConversation();
        this.connectVoice();
    }

    disconnectedCallback() {
        super.disconnectedCallback();
        for (const socket of this.sockets) {
            try {
                socket.close();
            } catch (_error) {
                // ignore
            }
        }
        this.sockets.length = 0;
    }

    connectConversation() {
        const socket = createTopicSocket({
            topic: '/conversation',
            type: 'psyched_msgs/msg/Message',
            role: 'subscribe',
        });
        socket.addEventListener('message', (event) => {
            const payload = JSON.parse(event.data);
            if (payload.event === 'message' && payload.data) {
                this.addMessage(payload.data);
                this.status = 'Live';
            }
        });
        socket.addEventListener('close', () => {
            this.status = 'Disconnected';
        });
        socket.addEventListener('error', () => {
            this.status = 'Error';
        });
        this.sockets.push(socket);

        this.conversationPublisher = createTopicSocket({
            topic: '/conversation',
            type: 'psyched_msgs/msg/Message',
            role: 'publish',
        });
        this.conversationPublisher.addEventListener('open', () => {
            this.formFeedback = '';
        });
        this.conversationPublisher.addEventListener('error', () => {
            this.formFeedback = 'Unable to publish to /conversation';
        });
        this.sockets.push(this.conversationPublisher);
    }

    connectVoice() {
        const socket = createTopicSocket({
            topic: '/voice',
            type: 'std_msgs/msg/String',
            role: 'subscribe',
        });
        socket.addEventListener('message', (event) => {
            const payload = JSON.parse(event.data);
            if (payload.event === 'message' && payload.data && typeof payload.data.data !== 'undefined') {
                this.voiceLast = String(payload.data.data ?? '');
                this.voiceStatus = 'Live';
            }
        });
        socket.addEventListener('close', () => {
            this.voiceStatus = 'Disconnected';
        });
        socket.addEventListener('error', () => {
            this.voiceStatus = 'Error';
        });
        this.sockets.push(socket);

        this.voicePublisher = createTopicSocket({
            topic: '/voice',
            type: 'std_msgs/msg/String',
            role: 'publish',
        });
        this.voicePublisher.addEventListener('open', () => {
            this.voiceFeedback = '';
        });
        this.voicePublisher.addEventListener('error', () => {
            this.voiceFeedback = 'Unable to publish to /voice';
        });
        this.sockets.push(this.voicePublisher);
    }

    addMessage(data) {
        const message = {
            id: generateId(),
            role: data.role || 'unknown',
            speaker: data.speaker || '',
            confidence: typeof data.confidence === 'number' ? data.confidence : null,
            content: data.content || '',
            timestamp: new Date().toLocaleTimeString(),
        };
        this.messages = [message, ...this.messages].slice(0, 50);
    }

    formatConfidence(value) {
        if (typeof value !== 'number' || Number.isNaN(value)) {
            return '—';
        }
        return `${Math.round(value * 100)}%`;
    }

    handleSendMessage(event) {
        event.preventDefault();
        const text = this.composerContent.trim();
        if (!text) {
            this.formFeedback = 'Message text required';
            return;
        }
        const payload = {
            role: this.composerRole || 'user',
            content: text,
            speaker: this.composerSpeaker || 'pilot',
            confidence: Number.isFinite(Number(this.composerConfidence)) ? Number(this.composerConfidence) : 1.0,
            segments: [],
            words: [],
        };
        try {
            this.conversationPublisher.send(JSON.stringify(payload));
            this.formFeedback = 'Message sent to /conversation';
            this.composerContent = '';
        } catch (error) {
            this.formFeedback = error instanceof Error ? error.message : String(error);
        }
    }

    handleSendVoice(event) {
        event.preventDefault();
        const text = this.voiceCommand.trim();
        if (!text) {
            this.voiceFeedback = 'Text required to publish to /voice';
            return;
        }
        const payload = { data: text };
        try {
            this.voicePublisher.send(JSON.stringify(payload));
            this.voiceFeedback = 'Voice command sent';
            this.voiceCommand = '';
            this.voiceLast = text;
        } catch (error) {
            this.voiceFeedback = error instanceof Error ? error.message : String(error);
        }
    }

    render() {
        const conversationStatus = this.status === 'Live' ? 'success' : this.status === 'Error' ? 'error' : undefined;
        const voiceStatusVariant = this.voiceStatus === 'Live' ? 'success' : this.voiceStatus === 'Error' ? 'error' : undefined;
        return html`
      <div class="surface-grid surface-grid--wide surface-grid--dense">
        <article class="surface-card surface-card--compact">
          <h3 class="surface-card__title">Conversation Stream</h3>
          <p class="surface-status" data-variant="${conversationStatus ?? ''}">Status: ${this.status}</p>
          <ul class="surface-list surface-list--scrollable conversation-log">
            ${this.messages.length === 0
                ? html`<li class="conversation-empty">Waiting for conversation activity…</li>`
                : this.messages.map(
                    (message) => html`<li class="conversation-entry" key="${message.id}">
                <div class="conversation-entry__meta">
                  <span class="conversation-entry__role">${message.role}</span>
                  <span>${message.speaker || '—'}</span>
                  <span>Conf: ${this.formatConfidence(message.confidence)}</span>
                  <span>${message.timestamp}</span>
                </div>
                <p class="conversation-entry__content">${message.content}</p>
              </li>`,
                )}
          </ul>
        </article>

        <article class="surface-card surface-card--compact">
          <h3 class="surface-card__title">Send to /conversation</h3>
          <form class="surface-form surface-form--compact" @submit=${this.handleSendMessage}>
            <label class="surface-field">
              <span class="surface-label">Role</span>
              <select
                class="surface-select"
                .value=${this.composerRole}
                @change=${(e) => (this.composerRole = e.target.value)}
              >
                <option value="user">user</option>
                <option value="assistant">assistant</option>
                <option value="system">system</option>
                <option value="pilot">pilot</option>
              </select>
            </label>
            <label class="surface-field">
              <span class="surface-label">Speaker</span>
              <input
                class="surface-input"
                type="text"
                .value=${this.composerSpeaker}
                @input=${(e) => (this.composerSpeaker = e.target.value)}
                placeholder="pilot"
              />
            </label>
            <label class="surface-field">
              <span class="surface-label">Confidence</span>
              <input
                class="surface-input surface-input--small"
                type="number"
                step="0.01"
                min="0"
                max="1"
                .value=${this.composerConfidence}
                @input=${(e) => (this.composerConfidence = parseFloat(e.target.value))}
              />
            </label>
            <label class="surface-field">
              <span class="surface-label">Message</span>
              <textarea
                class="surface-textarea"
                rows="4"
                .value=${this.composerContent}
                @input=${(e) => (this.composerContent = e.target.value)}
                placeholder="What would you like to say?"
              ></textarea>
            </label>
            <div class="surface-actions">
              <button class="surface-button" type="submit">Send</button>
            </div>
            ${this.formFeedback ? html`<p class="surface-status">${this.formFeedback}</p>` : ''}
          </form>
        </article>

        <article class="surface-card surface-card--compact">
          <h3 class="surface-card__title">Voice Bridge</h3>
          <p class="surface-status" data-variant="${voiceStatusVariant ?? ''}">Status: ${this.voiceStatus}</p>
          <div class="surface-panel surface-mono voice-log">${this.voiceLast || '—'}</div>
          <form class="surface-form surface-form--compact" @submit=${this.handleSendVoice}>
            <label class="surface-field">
              <span class="surface-label">Speak this text</span>
              <textarea
                class="surface-textarea"
                rows="3"
                .value=${this.voiceCommand}
                @input=${(e) => (this.voiceCommand = e.target.value)}
                placeholder="Hello from chat"
              ></textarea>
            </label>
            <div class="surface-actions">
              <button class="surface-button" type="submit">Send to /voice</button>
            </div>
            ${this.voiceFeedback ? html`<p class="surface-status">${this.voiceFeedback}</p>` : ''}
          </form>
        </article>
      </div>
    `;
    }
}

customElements.define('chat-dashboard', ChatDashboard);
