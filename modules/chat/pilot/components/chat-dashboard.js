import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';

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

    static styles = css`
    :host {
      display: block;
    }
    .chat-layout {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 1rem;
    }
    .chat-card {
      background: var(--control-surface-bg);
      border: 1px solid var(--control-surface-border);
      border-radius: var(--control-surface-radius);
      padding: var(--control-surface-padding);
      box-shadow: var(--control-surface-shadow);
      display: flex;
      flex-direction: column;
      gap: 0.75rem;
    }
    .chat-card h3 {
      margin: 0;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      font-size: 0.9rem;
      color: var(--metric-title-color);
    }
    .status {
      font-size: 0.8rem;
      color: var(--lcars-muted);
    }
    .status strong {
      color: var(--lcars-text);
    }
    .conversation-log {
      list-style: none;
      margin: 0;
      padding: 0;
      display: flex;
      flex-direction: column;
      gap: 0.5rem;
      max-height: 300px;
      overflow-y: auto;
    }
    .conversation-log li {
      background: rgba(0, 0, 0, 0.3);
      border: 1px solid var(--control-surface-border);
      border-radius: 0.5rem;
      padding: 0.5rem;
      display: flex;
      flex-direction: column;
      gap: 0.25rem;
    }
    .conversation-log .meta {
      display: flex;
      flex-wrap: wrap;
      gap: 0.5rem;
      font-size: 0.75rem;
      color: var(--lcars-muted);
    }
    .conversation-log .role {
      font-weight: 600;
      color: var(--lcars-accent-secondary);
    }
    .conversation-log .content {
      margin: 0;
      white-space: pre-wrap;
      line-height: 1.3;
      font-size: 0.85rem;
    }
    .conversation-empty {
      color: var(--lcars-muted);
      font-style: italic;
      text-align: center;
      font-size: 0.85rem;
    }
    form {
      display: flex;
      flex-direction: column;
      gap: 0.5rem;
    }
    label {
      display: flex;
      flex-direction: column;
      gap: 0.25rem;
      font-size: 0.75rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      color: var(--metric-label-color);
    }
    input, select, textarea {
      font: inherit;
      padding: 0.5rem;
      border-radius: 0.5rem;
      border: 1px solid var(--control-surface-border);
      background: rgba(0, 0, 0, 0.3);
      color: var(--lcars-text);
      font-family: var(--metric-value-font);
    }
    textarea {
      resize: vertical;
      min-height: 80px;
      font-size: 0.85rem;
    }
    button {
      align-self: flex-start;
      padding: 0.5rem 1rem;
      border-radius: 999px;
      background: var(--lcars-accent-secondary);
      color: #05070d;
      border: none;
      font-weight: 600;
      cursor: pointer;
      font-size: 0.8rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      transition: background 120ms ease;
    }
    button:hover {
      background: var(--lcars-accent);
    }
    pre {
      background: rgba(0, 0, 0, 0.3);
      border: 1px solid var(--control-surface-border);
      border-radius: 0.5rem;
      padding: 0.5rem;
      min-height: 3rem;
      overflow-wrap: anywhere;
      font-family: var(--metric-value-font);
      font-size: 0.8rem;
      margin: 0;
    }
  `;

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
        return html`
      <div class="chat-layout">
        <article class="chat-card">
          <h3>Conversation Stream</h3>
          <p class="status">Status: <strong>${this.status}</strong></p>
          <ul class="conversation-log">
            ${this.messages.length === 0
                ? html`<li class="conversation-empty">Waiting for conversation activity…</li>`
                : this.messages.map(
                    (message) => html`
                    <li key="${message.id}">
                      <div class="meta">
                        <span class="role">${message.role}</span>
                        <span class="speaker">${message.speaker || '—'}</span>
                        <span>Conf: ${this.formatConfidence(message.confidence)}</span>
                        <span>${message.timestamp}</span>
                      </div>
                      <p class="content">${message.content}</p>
                    </li>
                  `,
                )}
          </ul>
        </article>

        <article class="chat-card">
          <h3>Send to /conversation</h3>
          <form @submit=${this.handleSendMessage}>
            <label>
              Role
              <select .value=${this.composerRole} @change=${(e) => (this.composerRole = e.target.value)}>
                <option value="user">user</option>
                <option value="assistant">assistant</option>
                <option value="system">system</option>
                <option value="pilot">pilot</option>
              </select>
            </label>
            <label>
              Speaker
              <input type="text" .value=${this.composerSpeaker} @input=${(e) => (this.composerSpeaker = e.target.value)} placeholder="pilot" />
            </label>
            <label>
              Confidence
              <input type="number" step="0.01" min="0" max="1" .value=${this.composerConfidence} @input=${(e) => (this.composerConfidence = parseFloat(e.target.value))} />
            </label>
            <label>
              Message
              <textarea rows="4" .value=${this.composerContent} @input=${(e) => (this.composerContent = e.target.value)} placeholder="What would you like to say?"></textarea>
            </label>
            <button type="submit">Send</button>
            ${this.formFeedback ? html`<p class="status">${this.formFeedback}</p>` : ''}
          </form>
        </article>

        <article class="chat-card">
          <h3>Voice Bridge</h3>
          <p class="status">Status: <strong>${this.voiceStatus}</strong></p>
          <label style="margin-top: 0.5rem;">Latest /voice
            <pre>${this.voiceLast || '—'}</pre>
          </label>
          <form @submit=${this.handleSendVoice}>
            <label>
              Speak this text
              <textarea rows="3" .value=${this.voiceCommand} @input=${(e) => (this.voiceCommand = e.target.value)} placeholder="Hello from chat"></textarea>
            </label>
            <button type="submit">Send to /voice</button>
            ${this.voiceFeedback ? html`<p class="status">${this.voiceFeedback}</p>` : ''}
          </form>
        </article>
      </div>
    `;
    }
}

customElements.define('chat-dashboard', ChatDashboard);
