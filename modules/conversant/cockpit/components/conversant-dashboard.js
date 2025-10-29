import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';

/**
 * Dashboard for monitoring and nudging Pete's Conversant module.
 */
class ConversantDashboard extends LitElement {
  static properties = {
    topic: { state: true },
    topicStatus: { state: true },
    topicFeedback: { state: true },
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
    `,
  ];

  constructor() {
    super();
    this.topic = '';
    this.topicStatus = 'Connecting…';
    this.topicFeedback = '';
    this.directMessage = '';
    this.directFeedback = '';
    this.threadHint = '';
    this._topicSocket = null;
    this._concernPublisher = null;
    this._sockets = [];
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
  }

  _openTopicFeed() {
    const socket = createTopicSocket({
      module: 'conversant',
      topic: '/conversant/topic',
      type: 'std_msgs/msg/String',
      role: 'subscribe',
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
      const data = payload.data && typeof payload.data.data !== 'undefined' ? payload.data.data : '';
      this.topic = String(data || '');
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
    const threadId = this.threadHint.trim();
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
    this.threadHint = '';
    this.directFeedback = '';
  }

  render() {
    const topicVariant = this.topicStatus === 'Live' ? 'success' : this.topicStatus === 'Error' ? 'error' : '';
    return html`
      <div class="surface-grid surface-grid--wide surface-grid--dense">
        <article class="surface-card surface-card--compact">
          <h3 class="surface-card__title">Conversation stream</h3>
          <p class="surface-status" data-variant=${topicVariant}>Stream feed: ${this.topicStatus}</p>
          <label class="surface-field">
            <span class="surface-label">Current stream</span>
            <input class="surface-input" type="text" .value=${this.topic} readonly />
          </label>
          ${this.topicFeedback
            ? html`<p class="surface-status" data-variant="error">${this.topicFeedback}</p>`
            : ''}
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
              <textarea
                class="surface-textarea"
                name="message"
                .value=${this.directMessage}
                placeholder="e.g. Please acknowledge the battery check"
                @input=${(event) => (this.directMessage = event.target.value)}
              ></textarea>
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
