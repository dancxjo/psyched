import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

/**
 * Conversation console that shows recent dialogue and lets operators seed user turns.
 *
 * The console mirrors the `/conversation` topic so that only spoken assistant
 * turns (confirmed via `voice_done`) appear in the log. Operators can inject
 * additional user prompts for debugging or recovery without bypassing the
 * voice confirmation requirement.
 */
class PilotConversationConsole extends LitElement {
  static properties = {
    record: { type: Object },
    _input: { state: true },
    _speaker: { state: true },
    _isSending: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this._input = '';
    this._speaker = '';
    this._isSending = false;
  }

  createRenderRoot() {
    return this;
  }

  get history() {
    const messages = Array.isArray(this.record?.messages) ? this.record.messages : [];
    return [...messages].reverse();
  }

  updateInput(value) {
    this._input = value;
    this.requestUpdate();
  }

  updateSpeaker(value) {
    this._speaker = value;
    this.requestUpdate();
  }

  handleInput(event) {
    this.updateInput(event.target.value ?? '');
  }

  handleSpeaker(event) {
    this.updateSpeaker(event.target.value ?? '');
  }

  async handleSubmit(event) {
    event?.preventDefault?.();
    const message = (this._input || '').trim();
    if (!message || !this.record?.send) {
      return;
    }
    const speaker = (this._speaker || 'operator').trim();
    try {
      this._isSending = true;
      this.record.send({
        role: 'user',
        content: message,
        speaker,
        confidence: 1.0,
      });
      this.updateInput('');
    } finally {
      this._isSending = false;
      this.requestUpdate();
    }
  }

  handleKeyDown(event) {
    if (event.key === 'Enter' && !event.shiftKey) {
      event.preventDefault();
      this.handleSubmit(event);
    }
  }

  renderHistory() {
    const entries = this.history;
    if (!entries.length) {
      return html`<li class="conversation-empty">No conversation yet.</li>`;
    }
    return entries.map((entry, index) => {
      const role = (entry?.role || 'unknown').toLowerCase();
      const speaker = entry?.speaker ? String(entry.speaker) : '';
      const confidence = typeof entry?.confidence === 'number' ? entry.confidence : null;
      const content = typeof entry?.content === 'string' ? entry.content : JSON.stringify(entry?.content ?? '');
      return html`
        <li class=${`conversation-entry ${role}`}>
          <header>
            <span class="badge role">${role}</span>
            ${speaker ? html`<span class="badge speaker">${speaker}</span>` : nothing}
            ${confidence !== null
              ? html`<span class="badge confidence">${Math.round(confidence * 100)}%</span>`
              : nothing}
            <span class="badge index">#${index + 1}</span>
          </header>
          <pre>${content}</pre>
        </li>
      `;
    });
  }

  render() {
    const disabled = this._isSending || this.record?.state !== 'connected';
    return html`
      <div class="conversation-console">
        <div class="conversation-log">
          <h3>Conversation</h3>
          <ul>
            ${this.renderHistory()}
          </ul>
        </div>
        <form class="conversation-form" @submit=${(event) => this.handleSubmit(event)}>
          <label for="conversation-input">Send user prompt</label>
          <textarea
            id="conversation-input"
            rows="3"
            placeholder="Type a user message…"
            .value=${this._input}
            ?disabled=${disabled}
            @input=${(event) => this.handleInput(event)}
            @keydown=${(event) => this.handleKeyDown(event)}
          ></textarea>
          <div class="conversation-controls">
            <input
              type="text"
              class="conversation-speaker"
              placeholder="Speaker (default operator)"
              .value=${this._speaker}
              ?disabled=${disabled}
              @input=${(event) => this.handleSpeaker(event)}
            />
            <button
              type="submit"
              class="control-button"
              data-variant="accent"
              ?disabled=${disabled || !this._input.trim()}
            >
              Send
            </button>
          </div>
        </form>
      </div>
    `;
  }
}

customElements.define('pilot-conversation-console', PilotConversationConsole);
