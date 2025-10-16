import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

/**
 * Conversation console that shows recent dialogue and lets operators seed user turns.
 *
 * The console mirrors the `/conversation` topic so that only spoken assistant
 * turns (confirmed via `voice_done`) appear in the log. Operators can inject
 * additional user prompts for debugging or recovery without bypassing the
 * voice confirmation requirement.
 */
class CockpitConversationConsole extends LitElement {
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

  _formatSeconds(value) {
    const number = Number(value);
    if (!Number.isFinite(number)) {
      return '–';
    }
    return `${number.toFixed(2)}s`;
  }

  _renderSegments(segments) {
    if (!Array.isArray(segments) || !segments.length) {
      return nothing;
    }
    return html`
      <table class="timing-table segments">
        <caption>Segments</caption>
        <thead>
          <tr>
            <th scope="col">#</th>
            <th scope="col">Start</th>
            <th scope="col">End</th>
            <th scope="col">Speaker</th>
            <th scope="col">Text</th>
          </tr>
        </thead>
        <tbody>
          ${segments.map((segment, index) => {
            const start = this._formatSeconds(segment?.start);
            const end = this._formatSeconds(segment?.end);
            const speaker = segment?.speaker ? String(segment.speaker) : '—';
            const text = typeof segment?.text === 'string' ? segment.text.trim() : '';
            return html`
              <tr>
                <th scope="row">${index + 1}</th>
                <td>${start}</td>
                <td>${end}</td>
                <td>${speaker}</td>
                <td>${text}</td>
              </tr>
            `;
          })}
        </tbody>
      </table>
    `;
  }

  _renderWords(words) {
    if (!Array.isArray(words) || !words.length) {
      return nothing;
    }
    return html`
      <table class="timing-table words">
        <caption>Word timings</caption>
        <thead>
          <tr>
            <th scope="col">#</th>
            <th scope="col">Start</th>
            <th scope="col">End</th>
            <th scope="col">Word</th>
          </tr>
        </thead>
        <tbody>
          ${words.map((word, index) => {
            const start = this._formatSeconds(word?.start);
            const end = this._formatSeconds(word?.end);
            const text = typeof word?.text === 'string' ? word.text : JSON.stringify(word ?? {});
            return html`
              <tr>
                <th scope="row">${index + 1}</th>
                <td>${start}</td>
                <td>${end}</td>
                <td>${text}</td>
              </tr>
            `;
          })}
        </tbody>
      </table>
    `;
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

  updated(changedProperties) {
    // If the conversation log was scrolled to (or very near) the bottom,
    // keep it pinned to the bottom when new messages arrive. If the user
    // has scrolled up, do not force-scroll.
    try {
      const container = this.querySelector('.conversation-log');
      if (!container) return;
      // distance from bottom in pixels
      const distanceFromBottom = container.scrollHeight - container.scrollTop - container.clientHeight;
      const NEAR_BOTTOM_PX = 40; // tolerance
      const wasNearBottom = distanceFromBottom <= NEAR_BOTTOM_PX;
      if (wasNearBottom) {
        // Wait for layout to finish then jump to bottom
        requestAnimationFrame(() => {
          container.scrollTop = container.scrollHeight;
        });
      }
    } catch (err) {
      // keep silent on errors to avoid breaking the UI
      // console.error(err);
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
      const segments = Array.isArray(entry?.segments) ? entry.segments : [];
      const words = Array.isArray(entry?.words) ? entry.words : [];
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
          ${this._renderSegments(segments)}
          ${this._renderWords(words)}
        </li>
      `;
    });
  }

  render() {
    const disabled = this._isSending || this.record?.state !== 'connected';
    return html`
      <div class="conversation-console">
        <div class="conversation-log">
          <h5 class="audio-oscilloscope">Conversation Log</h5>
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
            <div class="conversation-speaker-group">
              <label for="conversation-speaker">Speaker</label>
              <input
                id="conversation-speaker"
                type="text"
                class="conversation-speaker"
                placeholder="Default operator"
                .value=${this._speaker}
                ?disabled=${disabled}
                @input=${(event) => this.handleSpeaker(event)}
              />
            </div>
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

customElements.define('cockpit-conversation-console', CockpitConversationConsole);
