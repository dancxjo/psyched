import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

/**
 * Simple text console for publishing std_msgs/String payloads to /voice.
 */
class PilotVoiceConsole extends LitElement {
  static properties = {
    record: { type: Object },
    _input: { state: true },
    _isSending: { state: true },
    _lastSent: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this._input = '';
    this._isSending = false;
    this._lastSent = null;
  }

  createRenderRoot() {
    return this;
  }

  updateInput(value) {
    this._input = value;
    this.requestUpdate();
  }

  handleInput(event) {
    this.updateInput(event.target.value ?? '');
  }

  async handleSubmit(event) {
    event?.preventDefault?.();
    const message = (this._input || '').trim();
    if (!message || !this.record?.send) {
      return;
    }
    try {
      this._isSending = true;
      this.record.send({ data: message });
      this._lastSent = message;
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

  render() {
    const disabled = this._isSending || this.record?.state !== 'connected';
    return html`
      <form class="voice-console" @submit=${(event) => this.handleSubmit(event)}>
        <label for="voice-input">Say something</label>
        <textarea
          id="voice-input"
          rows="3"
          placeholder="Type speech text…"
          .value=${this._input}
          ?disabled=${disabled}
          @input=${(event) => this.handleInput(event)}
          @keydown=${(event) => this.handleKeyDown(event)}
        ></textarea>
        <div class="voice-actions">
          <button type="submit" ?disabled=${disabled || !this._input.trim()}>Send</button>
          ${this._lastSent
            ? html`<span class="voice-last">Last sent: ${this._lastSent}</span>`
            : html`<span class="voice-hint">Connected clients will synthesize the text.</span>`}
        </div>
      </form>
    `;
  }
}

customElements.define('pilot-voice-console', PilotVoiceConsole);
