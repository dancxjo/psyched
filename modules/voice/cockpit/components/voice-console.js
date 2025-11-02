import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { subscribeVoiceControls, publishVoiceAction, setVoiceVolume } from '../utils/voice.js';

/**
 * Simple text console for publishing std_msgs/String payloads to /voice.
 */
class CockpitVoiceConsole extends LitElement {
  static properties = {
    record: { type: Object },
    _input: { state: true },
    _isSending: { state: true },
    _lastSent: { state: true },
    _controls: { state: true },
    _volume: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this._input = '';
    this._isSending = false;
    this._lastSent = null;
    this._controls = {};
    this._volume = 50;
    this._unsubscribe = null;
  }

  createRenderRoot() {
    return this;
  }

  connectedCallback() {
    super.connectedCallback();
    this._unsubscribe = subscribeVoiceControls((snapshot) => {
      this._controls = snapshot.actions ?? {};
      const descriptor = snapshot.volume;
      if (descriptor && Number.isFinite(descriptor.value)) {
        this._volume = Math.max(0, Math.min(100, Math.round(descriptor.value)));
      }
      this.requestUpdate();
    });
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    if (typeof this._unsubscribe === 'function') {
      this._unsubscribe();
    }
    this._unsubscribe = null;
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

  handleTransport(action) {
    publishVoiceAction(action);
  }

  handleVolumeInput(event) {
    const value = event.target?.valueAsNumber;
    if (!Number.isFinite(value)) {
      return;
    }
    this._volume = Math.max(0, Math.min(100, Math.round(value)));
    this.requestUpdate();
  }

  handleVolumeCommit(event) {
    const value = event.target?.valueAsNumber;
    if (!Number.isFinite(value)) {
      return;
    }
    setVoiceVolume(value);
  }

  renderTransportControls() {
    const actions = this._controls;
    const buttons = [
      { action: 'interrupt', label: '⏸️ Pause', variant: 'ghost' },
      { action: 'resume', label: '▶️ Resume', variant: 'accent' },
      { action: 'clear', label: '🧹 Clear', variant: 'critical' },
    ];
    return html`
      <div class="voice-transport">
        ${buttons.map(({ action, label, variant }) => {
          const available = actions?.[action]?.available;
          return html`<button
            type="button"
            class="control-button"
            data-variant=${variant}
            ?disabled=${!available}
            @click=${() => this.handleTransport(action)}
          >
            ${label}
          </button>`;
        })}
      </div>
    `;
  }

  renderVolumeControl() {
    return html`
      <div class="voice-volume-inline">
        <label for="voice-volume-inline-slider">Volume</label>
        <input
          id="voice-volume-inline-slider"
          type="range"
          min="0"
          max="100"
          .value=${String(this._volume)}
          @input=${(event) => this.handleVolumeInput(event)}
          @change=${(event) => this.handleVolumeCommit(event)}
        />
        <span class="value">${this._volume}</span>
      </div>
    `;
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
          <button
            type="submit"
            class="control-button"
            data-variant="accent"
            ?disabled=${disabled || !this._input.trim()}
          >
            🗣️ Send
          </button>
          ${this._lastSent
            ? html`<span class="voice-last">Last sent: ${this._lastSent}</span>`
            : html`<span class="voice-hint">Connected clients will synthesize the text.</span>`}
        </div>
        ${this.renderTransportControls()}
        ${this.renderVolumeControl()}
      </form>
    `;
  }
}

customElements.define('cockpit-voice-console', CockpitVoiceConsole);
