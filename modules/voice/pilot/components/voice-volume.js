import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { extractNumeric } from '/utils/metrics.js';
import { updateVoiceVolume, setVoiceVolume } from '../utils/voice.js';

/**
 * Volume slider for the voice synthesiser.
 */
class PilotVoiceVolume extends LitElement {
  static properties = {
    record: { type: Object },
    _value: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this._value = 50;
  }

  createRenderRoot() {
    return this;
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    updateVoiceVolume(null);
  }

  updated(changed) {
    if (changed.has('record') && this.record) {
      const next = extractNumeric(this.record.last, this._value);
      if (Number.isFinite(next)) {
        this._value = Math.max(0, Math.min(100, Math.round(next)));
      }
      if (this.record.send) {
        updateVoiceVolume({
          send: (message) => {
            try {
              this.record.send(message);
            } catch (error) {
              console.warn('Failed to publish voice volume', error);
            }
          },
          state: this.record.state ?? 'idle',
          value: this._value,
        });
      }
    }
  }

  handleInput(event) {
    const value = event.target?.valueAsNumber;
    if (!Number.isFinite(value)) {
      return;
    }
    this._value = Math.max(0, Math.min(100, Math.round(value)));
    this.requestUpdate();
  }

  handleCommit(event) {
    const value = event.target?.valueAsNumber;
    if (!Number.isFinite(value)) {
      return;
    }
    setVoiceVolume(value);
  }

  render() {
    return html`
      <div class="voice-volume" data-state=${this.record?.state ?? 'idle'}>
        <label for="voice-volume-slider">Output volume</label>
        <input
          id="voice-volume-slider"
          type="range"
          min="0"
          max="100"
          .value=${String(this._value)}
          @input=${(event) => this.handleInput(event)}
          @change=${(event) => this.handleCommit(event)}
        />
        <span class="volume-value">${this._value}</span>
      </div>
    `;
  }
}

customElements.define('pilot-voice-volume', PilotVoiceVolume);
