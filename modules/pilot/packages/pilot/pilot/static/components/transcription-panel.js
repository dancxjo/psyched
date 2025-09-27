import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

/**
 * Display streaming transcription results with speaker and confidence badges.
 */
class PilotTranscriptionPanel extends LitElement {
  static properties = {
    record: { type: Object },
  };

  constructor() {
    super();
    this.record = null;
  }

  createRenderRoot() {
    return this;
  }

  get entries() {
    if (Array.isArray(this.record?.messages)) {
      return this.record.messages;
    }
    if (this.record?.last) {
      return [this.record.last];
    }
    return [];
  }

  _renderEntry(entry, index) {
    const text = typeof entry?.text === 'string' ? entry.text : JSON.stringify(entry ?? {});
    const speaker = entry?.speaker ? String(entry.speaker) : 'unknown';
    const confidence = typeof entry?.confidence === 'number' ? entry.confidence : null;

    return html`
      <li class="transcription-entry">
        <header>
          <span class="badge index">#${index + 1}</span>
          <span class="badge speaker">${speaker}</span>
          ${confidence !== null
            ? html`<span class="badge confidence">${Math.round(confidence * 100)}%</span>`
            : nothing}
        </header>
        <p>${text}</p>
      </li>
    `;
  }

  render() {
    if (!this.record) {
      return html`<div class="inactive">Not subscribed</div>`;
    }
    const entries = this.entries;
    if (!entries.length) {
      return html`<div class="transcription-empty">Awaiting transcripts…</div>`;
    }
    return html`
      <div class="transcription-panel">
        <ol>
          ${entries.map((entry, index) => this._renderEntry(entry, index))}
        </ol>
      </div>
    `;
  }
}

customElements.define('pilot-transcription-panel', PilotTranscriptionPanel);
