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

  _resolveText(entry) {
    if (!entry) {
      return '';
    }
    const segments = Array.isArray(entry.segments) ? entry.segments : null;
    if (segments?.length) {
      const pieces = segments
        .map((segment) => (typeof segment?.text === 'string' ? segment.text.trim() : ''))
        .filter((piece) => piece.length > 0);
      if (pieces.length) {
        return pieces.join(' ');
      }
    }
    const text = entry.text;
    if (typeof text === 'string') {
      const trimmed = text.trim();
      if ((trimmed.startsWith('{') || trimmed.startsWith('['))) {
        try {
          const parsed = JSON.parse(trimmed);
          if (Array.isArray(parsed)) {
            const parts = parsed
              .map((segment) => (typeof segment?.text === 'string' ? segment.text.trim() : ''))
              .filter((piece) => piece.length > 0);
            if (parts.length) {
              return parts.join(' ');
            }
          } else if (parsed && typeof parsed === 'object') {
            const nestedSegments = Array.isArray(parsed.segments) ? parsed.segments : [];
            if (nestedSegments.length) {
              const parts = nestedSegments
                .map((segment) => (typeof segment?.text === 'string' ? segment.text.trim() : ''))
                .filter((piece) => piece.length > 0);
              if (parts.length) {
                return parts.join(' ');
              }
            }
            if (typeof parsed.text === 'string' && parsed.text.trim()) {
              return parsed.text.trim();
            }
          }
        } catch (error) {
          // If parsing fails we fall back to the raw string below.
        }
      }
      return trimmed;
    }
    if (typeof text === 'number') {
      return String(text);
    }
    if (text && typeof text === 'object') {
      return JSON.stringify(text);
    }
    return JSON.stringify(entry ?? {});
  }

  _renderEntry(entry, index) {
    const text = this._resolveText(entry);
    const speaker = entry?.speaker ? String(entry.speaker) : 'unknown';
    const confidence = typeof entry?.confidence === 'number' ? entry.confidence : null;
    const segments = Array.isArray(entry?.segments) ? entry.segments : [];
    const words = Array.isArray(entry?.words) ? entry.words : [];

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
        ${this._renderSegments(segments)}
        ${this._renderWords(words)}
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
