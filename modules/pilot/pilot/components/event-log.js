import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { extractText } from '/components/utils/metrics.js';

/**
 * Display a rolling list of event payloads.
 */
class PilotEventLog extends LitElement {
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

  render() {
    if (!this.record) {
      return html`<div class="event-log inactive">Awaiting events…</div>`;
    }
    const messages = Array.isArray(this.record.messages) ? this.record.messages : [];
    if (!messages.length) {
      return html`<div class="event-log empty">Awaiting events…</div>`;
    }
    return html`
      <ol class="event-log">
        ${messages.slice(0, 5).map((entry, index) => {
          const label = `Event ${messages.length - index}`;
          const text = extractText(entry);
          return html`<li><strong>${label}:</strong> ${text}</li>`;
        })}
      </ol>
    `;
  }
}

customElements.define('pilot-event-log', PilotEventLog);
