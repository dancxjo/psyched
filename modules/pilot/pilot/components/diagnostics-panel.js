import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { extractText } from '/components/utils/metrics.js';

/**
 * Present diagnostics output inside a collapsible details element.
 */
class PilotDiagnosticsPanel extends LitElement {
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
    const text = extractText(this.record?.last ?? this.record?.messages?.[0]);
    return html`
      <details class="diagnostics-panel" ?open=${Boolean(text)}>
        <summary>Diagnostics</summary>
        <pre>${text || 'Awaiting diagnostic messages…'}</pre>
      </details>
    `;
  }
}

customElements.define('pilot-diagnostics-panel', PilotDiagnosticsPanel);
