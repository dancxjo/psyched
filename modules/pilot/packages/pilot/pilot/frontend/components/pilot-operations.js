import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { surfaceStyles } from './pilot-style.js';

/**
 * Surface repository and provisioning helpers inside the pilot cockpit.
 *
 * @example
 *   <pilot-operations-panel></pilot-operations-panel>
 */
class PilotOperationsPanel extends LitElement {
  static properties = {
    operations: { state: true },
    loading: { state: true },
    errorMessage: { state: true },
    runningOperationId: { state: true },
    results: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      :host {
        display: block;
        margin-bottom: 2rem;
      }

      .surface-section__title {
        margin: 0;
        font-size: 1.35rem;
        text-transform: uppercase;
        letter-spacing: 0.12rem;
        color: var(--lcars-accent-secondary, #f7a400);
      }

      .pilot-operations__header {
        display: flex;
        flex-direction: column;
        gap: 0.25rem;
        margin-bottom: 1rem;
      }

      .pilot-operations__description {
        margin: 0;
        color: rgba(255, 255, 255, 0.75);
        font-size: 0.95rem;
        line-height: 1.4;
      }

      .pilot-operations__grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
        gap: 1rem;
      }

      .pilot-operations__card {
        display: flex;
        flex-direction: column;
        gap: 0.75rem;
        background: rgba(0, 0, 0, 0.35);
        border-radius: 0.75rem;
        padding: 1rem;
        box-shadow: 0 0 0 1px rgba(255, 255, 255, 0.08);
      }

      .pilot-operations__title {
        margin: 0;
      }

      .pilot-operations__command {
        margin: 0;
        font-size: 0.85rem;
        color: rgba(255, 255, 255, 0.7);
        font-family: var(
          --pilot-monospace-font,
          'SFMono-Regular',
          'Consolas',
          'Liberation Mono',
          monospace
        );
      }

      .pilot-operations__button {
        align-self: flex-start;
      }

      .pilot-operations__status {
        margin: 0;
        font-size: 0.85rem;
      }

      .pilot-operations__status[data-variant='ok'] {
        color: #8ce99a;
      }

      .pilot-operations__status[data-variant='error'] {
        color: #ffa8a8;
      }

      .pilot-operations__logs {
        margin: 0.5rem 0 0;
        padding: 0.5rem;
        border-radius: 0.5rem;
        background: rgba(0, 0, 0, 0.6);
        font-family: var(
          --pilot-monospace-font,
          'SFMono-Regular',
          'Consolas',
          'Liberation Mono',
          monospace
        );
        font-size: 0.75rem;
        line-height: 1.35;
        white-space: pre-wrap;
        word-break: break-word;
        max-height: 12rem;
        overflow: auto;
      }

      .pilot-operations__logs[data-variant='stderr'] {
        border: 1px solid rgba(255, 100, 100, 0.45);
      }

      .pilot-operations__empty,
      .pilot-operations__loading {
        margin: 0;
        font-size: 0.95rem;
        color: rgba(255, 255, 255, 0.75);
      }

      .pilot-operations__error {
        padding: 1rem;
        border-radius: 0.75rem;
        background: rgba(255, 77, 77, 0.25);
        display: flex;
        flex-direction: column;
        gap: 0.75rem;
      }

      .pilot-operations__error p {
        margin: 0;
      }

      details {
        background: rgba(255, 255, 255, 0.05);
        border-radius: 0.5rem;
        padding: 0.5rem 0.75rem;
      }
    `,
  ];

  constructor() {
    super();
    this.operations = [];
    this.loading = true;
    this.errorMessage = '';
    this.runningOperationId = '';
    this.results = {};
  }

  connectedCallback() {
    super.connectedCallback();
    this.loadOperations();
  }

  async loadOperations() {
    this.loading = true;
    this.errorMessage = '';
    try {
      const response = await fetch('/api/operations');
      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }
      const payload = await response.json();
      const operations = Array.isArray(payload.operations)
        ? payload.operations
        : [];
      this.operations = operations;
      this.results = {};
    } catch (error) {
      this.errorMessage = error instanceof Error ? error.message : String(error);
      this.operations = [];
    } finally {
      this.loading = false;
    }
  }

  async runOperation(operation) {
    if (!operation || !operation.id) {
      return;
    }
    this.runningOperationId = operation.id;
    const nextResults = { ...this.results };

    try {
      const response = await fetch('/api/operations', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ operation: operation.id }),
      });
      let payload;
      try {
        payload = await response.json();
      } catch (_error) {
        payload = {};
      }
      const status = typeof payload.status === 'string'
        ? payload.status
        : response.ok
        ? 'ok'
        : 'error';
      nextResults[operation.id] = {
        status,
        exitCode: typeof payload.exit_code === 'number' ? payload.exit_code : null,
        stdout: typeof payload.stdout === 'string' ? payload.stdout : '',
        stderr: typeof payload.stderr === 'string' ? payload.stderr : '',
        completedAt: new Date(),
      };
    } catch (error) {
      nextResults[operation.id] = {
        status: 'error',
        exitCode: null,
        stdout: '',
        stderr: error instanceof Error ? error.message : String(error),
        completedAt: new Date(),
      };
    } finally {
      this.results = nextResults;
      this.runningOperationId = '';
    }
  }

  render() {
    return html`
      <section class="pilot-operations">
        <header class="pilot-operations__header">
          <h2 class="surface-section__title">Operations</h2>
          <p class="pilot-operations__description">
            Trigger repository syncs and bulk provisioning commands without
            leaving the cockpit.
          </p>
        </header>
        ${this.renderBody()}
      </section>
    `;
  }

  renderBody() {
    if (this.loading) {
      return html`<p class="pilot-operations__loading">Loading operations…</p>`;
    }
    if (this.errorMessage) {
      return html`
        <div class="pilot-operations__error">
          <p>Failed to load operations: ${this.errorMessage}</p>
          <button type="button" @click=${() => this.loadOperations()}>
            Retry
          </button>
        </div>
      `;
    }
    if (!this.operations.length) {
      return html`
        <p class="pilot-operations__empty">
          No cockpit-triggered operations are available.
        </p>
      `;
    }
    return html`
      <div class="pilot-operations__grid">
        ${this.operations.map((operation) => this.renderOperation(operation))}
      </div>
    `;
  }

  renderOperation(operation) {
    const result = this.results?.[operation.id] || null;
    const command = Array.isArray(operation.command)
      ? operation.command.join(' ')
      : operation.command_preview || '';
    const busy = this.runningOperationId === operation.id;
    const variant = result?.status === 'ok' ? 'ok' : result?.status === 'error'
      ? 'error'
      : '';
    const timestamp = result?.completedAt
      ? this.formatTimestamp(result.completedAt)
      : '';

    return html`
      <article class="pilot-operations__card">
        <div>
          <h3 class="pilot-operations__title">${operation.label}</h3>
          ${command
            ? html`<p class="pilot-operations__command">${command}</p>`
            : ''}
        </div>
        ${operation.description
          ? html`<p class="pilot-operations__description">
              ${operation.description}
            </p>`
          : ''}
        <button
          type="button"
          class="pilot-operations__button"
          ?disabled=${busy}
          aria-busy=${busy ? 'true' : 'false'}
          @click=${() => this.runOperation(operation)}
        >
          ${busy ? 'Running…' : `Run ${operation.label}`}
        </button>
        ${result
          ? html`
              <div class="pilot-operations__result">
                <p
                  class="pilot-operations__status"
                  data-variant=${variant}
                  aria-live="polite"
                >
                  ${this.describeResult(result, timestamp)}
                </p>
                ${this.renderLogs(result)}
              </div>
            `
          : ''}
      </article>
    `;
  }

  renderLogs(result) {
    const stdout = (result.stdout || '').trim();
    const stderr = (result.stderr || '').trim();
    if (!stdout && !stderr) {
      return null;
    }
    return html`
      <details>
        <summary>View logs</summary>
        ${stdout
          ? html`<pre class="pilot-operations__logs">${stdout}</pre>`
          : ''}
        ${stderr
          ? html`<pre
              class="pilot-operations__logs"
              data-variant="stderr"
            >${stderr}</pre>`
          : ''}
      </details>
    `;
  }

  describeResult(result, timestamp) {
    const suffix = timestamp ? ` at ${timestamp}` : '';
    if (result.status === 'ok') {
      return `Completed successfully${suffix}`;
    }
    const code = typeof result.exitCode === 'number' ? ` (exit ${result.exitCode})` : '';
    return `Failed${code}${suffix}`;
  }

  formatTimestamp(date) {
    try {
      return date.toLocaleTimeString();
    } catch (_error) {
      return '';
    }
  }
}

customElements.define('pilot-operations-panel', PilotOperationsPanel);
