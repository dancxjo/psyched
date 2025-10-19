import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { unsafeHTML } from 'https://unpkg.com/lit@3.1.4/directives/unsafe-html.js?module';
import { AnsiUp } from 'https://esm.sh/ansi_up@6.0.2';
import { surfaceStyles } from './cockpit-style.js';

/**
 * Compact module log viewer that surfaces the tail of a module's log file.
 *
 * The component fetches log entries from the cockpit backend and renders the
 * newest lines inside a scrollable panel. Operators can trigger manual
 * refreshes which abort any in-flight requests to keep the UI responsive.
 *
 * Example:
 * ```html
 * <cockpit-module-logs module="nav"></cockpit-module-logs>
 * ```
 */
class CockpitModuleLogs extends LitElement {
  static properties = {
    module: { type: String, reflect: true },
    lines: { state: true },
    truncated: { state: true },
    loading: { state: true },
    clearing: { state: true },
    errorMessage: { state: true },
    updatedAt: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      :host {
        display: block;
      }

      .surface-log__header {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.75rem;
      }

      .surface-log__meta {
        font-size: 0.75rem;
        color: var(--lcars-muted);
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
      }

      .surface-log__content {
        margin: 0;
        padding: 0.75rem 1rem;
        background: rgba(0, 0, 0, 0.45);
        border-radius: 0.5rem;
        max-height: 240px;
        overflow: auto;
        font-family: var(--metric-value-font);
        font-size: 0.65rem;
        line-height: 1.3;
        white-space: pre-wrap;
        word-break: break-word;
      }

      .surface-log__empty {
        font-size: 0.85rem;
        color: var(--lcars-muted);
        font-style: italic;
      }

      .surface-log__actions {
        display: flex;
        justify-content: flex-end;
        gap: 0.5rem;
        flex-wrap: wrap;
      }

      .surface-log__details {
        margin-top: 0.5rem;
      }

      .surface-log__summary {
        cursor: pointer;
        font-size: 0.85rem;
        font-weight: 600;
        color: var(--lcars-link, #f8c77c);
        display: inline-flex;
        align-items: center;
        gap: 0.35rem;
      }
    `,
  ];

  constructor() {
    super();
    this.module = '';
    this.lines = [];
    this.truncated = false;
    this.loading = false;
    this.clearing = false;
    this.errorMessage = '';
    this.updatedAt = null;
    this._connected = false;
    this._abortController = null;
    this._lastFetchedModule = null;
    this._ansi = new AnsiUp();
  }

  connectedCallback() {
    super.connectedCallback();
    this._connected = true;
    if (this.module && this._lastFetchedModule !== this.module) {
      this.refresh();
    }
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._connected = false;
    this._abortFetch();
  }

  updated(changedProperties) {
    if (!this._connected) {
      return;
    }
    if (changedProperties.has('module')) {
      if (!this.module) {
        this.lines = [];
        this.truncated = false;
        this.updatedAt = null;
        this.errorMessage = '';
        this._lastFetchedModule = null;
        return;
      }
      if (this.module && this.module !== this._lastFetchedModule) {
        this.refresh();
      }
    }
  }

  render() {
    return html`
      <article class="surface-card surface-card--compact">
        <div class="surface-log__header">
          <h3 class="surface-card__title">Module log</h3>
          <div class="surface-log__actions">
            <button
              type="button"
              class="surface-action"
              ?disabled=${this.loading || this.clearing || !this.module}
              @click=${() => this._clearLogs()}
            >
              ${this.clearing ? 'Clearing…' : 'Clear log'}
            </button>
            <button
              type="button"
              class="surface-action"
              ?disabled=${this.loading || this.clearing || !this.module}
              @click=${() => this.refresh()}
            >
              ${this.loading ? 'Refreshing…' : 'Refresh log'}
            </button>
          </div>
        </div>
        ${this._renderStatus()}
      </article>
    `;
  }

  _renderStatus() {
    if (!this.module) {
      return html`<p class="surface-status" data-variant="warning">Module not specified.</p>`;
    }
    if (this.errorMessage) {
      return html`<p class="surface-status" data-variant="error">${this.errorMessage}</p>`;
    }
    if (this.loading && !this.lines.length) {
      return html`<p class="surface-status">Loading log…</p>`;
    }
    if (!this.lines.length) {
      return html`<p class="surface-log__empty">No log entries captured yet.</p>`;
    }
    const ansiText = this.lines.join('\n');
    // Convert ANSI codes to HTML, then remove <br/> tags since we're using <pre>
    const htmlContent = ansiText ? this._ansi.ansi_to_html(ansiText).replace(/<br\s*\/?>/gi, '\n') : '';
    return html`
      <div class="surface-log__meta">
        <span>Showing ${this.lines.length} line${this.lines.length === 1 ? '' : 's'}.</span>
        ${this.truncated
          ? html`<span>Older entries truncated.</span>`
          : ''}
        ${this.updatedAt
          ? html`<span>Updated ${this._formatTimestamp(this.updatedAt)}</span>`
          : html`<span>Awaiting first log write.</span>`}
      </div>
      <details class="surface-log__details">
        <summary class="surface-log__summary">View module log</summary>
        <pre class="surface-log__content">${unsafeHTML(htmlContent)}</pre>
      </details>
    `;
  }

  async refresh() {
    if (!this.module) {
      return;
    }
    this._abortFetch();
    const controller = new AbortController();
    this._abortController = controller;
    this._lastFetchedModule = this.module;
    this.loading = true;
    this.errorMessage = '';

    try {
      const response = await fetch(`/api/modules/${encodeURIComponent(this.module)}/logs`, {
        signal: controller.signal,
      });
      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }
      const payload = await response.json();
      this.lines = Array.isArray(payload.lines) ? payload.lines : [];
      this.truncated = Boolean(payload.truncated);
      this.updatedAt = typeof payload.updated_at === 'string' ? payload.updated_at : null;
    } catch (error) {
      if (controller.signal.aborted) {
        return;
      }
      const message = error instanceof Error ? error.message : String(error);
      this.errorMessage = `Failed to load module log: ${message}`;
      this.lines = [];
      this.truncated = false;
      this.updatedAt = null;
    } finally {
      if (this._abortController === controller) {
        this._abortController = null;
      }
      this.loading = false;
    }
  }

  async _clearLogs() {
    if (!this.module || this.clearing) {
      return;
    }
    this._abortFetch();
    this.clearing = true;
    this.errorMessage = '';

    try {
      const response = await fetch(`/api/modules/${encodeURIComponent(this.module)}/logs`, {
        method: 'DELETE',
      });
      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }
      this.lines = [];
      this.truncated = false;
      this.updatedAt = null;
      await this.refresh();
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.errorMessage = `Failed to clear module log: ${message}`;
    } finally {
      this.clearing = false;
    }
  }

  _abortFetch() {
    if (this._abortController) {
      this._abortController.abort();
      this._abortController = null;
    }
  }

  _formatTimestamp(value) {
    try {
      const date = new Date(value);
      if (Number.isNaN(date.getTime())) {
        return value;
      }
      return `${date.toLocaleDateString()} ${date.toLocaleTimeString()}`;
    } catch (_error) {
      return value;
    }
  }
}

customElements.define('cockpit-module-logs', CockpitModuleLogs);
