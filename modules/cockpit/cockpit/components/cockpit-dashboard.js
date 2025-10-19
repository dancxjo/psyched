import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { surfaceStyles } from '/components/cockpit-style.js';
import {
  normaliseHostMetadata,
  summariseModules,
  normaliseBridgeSettings,
} from './cockpit-dashboard.helpers.js';

/**
 * Cockpit operations console that summarises host metadata and bridge endpoints.
 *
 * The dashboard provides a quick snapshot of the cockpit's own health so
 * operators can confirm rosbridge and video relays before diving into other
 * module consoles.
 *
 * @example
 * ```html
 * <cockpit-dashboard></cockpit-dashboard>
 * ```
 */
class CockpitDashboard extends LitElement {
  static properties = {
    loading: { state: true },
    errorMessage: { state: true },
    host: { state: true },
    bridge: { state: true },
    summary: { state: true },
    lastUpdated: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      :host {
        display: block;
      }

      .dashboard-status {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.75rem;
      }

      .dashboard-status__meta {
        display: flex;
        flex-direction: column;
        gap: 0.25rem;
      }

      .dashboard-status__actions {
        display: flex;
        align-items: center;
      }

      .metric-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
        gap: 0.75rem;
      }

    `,
  ];

  constructor() {
    super();
    this.loading = true;
    this.errorMessage = '';
    this.host = normaliseHostMetadata(null);
    this.bridge = normaliseBridgeSettings(null);
    this.summary = summariseModules([]);
    this.lastUpdated = null;
    this._abortController = null;
  }

  connectedCallback() {
    super.connectedCallback();
    this.refresh();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._abortFetch();
  }

  render() {
    return html`
      <div class="surface-grid surface-grid--stack">
        <article class="surface-card">
          <header class="dashboard-status">
            <div class="dashboard-status__meta">
              <h3 class="surface-card__title">Cockpit overview</h3>
              ${this._renderStatus()}
            </div>
            <div class="dashboard-status__actions">
              <button
                type="button"
                class="surface-action"
                ?disabled=${this.loading}
                @click=${() => this.refresh()}
              >
                ${this.loading ? 'Refreshing…' : 'Refresh'}
              </button>
            </div>
          </header>
          <div class="metric-grid">
            ${this._renderHostCard()}
            ${this._renderBridgeCard()}
            ${this._renderModuleSummaryCard()}
          </div>
        </article>
      </div>
    `;
  }

  _renderStatus() {
    if (this.errorMessage) {
      return html`<p class="surface-status" data-variant="error">${this.errorMessage}</p>`;
    }
    if (this.loading) {
      return html`<p class="surface-status">Loading cockpit metadata…</p>`;
    }
    const timestamp = this.lastUpdated instanceof Date ? this.lastUpdated.toLocaleTimeString() : 'Never';
    return html`<p class="surface-status">Updated ${timestamp}</p>`;
  }

  _renderHostCard() {
    return html`
      <section class="surface-metric">
        <span class="surface-metric__label">Host</span>
        <span class="surface-metric__value surface-metric__value--large">${this.host.name}</span>
        <span class="surface-status">Shortname: ${this.host.shortname}</span>
      </section>
    `;
  }

  _renderBridgeCard() {
    return html`
      <section class="surface-metric">
        <span class="surface-metric__label">ROS Bridge</span>
        <span class="surface-metric__value">${this.bridge.mode}</span>
        <span class="surface-status">Primary: ${this.bridge.effectiveRosbridgeUri}</span>
        ${this.bridge.videoBase
        ? html`<span class="surface-status">Video base: ${this.bridge.videoBase}${this.bridge.videoPort ? `:${this.bridge.videoPort}` : ''}</span>`
        : ''}
      </section>
    `;
  }

  _renderModuleSummaryCard() {
    const { total, withCockpit, withoutCockpit } = this.summary;
    return html`
      <section class="surface-metric">
        <span class="surface-metric__label">Modules</span>
        <span class="surface-metric__value surface-metric__value--large">${total}</span>
        <span class="surface-status">Dashboards ready: ${withCockpit}</span>
        <span class="surface-status">Awaiting dashboards: ${withoutCockpit}</span>
      </section>
    `;
  }


  async refresh() {
    this._abortFetch();
    const controller = new AbortController();
    this._abortController = controller;
    this.loading = true;
    this.errorMessage = '';

    try {
      const response = await fetch('/api/modules', { signal: controller.signal });
      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }
      const payload = await response.json();
      this.host = normaliseHostMetadata(payload.host);
      this.bridge = normaliseBridgeSettings(payload.bridge);
      const modules = Array.isArray(payload.modules) ? payload.modules : [];
      this.summary = summariseModules(modules);
      this.lastUpdated = new Date();
    } catch (error) {
      if (controller.signal.aborted) {
        return;
      }
      this.errorMessage = error instanceof Error ? error.message : String(error);
    } finally {
      if (this._abortController === controller) {
        this._abortController = null;
      }
      this.loading = false;
    }
  }

  _abortFetch() {
    if (this._abortController) {
      this._abortController.abort();
      this._abortController = null;
    }
  }
}

customElements.define('cockpit-dashboard', CockpitDashboard);
