import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { surfaceStyles } from '/components/pilot-style.js';
import {
  normaliseHostMetadata,
  summariseModules,
  normaliseBridgeSettings,
} from './pilot-dashboard.helpers.js';

/**
 * Pilot operations console that summarises host metadata and bridge endpoints.
 *
 * The dashboard provides a quick snapshot of the cockpit's own health so
 * operators can confirm rosbridge and video relays before diving into other
 * module consoles.
 *
 * @example
 * ```html
 * <pilot-dashboard></pilot-dashboard>
 * ```
 */
class PilotDashboard extends LitElement {
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

      .module-list {
        list-style: none;
        margin: 0;
        padding: 0;
        display: grid;
        gap: 0.85rem;
      }

      .module-list__item {
        display: grid;
        gap: 0.5rem;
        padding: 0.75rem 0.9rem;
        border-radius: 0.5rem;
        background: rgba(0, 0, 0, 0.3);
      }

      .module-list__header {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.75rem;
      }

      .module-list__name {
        font-family: var(--metric-value-font);
        font-size: 1.05rem;
      }

      .module-list__description {
        margin: 0;
        color: var(--lcars-muted);
        font-size: 0.85rem;
      }

      .module-list__meta {
        display: flex;
        flex-wrap: wrap;
        gap: 0.75rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .module-list__link {
        color: var(--lcars-accent);
        text-decoration: none;
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
              <h3 class="surface-card__title">Pilot overview</h3>
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
        <article class="surface-card surface-card--compact">
          <h3 class="surface-card__title">Active modules</h3>
          ${this._renderModuleList()}
        </article>
      </div>
    `;
  }

  _renderStatus() {
    if (this.errorMessage) {
      return html`<p class="surface-status" data-variant="error">${this.errorMessage}</p>`;
    }
    if (this.loading) {
      return html`<p class="surface-status">Loading pilot metadata…</p>`;
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
    const { total, withPilot, withoutPilot } = this.summary;
    return html`
      <section class="surface-metric">
        <span class="surface-metric__label">Modules</span>
        <span class="surface-metric__value surface-metric__value--large">${total}</span>
        <span class="surface-status">Dashboards ready: ${withPilot}</span>
        <span class="surface-status">Awaiting dashboards: ${withoutPilot}</span>
      </section>
    `;
  }

  _renderModuleList() {
    if (this.errorMessage) {
      return html`<p class="surface-status" data-variant="error">${this.errorMessage}</p>`;
    }
    if (this.loading && !this.summary.modules.length) {
      return html`<p class="surface-status">Loading modules…</p>`;
    }
    if (!this.summary.modules.length) {
      return html`<p class="surface-status">No modules reported by the pilot host.</p>`;
    }
    return html`
      <ul class="module-list">
        ${this.summary.modules.map((module) => this._renderModule(module))}
      </ul>
    `;
  }

  _renderModule(module) {
    return html`
      <li class="module-list__item">
        <div class="module-list__header">
          <span class="module-list__name">${module.displayName}</span>
          <span class="surface-chip" data-variant=${module.hasPilot ? 'success' : 'warning'}>
            ${module.hasPilot ? 'Dashboard ready' : 'No dashboard'}
          </span>
        </div>
        ${module.description
          ? html`<p class="module-list__description">${module.description}</p>`
          : ''}
        <div class="module-list__meta">
          <span>Slug: ${module.slug}</span>
          ${module.dashboardUrl
            ? html`<a class="module-list__link" href="${module.dashboardUrl}" target="_blank" rel="noreferrer">Open dashboard</a>`
            : ''}
        </div>
      </li>
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
      this.summary = summariseModules(payload.modules);
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

customElements.define('pilot-dashboard', PilotDashboard);
