import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

/**
 * Lightweight shell that lists module dashboards surfaced by the pilot.
 */
class PilotApp extends LitElement {
  static properties = {
    modules: { state: true },
    loading: { state: true },
    errorMessage: { state: true },
  };

  constructor() {
    super();
    this.modules = [];
    this.loading = true;
    this.errorMessage = '';
  }

  createRenderRoot() {
    return this;
  }

  connectedCallback() {
    super.connectedCallback();
    this.refresh();
  }

  async refresh() {
    this.loading = true;
    this.errorMessage = '';
    try {
      const response = await fetch('/api/modules');
      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }
      const payload = await response.json();
      this.modules = Array.isArray(payload.modules) ? payload.modules : [];
      this.broadcastNavigation();
      this.updateBridgeGlobals(payload.bridge);
    } catch (error) {
      this.errorMessage = error instanceof Error ? error.message : String(error);
    } finally {
      this.loading = false;
    }
  }

  updateBridgeGlobals(bridge) {
    if (typeof window === 'undefined') {
      return;
    }
    const pilotGlobals = window.Pilot ? { ...window.Pilot } : {};
    pilotGlobals.bridge = {
      ...(pilotGlobals.bridge || {}),
      ...(bridge || {}),
    };
    window.Pilot = pilotGlobals;
  }

  broadcastNavigation() {
    if (typeof window === 'undefined') {
      return;
    }
    const detail = [];
    let index = 0;
    for (const module of this.modules) {
      const slug = module.slug || module.name;
      if (!slug) continue;
      detail.push({
        id: `module-${slug}`,
        label: module.display_name || module.name,
        index: index++,
        url: module.dashboard_url || (module.has_pilot ? `/modules/${module.name}/` : undefined),
      });
    }
    window.dispatchEvent(new CustomEvent('pilot-sections', { detail }));
  }

  render() {
    if (this.loading) {
      return html`<section class="pilot-loading">Loading module dashboards…</section>`;
    }
    if (this.errorMessage) {
      return html`<section class="pilot-error">
        <h2>Failed to load modules</h2>
        <p>${this.errorMessage}</p>
        <button type="button" @click=${() => this.refresh()}>Retry</button>
      </section>`;
    }
    return html`
      <section class="pilot-directory">
        <h2>Available Dashboards</h2>
        <p>Select a module to open its dedicated control surface.</p>
        <ul class="pilot-directory__list">
          ${this.modules.map((module) => this.renderModuleCard(module))}
        </ul>
      </section>
    `;
  }

  renderModuleCard(module) {
    const href = module.dashboard_url || (module.has_pilot ? `/modules/${module.name}/` : null);
    return html`
      <li class="pilot-directory__item" id=${`module-${module.slug || module.name}`}>
        <div class="pilot-card">
          <div class="pilot-card__body">
            <h3>${module.display_name || module.name}</h3>
            ${module.description ? html`<p>${module.description}</p>` : ''}
          </div>
          ${href
            ? html`<a class="pilot-card__link" href=${href} target="_blank" rel="noopener">Open dashboard ↗</a>`
            : html`<span class="pilot-card__link pilot-card__link--disabled">No pilot assets</span>`}
        </div>
      </li>
    `;
  }
}

customElements.define('pilot-app', PilotApp);
