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

  updated(changedProperties) {
    if (changedProperties.has('modules')) {
      this.updateComplete.then(() => this._bootstrapFrames());
    }
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
    if (!this.modules.length) {
      return html`<section class="pilot-empty">
        <h2>No modules are currently active</h2>
        <p>The host configuration does not surface any pilot dashboards. Confirm host.modules is populated.</p>
      </section>`;
    }

    return html`
      <div class="module-stack">
        ${this.modules.map((module) => this.renderModuleSurface(module))}
      </div>
    `;
  }

  renderModuleSurface(module) {
    const slug = module.slug || module.name;
    const fallbackUrl = module.has_pilot ? `/modules/${module.name}/` : null;
    const dashboardUrl = module.dashboard_url || fallbackUrl;
    const displayName = module.display_name || module.name;

    return html`
      <section class="module-section" id=${`module-${slug}`}>
        <header class="module-section__header">
          <div class="module-section__meta">
            <h2>${displayName}</h2>
            ${module.description ? html`<p>${module.description}</p>` : ''}
          </div>
          ${dashboardUrl
        ? html`<a class="module-section__link" href=${dashboardUrl} target="_blank" rel="noopener">Open standalone ↗</a>`
        : ''}
        </header>
        ${dashboardUrl
        ? html`<div class="module-surface">
              <iframe
                src=${dashboardUrl}
                title=${`${displayName} dashboard`}
                loading="lazy"
                allow="autoplay; clipboard-read; clipboard-write"
                data-module=${slug}
              ></iframe>
            </div>`
        : html`<div class="module-placeholder">
              <p>This module does not expose pilot assets yet.</p>
            </div>`}
      </section>
    `;
  }

  _bootstrapFrames() {
    const frames = this.querySelectorAll('iframe[data-module]');
    frames.forEach((frame) => {
      if (frame.dataset.bootstrap === 'true') {
        return;
      }
      frame.dataset.bootstrap = 'true';
      frame.addEventListener('load', () => {
        this._resizeFrame(frame);
        this._attachResizeObserver(frame);
      });
    });
  }

  _attachResizeObserver(frame) {
    try {
      const win = frame.contentWindow;
      const doc = win?.document;
      if (!win || !doc || typeof win.ResizeObserver === 'undefined') {
        return;
      }
      const target = doc.body || doc.documentElement;
      if (!target) {
        return;
      }
      const observer = new win.ResizeObserver(() => this._resizeFrame(frame));
      observer.observe(target);
      frame._pilotResizeObserver = observer;
    } catch (error) {
      // Cross-origin frames cannot be observed; ignore safely.
    }
  }

  _resizeFrame(frame) {
    const minHeight = 640;
    try {
      const doc = frame.contentDocument || frame.contentWindow?.document;
      if (!doc) {
        frame.style.height = `${minHeight}px`;
        return;
      }
      const body = doc.body;
      const html = doc.documentElement;
      const measured = Math.max(
        body ? body.scrollHeight : 0,
        html ? html.scrollHeight : 0,
        minHeight,
      );
      frame.style.height = `${measured}px`;
    } catch (error) {
      frame.style.height = `${minHeight}px`;
    }
  }
}

customElements.define('pilot-app', PilotApp);
