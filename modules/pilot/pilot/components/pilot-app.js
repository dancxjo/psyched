import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { buildNavigationSections } from '../utils/navigation.js';
import './module-log-viewer.js';

// Component registry - maps module names to their component tag names
const MODULE_COMPONENTS = {
  chat: 'chat-dashboard',
  ear: 'ear-dashboard',
  eye: 'eye-dashboard',
  faces: 'faces-dashboard',
  felt: 'felt-dashboard',
  foot: 'foot-dashboard',
  gps: 'gps-dashboard',
  hypothalamus: 'hypothalamus-dashboard',
  imu: 'imu-dashboard',
  memory: 'memory-dashboard',
  nav: 'nav-dashboard',
  pilot: 'pilot-dashboard',
  viscera: 'viscera-dashboard',
  voice: 'voice-dashboard',
};

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
    this.loadedComponents = new Set();
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
      this.updateHostGlobals(payload.host);
      this.updateBridgeGlobals(payload.bridge);
      await this.loadModuleComponents();
    } catch (error) {
      this.errorMessage = error instanceof Error ? error.message : String(error);
    } finally {
      this.loading = false;
    }
  }

  async loadModuleComponents() {
    for (const module of this.modules) {
      const componentTag = MODULE_COMPONENTS[module.name];
      if (componentTag && !this.loadedComponents.has(module.name)) {
        try {
          await import(`/modules/${module.name}/components/${componentTag}.js`);
          this.loadedComponents.add(module.name);
        } catch (error) {
          console.warn(`Failed to load component for module ${module.name}:`, error);
        }
      }
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

  updateHostGlobals(host) {
    if (typeof window === 'undefined' || !host || typeof host !== 'object') {
      return;
    }
    const pilotGlobals = window.Pilot ? { ...window.Pilot } : {};
    pilotGlobals.host = {
      ...(pilotGlobals.host || {}),
      ...host,
    };
    window.Pilot = pilotGlobals;
  }

  broadcastNavigation() {
    if (typeof window === 'undefined') {
      return;
    }

    const sections = buildNavigationSections(this.modules);
    const pilotGlobals = window.Pilot ? { ...window.Pilot } : {};
    pilotGlobals.navigation = {
      ...(pilotGlobals.navigation || {}),
      sections,
    };
    window.Pilot = pilotGlobals;

    window.dispatchEvent(new CustomEvent('pilot-sections', { detail: sections }));
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
    const displayName = module.display_name || module.name;
    const componentTag = MODULE_COMPONENTS[module.name];

    return html`
      <section class="module-section" id=${`module-${slug}`}>
        <header class="module-section__header">
          <h2>${displayName}</h2>
          ${module.description ? html`<p class="module-description">${module.description}</p>` : ''}
        </header>
        ${componentTag
        ? html`<div class="module-surface">
              ${this.renderComponent(componentTag)}
              <pilot-module-logs module=${module.name}></pilot-module-logs>
            </div>`
        : html`<div class="module-placeholder">
              <p>This module does not expose pilot assets yet.</p>
            </div>`}
      </section>
    `;
  }

  renderComponent(tagName) {
    // Create the component element dynamically based on tag name
    const tagMap = {
      'chat-dashboard': html`<chat-dashboard></chat-dashboard>`,
      'ear-dashboard': html`<ear-dashboard></ear-dashboard>`,
      'eye-dashboard': html`<eye-dashboard></eye-dashboard>`,
      'faces-dashboard': html`<faces-dashboard></faces-dashboard>`,
      'felt-dashboard': html`<felt-dashboard></felt-dashboard>`,
      'foot-dashboard': html`<foot-dashboard></foot-dashboard>`,
      'gps-dashboard': html`<gps-dashboard></gps-dashboard>`,
      'hypothalamus-dashboard': html`<hypothalamus-dashboard></hypothalamus-dashboard>`,
      'imu-dashboard': html`<imu-dashboard></imu-dashboard>`,
      'memory-dashboard': html`<memory-dashboard></memory-dashboard>`,
      'nav-dashboard': html`<nav-dashboard></nav-dashboard>`,
      'pilot-dashboard': html`<pilot-dashboard></pilot-dashboard>`,
      'viscera-dashboard': html`<viscera-dashboard></viscera-dashboard>`,
      'voice-dashboard': html`<voice-dashboard></voice-dashboard>`,
    };
    return tagMap[tagName] || html`<p>Component ${tagName} not found</p>`;
  }
}

customElements.define('pilot-app', PilotApp);
