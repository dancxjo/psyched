import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { buildNavigationSections } from '/utils/navigation.js';
import './module-log-viewer.js';
import { surfaceStyles } from './cockpit-style.js';

// Component registry - maps module names to their component tag names
const MODULE_COMPONENTS = {
  chat: 'chat-dashboard',
  ear: 'ear-dashboard',
  eye: 'eye-dashboard',
  faces: 'faces-dashboard',
  pilot: 'pilot-dashboard',
  foot: 'foot-dashboard',
  gps: 'gps-dashboard',
  hypothalamus: 'hypothalamus-dashboard',
  imu: 'imu-dashboard',
  memory: 'memory-dashboard',
  nav: 'nav-dashboard',
  cockpit: 'cockpit-dashboard',
  viscera: 'viscera-dashboard',
  voice: 'voice-dashboard',
};

/**
 * Lightweight shell that lists module dashboards surfaced by the cockpit.
 */
class CockpitApp extends LitElement {
  static styles = [
    surfaceStyles,
    css`
      .module-surface {
        display: grid;
        gap: 0.85rem;
        grid-template-columns: minmax(0, 1fr);
      }
    `,
  ];

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
    const cockpitGlobals = window.Cockpit ? { ...window.Cockpit } : {};
    cockpitGlobals.bridge = {
      ...(cockpitGlobals.bridge || {}),
      ...(bridge || {}),
    };
    window.Cockpit = cockpitGlobals;
  }

  updateHostGlobals(host) {
    if (typeof window === 'undefined' || !host || typeof host !== 'object') {
      return;
    }
    const cockpitGlobals = window.Cockpit ? { ...window.Cockpit } : {};
    cockpitGlobals.host = {
      ...(cockpitGlobals.host || {}),
      ...host,
    };
    window.Cockpit = cockpitGlobals;
  }

  broadcastNavigation() {
    if (typeof window === 'undefined') {
      return;
    }

    const sections = buildNavigationSections(this.modules);
    const cockpitGlobals = window.Cockpit ? { ...window.Cockpit } : {};
    cockpitGlobals.navigation = {
      ...(cockpitGlobals.navigation || {}),
      sections,
    };
    window.Cockpit = cockpitGlobals;

    window.dispatchEvent(new CustomEvent('cockpit-sections', { detail: sections }));
  }

  render() {
    if (this.loading) {
      return html`<section class="cockpit-loading">Loading module dashboards…</section>`;
    }
    if (this.errorMessage) {
      return html`<section class="cockpit-error">
        <h2>Failed to load modules</h2>
        <p>${this.errorMessage}</p>
        <button type="button" @click=${() => this.refresh()}>Retry</button>
      </section>`;
    }
    if (!this.modules.length) {
      return html`<section class="cockpit-empty">
        <h2>No modules are currently active</h2>
        <p>The host configuration does not surface any cockpit dashboards. Confirm host.modules is populated.</p>
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
    const moduleContent = componentTag
      ? this.renderComponent(componentTag)
      : html`<div class="module-placeholder">
          <p>This module does not expose cockpit assets yet.</p>
        </div>`;

    return html`
      <section class="module-section" id=${`module-${slug}`}>
        <header class="module-section__header">
          <h2>${displayName}</h2>
          ${module.description ? html`<p class="module-description">${module.description}</p>` : ''}
        </header>
        <div class="module-surface surface-grid surface-grid--stack">
          ${moduleContent}
          <cockpit-module-logs module=${module.name} .moduleInfo=${module}></cockpit-module-logs>
        </div>
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
      'pilot-dashboard': html`<pilot-dashboard></pilot-dashboard>`,
      'foot-dashboard': html`<foot-dashboard></foot-dashboard>`,
      'gps-dashboard': html`<gps-dashboard></gps-dashboard>`,
      'hypothalamus-dashboard': html`<hypothalamus-dashboard></hypothalamus-dashboard>`,
      'imu-dashboard': html`<imu-dashboard></imu-dashboard>`,
      'memory-dashboard': html`<memory-dashboard></memory-dashboard>`,
      'nav-dashboard': html`<nav-dashboard></nav-dashboard>`,
      'cockpit-dashboard': html`<cockpit-dashboard></cockpit-dashboard>`,
      'viscera-dashboard': html`<viscera-dashboard></viscera-dashboard>`,
      'voice-dashboard': html`<voice-dashboard></voice-dashboard>`,
    };
    return tagMap[tagName] || html`<p>Component ${tagName} not found</p>`;
  }
}

customElements.define('cockpit-app', CockpitApp);
