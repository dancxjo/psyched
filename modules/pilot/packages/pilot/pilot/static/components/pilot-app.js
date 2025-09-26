import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { buildRegimeGroups } from '../utils/regimes.js';
import { topicKey, topicIdentifier } from '../utils/topics.js';
import './module-section.js';

const MAX_LOG_ENTRIES = 40;

/**
 * Root component that orchestrates the pilot control surface.
 */
class PilotApp extends LitElement {
  static properties = {
    modules: { state: true },
    regimeGroups: { state: true },
    loading: { state: true },
    errorMessage: { state: true },
    commandLog: { state: true },
    activeTopics: { state: true },
    socketCount: { state: true },
  };

  constructor() {
    super();
    this.modules = [];
    this.regimeGroups = [];
    this.loading = true;
    this.errorMessage = '';
    this.commandLog = [];
    this.activeTopics = new Map();
    this.socketCount = 0;
  }

  /**
   * Render into light DOM so the existing LCARS stylesheets continue to apply.
   */
  createRenderRoot() {
    return this;
  }

  connectedCallback() {
    super.connectedCallback();
    this.fetchModules();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    for (const record of this.activeTopics.values()) {
      this.closeSocket(record);
    }
  }

  async fetchModules() {
    this.loading = true;
    this.errorMessage = '';
    try {
      const response = await fetch('/api/modules');
      if (!response.ok) {
        throw new Error(`Failed to load modules (${response.status})`);
      }
      const payload = await response.json();
      this.modules = Array.isArray(payload.modules) ? payload.modules : [];
      this.regimeGroups = buildRegimeGroups(this.modules);
      this.emitNavUpdate();
    } catch (error) {
      this.errorMessage = error?.message ?? String(error);
    } finally {
      this.loading = false;
    }
  }

  emitNavUpdate() {
    try {
      const detail = this.modules.map((module, index) => ({
        id: `module-${module.name}`,
        label: module.display_name || module.name,
        index,
      }));
      window.dispatchEvent(new CustomEvent('pilot-sections', { detail }));
    } catch (error) {
      console.warn('Failed to emit pilot navigation update', error);
    }
  }

  logCommand(entry) {
    const timestamped = { ...entry, timestamp: new Date().toISOString() };
    this.commandLog = [timestamped, ...this.commandLog].slice(0, MAX_LOG_ENTRIES);
  }

  setTopicRecord(key, record) {
    const next = new Map(this.activeTopics);
    next.set(key, record);
    this.activeTopics = next;
  }

  updateTopicRecord(key, updater) {
    const existing = this.activeTopics.get(key);
    if (!existing) {
      return;
    }
    const updated = updater(existing) || existing;
    this.setTopicRecord(key, updated);
  }

  deleteTopicRecord(key) {
    const next = new Map(this.activeTopics);
    next.delete(key);
    this.activeTopics = next;
  }

  closeSocket(record) {
    try {
      record?.socket?.close();
    } catch (error) {
      console.warn('Failed to close socket', error);
    }
  }

  async runCommand(moduleName, scope, command) {
    try {
      this.logCommand({ module: moduleName, scope, command, status: 'running' });
      const response = await fetch(`/api/modules/${moduleName}/commands`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ scope, command }),
      });
      if (!response.ok) {
        const detail = await response.json().catch(() => ({}));
        throw new Error(detail.detail || `Command failed (${response.status})`);
      }
      const data = await response.json();
      this.logCommand({ module: moduleName, scope, command, status: 'ok', result: data.result });
    } catch (error) {
      this.logCommand({
        module: moduleName,
        scope,
        command,
        status: 'error',
        error: error?.message ?? String(error),
      });
    }
  }

  async startTopic(moduleName, topic) {
    const key = topicKey(moduleName, topic);
    if (this.activeTopics.has(key)) {
      return;
    }

    const payload = {
      topic: topic.topic || topic.name,
      access: topic.access || 'ro',
      module: moduleName,
      qos: topic.qos ?? undefined,
    };

    try {
      const response = await fetch('/api/topics', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      });
      if (!response.ok) {
        const detail = await response.json().catch(() => ({}));
        throw new Error(detail.detail || `Unable to subscribe (${response.status})`);
      }
      const data = await response.json();
      const session = data.session;
      const send = (message) => {
        const current = this.activeTopics.get(key);
        if (current?.socket?.readyState === WebSocket.OPEN) {
          current.socket.send(JSON.stringify(message));
        }
      };
      const record = {
        module: moduleName,
        topic,
        session,
        state: 'connecting',
        last: null,
        messages: [],
        paused: Boolean(session?.paused),
        socket: null,
        error: null,
        send,
      };
      this.setTopicRecord(key, record);
      this.openSocket(key, record);
    } catch (error) {
      this.logCommand({
        module: moduleName,
        scope: 'topic',
        command: topicIdentifier(topic),
        status: 'error',
        error: error?.message ?? String(error),
      });
    }
  }

  async stopTopic(moduleName, topic) {
    const key = topicKey(moduleName, topic);
    const record = this.activeTopics.get(key);
    if (!record) {
      return;
    }
    this.closeSocket(record);
    try {
      await fetch(`/api/topics/${record.session.id}`, { method: 'DELETE' });
    } catch (error) {
      console.warn('Failed to drop topic', error);
    }
    this.deleteTopicRecord(key);
  }

  async togglePause(moduleName, topic, paused) {
    const key = topicKey(moduleName, topic);
    const record = this.activeTopics.get(key);
    if (!record) {
      return;
    }
    try {
      const response = await fetch(`/api/topics/${record.session.id}/pause`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ paused }),
      });
      if (!response.ok) {
        const detail = await response.json().catch(() => ({}));
        throw new Error(detail.detail || `Failed to update pause (${response.status})`);
      }
      const data = await response.json();
      this.updateTopicRecord(key, (existing) => ({ ...existing, paused: Boolean(data.session?.paused) }));
    } catch (error) {
      this.logCommand({
        module: moduleName,
        scope: 'topic',
        command: `${topicIdentifier(topic)}:pause`,
        status: 'error',
        error: error?.message ?? String(error),
      });
    }
  }

  openSocket(key, record) {
    const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    const url = `${protocol}://${window.location.host}/ws/topics/${record.session.id}`;

    let socket;
    try {
      socket = new WebSocket(url);
    } catch (error) {
      this.updateTopicRecord(key, (existing) => ({
        ...existing,
        state: 'error',
        error: error?.message ?? String(error),
      }));
      return;
    }

    this.socketCount += 1;
    this.updateTopicRecord(key, (existing) => ({ ...existing, socket, state: 'connecting', error: null }));

    socket.addEventListener('open', () => {
      this.updateTopicRecord(key, (existing) => ({ ...existing, state: 'connected', error: null }));
    });

    socket.addEventListener('close', () => {
      this.socketCount = Math.max(0, this.socketCount - 1);
      this.updateTopicRecord(key, (existing) => ({ ...existing, state: 'closed' }));
    });

    socket.addEventListener('error', () => {
      this.updateTopicRecord(key, (existing) => ({ ...existing, error: 'WebSocket error' }));
    });

    socket.addEventListener('message', (event) => {
      try {
        const payload = JSON.parse(event.data);
        const data = payload?.data ?? payload;
        this.updateTopicRecord(key, (existing) => ({
          ...existing,
          last: data,
          messages: [data, ...existing.messages].slice(0, 50),
        }));
        try {
          if (record.topic?.presentation === 'imu') {
            window.dispatchEvent(new CustomEvent('pilot-imu', { detail: data }));
          }
        } catch (dispatchError) {
          console.warn('Failed to dispatch pilot-imu event', dispatchError);
        }
      } catch (error) {
        console.warn('Failed to parse topic payload', error);
      }
    });
  }

  renderRegimeGroups() {
    return this.regimeGroups.map(
      (group) => html`
        <section class="regime-group" id=${`regime-${group.regime}`}>
          <header class="regime-header">
            <h1>${group.label}</h1>
            <span class="module-count">
              ${group.modules.length} ${group.modules.length === 1 ? 'module' : 'modules'}
            </span>
          </header>
          <div class="regime-modules">
            ${group.modules.map(
              (module) => html`
                <pilot-module-section
                  id=${`module-${module.name}`}
                  .module=${module}
                  .activeRecords=${this.activeTopics}
                  @run-command=${(event) => this.runCommand(module.name, event.detail.scope, event.detail.command)}
                  @start-topic=${(event) => this.startTopic(module.name, event.detail.topic)}
                  @stop-topic=${(event) => this.stopTopic(module.name, event.detail.topic)}
                  @pause-topic=${(event) => this.togglePause(module.name, event.detail.topic, event.detail.paused)}
                ></pilot-module-section>
              `,
            )}
          </div>
        </section>
      `,
    );
  }

  renderCommandLog() {
    if (!this.commandLog.length) {
      return nothing;
    }
    return html`
      <section class="command-log" id="command-log">
        <h2>Command Log</h2>
        <ul>
          ${this.commandLog.map(
            (entry) => html`
              <li class=${entry.status}>
                <span class="timestamp">${entry.timestamp}</span>
                <span class="module">${entry.module}</span>
                <span class="scope">${entry.scope}</span>
                <span class="command">${entry.command}</span>
                ${entry.status === 'error' && entry.error
                  ? html`<span class="detail">${entry.error}</span>`
                  : entry.result
                    ? html`<span class="detail">${JSON.stringify(entry.result)}</span>`
                    : nothing}
              </li>
            `,
          )}
        </ul>
      </section>
    `;
  }

  render() {
    if (this.loading) {
      return html`<div class="loading">Loading pilot modules…</div>`;
    }
    if (this.errorMessage) {
      return html`<div class="error">${this.errorMessage}</div>`;
    }
    return html`
      <div class="pilot-layout">
        ${this.renderRegimeGroups()}
      </div>
      ${this.renderCommandLog()}
      <footer class="status-bar">
        <div>Active sockets: ${this.socketCount}</div>
        <div>Total modules: ${this.modules.length}</div>
      </footer>
    `;
  }
}

customElements.define('pilot-app', PilotApp);
