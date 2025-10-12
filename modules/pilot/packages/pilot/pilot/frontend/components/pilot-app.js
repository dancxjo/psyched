import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

// Regime grouping removed - modules are shown in the order returned by the API (host-defined)
import { createTopicSocket } from '../js/pilot.js';
import { topicKey, topicIdentifier, topicUpdateProfile } from '../utils/topics.js';
import './module-section.js';

const MAX_LOG_ENTRIES = 40;
const MAX_MESSAGE_HISTORY = 50;
const MAX_PENDING_BATCH = 32;

/**
 * Root component that orchestrates the pilot control surface.
 */
class PilotApp extends LitElement {
  static properties = {
    modules: { state: true },
    // regimeGroups removed
    loading: { state: true },
    errorMessage: { state: true },
    commandLog: { state: true },
    activeTopics: { state: true },
    socketCount: { state: true },
    logCollapsed: { state: true },
  };

  constructor() {
    super();
    this.modules = [];
    // modules are presented in server-provided order
    this.loading = true;
    this.errorMessage = '';
    this.commandLog = [];
    this.activeTopics = new Map();
    this.socketCount = 0;
    this.logCollapsed = false;
    this._pendingTopicFlushes = new Map();
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
    this.clearPendingTopicUpdates(key);
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

    const topicName = topic.topic || topic.name;
    const messageType = topic.type;
    if (!topicName || !messageType) {
      this.logCommand({
        module: moduleName,
        scope: 'topic',
        command: topicIdentifier(topic),
        status: 'error',
        error: 'Topic metadata missing name or type',
      });
      return;
    }

    const access = String(topic.access || 'ro').toLowerCase();
    let role = 'subscribe';
    if (access === 'wo') {
      role = 'publish';
    } else if (access === 'rw') {
      role = 'both';
    }

    let socket;
    try {
      socket = createTopicSocket({
        topic: topicName,
        type: messageType,
        role,
      });
    } catch (error) {
      this.logCommand({
        module: moduleName,
        scope: 'topic',
        command: topicIdentifier(topic),
        status: 'error',
        error: error?.message ?? String(error),
      });
      return;
    }

    const send = (message) => {
      if (socket?.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(message));
      }
    };

    const record = {
      module: moduleName,
      topic,
      state: 'connecting',
      last: null,
      messages: [],
      paused: false,
      socket,
      error: null,
      send,
    };

    this.setTopicRecord(key, record);
    this.socketCount += 1;

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
        const kind = payload?.event;
        if (kind === 'ready') {
          this.updateTopicRecord(key, (existing) => ({ ...existing, state: 'connected', error: null }));
          return;
        }
        if (kind === 'error') {
          const message = payload?.message || 'Topic bridge error';
          this.updateTopicRecord(key, (existing) => ({ ...existing, state: 'error', error: message }));
          return;
        }
        const data = kind === 'message' ? payload.data : payload;
        this.queueTopicMessage(key, data);
      } catch (error) {
        console.warn('Failed to parse topic payload', error);
      }
    });
  }

  async stopTopic(moduleName, topic) {
    const key = topicKey(moduleName, topic);
    const record = this.activeTopics.get(key);
    if (!record) {
      return;
    }
    this.closeSocket(record);
    this.deleteTopicRecord(key);
  }

  async togglePause(moduleName, topic, paused) {
    const key = topicKey(moduleName, topic);
    const record = this.activeTopics.get(key);
    if (!record) {
      return;
    }
    this.updateTopicRecord(key, (existing) => ({ ...existing, paused: Boolean(paused) }));
    this.logCommand({
      module: moduleName,
      scope: 'topic',
      command: `${topicIdentifier(topic)}:${paused ? 'pause' : 'resume'}`,
      status: 'ok',
    });
  }

  clearPendingTopicUpdates(key) {
    const pending = this._pendingTopicFlushes.get(key);
    if (!pending) {
      return;
    }
    if (typeof pending.cancel === 'function') {
      try {
        pending.cancel();
      } catch (error) {
        console.warn('Failed to cancel pending topic flush', error);
      }
    }
    this._pendingTopicFlushes.delete(key);
  }

  queueTopicMessage(key, data) {
    const record = this.activeTopics.get(key);
    const topic = record?.topic;
    if (!record || !topic) {
      return;
    }
    if (record.paused) {
      return;
    }

    const profile = topicUpdateProfile(topic);
    if (profile.mode === 'immediate') {
      this.applyTopicUpdates(key, [data]);
      this.dispatchImuEvent(topic, data);
      return;
    }

    const existing = this._pendingTopicFlushes.get(key);
    const queue = existing?.queue ?? [];
    if (profile.collapse) {
      queue.length = 0;
      queue.push(data);
    } else {
      queue.push(data);
      if (queue.length > MAX_PENDING_BATCH) {
        queue.splice(0, queue.length - MAX_PENDING_BATCH);
      }
    }

    const scheduleFlush = () => {
      this._pendingTopicFlushes.delete(key);
      const updates = queue.splice(0);
      if (!updates.length) {
        return;
      }
      this.applyTopicUpdates(key, updates);
      const latest = updates[updates.length - 1];
      this.dispatchImuEvent(topic, latest);
    };

    if (existing?.timer) {
      return;
    }

    if (profile.frame) {
      const handle = window.requestAnimationFrame(() => {
        scheduleFlush();
      });
      this._pendingTopicFlushes.set(key, {
        queue,
        timer: handle,
        cancel: () => window.cancelAnimationFrame(handle),
      });
      return;
    }

    const delay = Math.max(0, Number(profile.interval) || 0);
    const timeout = window.setTimeout(() => {
      scheduleFlush();
    }, delay);
    this._pendingTopicFlushes.set(key, {
      queue,
      timer: timeout,
      cancel: () => window.clearTimeout(timeout),
    });
  }

  applyTopicUpdates(key, updates) {
    if (!updates?.length) {
      return;
    }
    const newest = updates[updates.length - 1];
    const reversed = [...updates].reverse();
    this.updateTopicRecord(key, (existing) => {
      if (!existing) {
        return existing;
      }
      const history = Array.isArray(existing.messages) ? existing.messages : [];
      const messages = [...reversed, ...history].slice(0, MAX_MESSAGE_HISTORY);
      return {
        ...existing,
        last: newest,
        messages,
      };
    });
  }

  dispatchImuEvent(topic, data) {
    try {
      if (topic?.presentation === 'imu' || topic?.topic === '/imu') {
        window.dispatchEvent(new CustomEvent('pilot-imu', { detail: data }));
      }
    } catch (dispatchError) {
      console.warn('Failed to dispatch pilot-imu event', dispatchError);
    }
  }

  renderModules() {
    return this.modules.map(
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
    );
  }

  renderCommandLog() {
    if (!this.commandLog.length) {
      return nothing;
    }
    return html`
			<section class="command-log" id="command-log" ?data-collapsed=${this.logCollapsed}>
        <div class="command-log-header" style="display: flex; align-items: center; justify-content: space-between;">
          <h2 style="display: inline-block;">Command Log</h2>
          <button
            class="log-toggle control-button"
            aria-pressed=${this.logCollapsed}
            aria-label=${this.logCollapsed ? 'Expand command log' : 'Collapse command log'}
            @click=${() => { this.logCollapsed = !this.logCollapsed; }}
          >
            ${this.logCollapsed ? html`<svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" aria-hidden="true"><path d="M12 5v14M5 12h14" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></svg>` : html`<svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" aria-hidden="true"><path d="M5 12h14" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></svg>`}
          </button>
        </div>
				${this.logCollapsed ? nothing : html`
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
          : this.renderCommandResult(entry)}
							</li>
						`,
    )}
				</ul>
				`}
			</section>
		`;
  }

  renderCommandResult(entry) {
    const result = entry?.result;
    if (!result) {
      return nothing;
    }

    const hasCode = Number.isInteger(result.code);
    const stdout = result.stdout_plain ?? result.stdout ?? '';
    const stderr = result.stderr_plain ?? result.stderr ?? '';

    return html`
			<div class="detail result">
				${hasCode ? html`<div class="exit-code">Exit code: ${result.code}</div>` : nothing}
				${stdout ? html`<pre class="command-output stdout">${stdout}</pre>` : nothing}
				${stderr ? html`<pre class="command-output stderr">${stderr}</pre>` : nothing}
			</div>
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
				${this.renderModules()}
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
