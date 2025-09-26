<script>
  import { onMount } from 'svelte';
  import ModuleSection from './ModuleSection.svelte';

  let modules = [];
  let regimeGroups = [];
  let loading = true;
  let errorMessage = '';
  let commandLog = [];
  let activeTopics = new Map();
  let socketCount = 0;

  const maxLogEntries = 40;
  const defaultRegime = 'general';
  const regimeOrder = ['system', 'audio', 'conversation', 'navigation', 'behavior', 'perception', 'mobility'];
  const regimeLabels = {
    system: 'System',
    audio: 'Audio',
    conversation: 'Conversation',
    navigation: 'Navigation',
    behavior: 'Behavior',
    perception: 'Perception',
    mobility: 'Mobility',
    general: 'General',
  };

  function topicKey(moduleName, topic) {
    return `${moduleName}:${topic.name || topic.topic || topic}`;
  }

  function normalizeRegimes(entry) {
    if (!entry || !entry.regimes) {
      return [defaultRegime];
    }
    const value = entry.regimes;
    if (Array.isArray(value)) {
      const cleaned = value.map((item) => `${item}`.trim()).filter(Boolean);
      return cleaned.length ? cleaned : [defaultRegime];
    }
    const coerced = `${value}`.trim();
    return coerced ? [coerced] : [defaultRegime];
  }

  function formatRegimeName(slug) {
    if (slug in regimeLabels) {
      return regimeLabels[slug];
    }
    return slug
      .split(/[-_\s]+/)
      .filter(Boolean)
      .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
      .join(' ');
  }

  function buildRegimeGroups(entries) {
    const groups = new Map();
    for (const module of entries) {
      const regimes = normalizeRegimes(module);
      for (const regime of regimes) {
        if (!groups.has(regime)) {
          groups.set(regime, []);
        }
        groups.get(regime).push(module);
      }
    }
    const orderedKeys = [
      ...regimeOrder,
      ...Array.from(groups.keys()).filter((regime) => !regimeOrder.includes(regime)).sort(),
    ];
    return orderedKeys
      .filter((regime) => groups.has(regime))
      .map((regime) => ({
        regime,
        label: formatRegimeName(regime),
        modules: groups.get(regime),
      }));
  }

  function emitNavUpdate() {
    if (typeof window === 'undefined') {
      return;
    }
    const detail = modules.map((module, index) => ({
      id: `module-${module.name}`,
      label: module.display_name || module.name,
      index,
    }));
    window.dispatchEvent(new CustomEvent('pilot-sections', { detail }));
  }

  function logCommand(entry) {
    commandLog = [
      { ...entry, timestamp: new Date().toISOString() },
      ...commandLog,
    ].slice(0, maxLogEntries);
  }

  function setTopicRecord(key, record) {
    const next = new Map(activeTopics);
    next.set(key, record);
    activeTopics = next;
  }

  function deleteTopicRecord(key) {
    const next = new Map(activeTopics);
    next.delete(key);
    activeTopics = next;
  }

  async function fetchModules() {
    loading = true;
    errorMessage = '';
    try {
      const response = await fetch('/api/modules');
      if (!response.ok) {
        throw new Error(`Failed to load modules (${response.status})`);
      }
      const payload = await response.json();
      modules = payload.modules ?? [];
      regimeGroups = buildRegimeGroups(modules);
      emitNavUpdate();
    } catch (error) {
      errorMessage = error.message ?? String(error);
    } finally {
      loading = false;
    }
  }

  onMount(() => {
    fetchModules();
  });

  async function runCommand(moduleName, scope, command) {
    try {
      logCommand({ module: moduleName, scope, command, status: 'running' });
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
      logCommand({ module: moduleName, scope, command, status: 'ok', result: data.result });
    } catch (error) {
      logCommand({ module: moduleName, scope, command, status: 'error', error: error.message ?? String(error) });
    }
  }

  function getRecord(moduleName, topic) {
    return activeTopics.get(topicKey(moduleName, topic));
  }

  function updateRecord(moduleName, topic, updater) {
    const key = topicKey(moduleName, topic);
    const existing = activeTopics.get(key);
    if (!existing) return;
    const updated = updater(existing) || existing;
    setTopicRecord(key, updated);
  }

  function closeSocket(record) {
    try {
      record.socket?.close();
    } catch (error) {
      console.warn('Failed to close socket', error);
    }
  }

  async function startTopic(moduleName, topic) {
    const key = topicKey(moduleName, topic);
    if (activeTopics.has(key)) {
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
      const record = {
        module: moduleName,
        topic,
        session,
        state: 'connecting',
        last: null,
        messages: [],
        paused: Boolean(session.paused),
        socket: null,
        error: null,
        send(payload) {
          if (this.socket && this.socket.readyState === WebSocket.OPEN) {
            this.socket.send(JSON.stringify(payload));
          }
        },
      };
      setTopicRecord(key, record);
      openSocket(key, record);
    } catch (error) {
      logCommand({ module: moduleName, scope: 'topic', command: payload.topic, status: 'error', error: error.message ?? String(error) });
    }
  }

  async function stopTopic(moduleName, topic) {
    const key = topicKey(moduleName, topic);
    const record = activeTopics.get(key);
    if (!record) return;
    closeSocket(record);
    try {
      await fetch(`/api/topics/${record.session.id}`, { method: 'DELETE' });
    } catch (error) {
      console.warn('Failed to drop topic', error);
    }
    deleteTopicRecord(key);
  }

  async function togglePause(moduleName, topic, paused) {
    const record = getRecord(moduleName, topic);
    if (!record) return;
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
      updateRecord(moduleName, topic, (existing) => ({ ...existing, paused: data.session.paused }));
    } catch (error) {
      logCommand({ module: moduleName, scope: 'topic', command: `${topic.topic || topic.name}:pause`, status: 'error', error: error.message ?? String(error) });
    }
  }

  function openSocket(key, record) {
    const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    const url = `${protocol}://${window.location.host}/ws/topics/${record.session.id}`;
    try {
      const socket = new WebSocket(url);
      record.socket = socket;
      socketCount += 1;
      record.state = 'connecting';
      setTopicRecord(key, record);

      socket.addEventListener('open', () => {
        record.state = 'connected';
        record.error = null;
        setTopicRecord(key, record);
      });

      socket.addEventListener('close', () => {
        record.state = 'closed';
        setTopicRecord(key, record);
        socketCount = Math.max(0, socketCount - 1);
      });

      socket.addEventListener('error', () => {
        record.error = 'WebSocket error';
        setTopicRecord(key, record);
      });

      socket.addEventListener('message', (event) => {
        try {
          const payload = JSON.parse(event.data);
          record.last = payload.data ?? payload;
          record.messages = [record.last, ...record.messages].slice(0, 50);
          setTopicRecord(key, record);
          // Emit a global event for IMU messages so other UI controls can react.
          try {
            if (record.topic && record.topic.presentation === 'imu') {
              window.dispatchEvent(new CustomEvent('pilot-imu', { detail: record.last }));
            }
          } catch (e) {
            console.warn('Failed to dispatch pilot-imu event', e);
          }
        } catch (error) {
          console.warn('Failed to parse topic payload', error);
        }
      });
    } catch (error) {
      record.error = error.message ?? String(error);
      record.state = 'error';
      setTopicRecord(key, record);
    }
  }

  function handleStartTopic(event) {
    const { module, topic } = event.detail;
    startTopic(module, topic);
  }

  function handleStopTopic(event) {
    const { module, topic } = event.detail;
    stopTopic(module, topic);
  }

  function handlePauseTopic(event) {
    const { module, topic, paused } = event.detail;
    togglePause(module, topic, paused);
  }
</script>

{#if loading}
  <div class="loading">Loading pilot modules…</div>
{:else if errorMessage}
  <div class="error">{errorMessage}</div>
{:else}
  <div class="pilot-layout">
    {#each regimeGroups as group (group.regime)}
      <section class="regime-group" id={`regime-${group.regime}`}>
        <header class="regime-header">
          <h1>{group.label}</h1>
          <span class="module-count">{group.modules.length} {group.modules.length === 1 ? 'module' : 'modules'}</span>
        </header>
        <div class="regime-modules">
          {#each group.modules as module (module.name)}
            <ModuleSection
              id={`module-${module.name}`}
              module={module}
              activeTopics={activeTopics}
              on:runCommand={({ detail }) => runCommand(module.name, detail.scope, detail.command)}
              on:startTopic={handleStartTopic}
              on:stopTopic={handleStopTopic}
              on:pauseTopic={handlePauseTopic}
            />
          {/each}
        </div>
      </section>
    {/each}
  </div>
{/if}

{#if commandLog.length}
  <section class="command-log" id="command-log">
    <h2>Command Log</h2>
    <ul>
      {#each commandLog as entry, index}
        <li class={entry.status}>
          <span class="timestamp">{entry.timestamp}</span>
          <span class="module">{entry.module}</span>
          <span class="scope">{entry.scope}</span>
          <span class="command">{entry.command}</span>
          {#if entry.status === 'error'}
            <span class="detail">{entry.error}</span>
          {:else if entry.result}
            <span class="detail">{JSON.stringify(entry.result)}</span>
          {/if}
        </li>
      {/each}
    </ul>
  </section>
{/if}

<footer class="status-bar">
  <div>Active sockets: {socketCount}</div>
  <div>Total modules: {modules.length}</div>
</footer>

<style>
  .pilot-layout {
    display: flex;
    flex-direction: column;
    gap: 2.5rem;
  }

  .regime-group {
    display: flex;
    flex-direction: column;
    gap: 1.5rem;
    padding-bottom: 1.5rem;
    border-bottom: 1px solid rgba(148, 163, 184, 0.4);
  }

  .regime-group:last-of-type {
    border-bottom: none;
  }

  .regime-header {
    display: flex;
    flex-wrap: wrap;
    align-items: baseline;
    gap: 0.75rem;
    justify-content: space-between;
  }

  .regime-header h1 {
    margin: 0;
    font-size: 1.75rem;
  }

  .module-count {
    font-size: 0.875rem;
    font-weight: 600;
    letter-spacing: 0.03em;
    text-transform: uppercase;
    color: #64748b;
  }

  .regime-modules {
    display: grid;
    gap: 1.5rem;
  }

  @media (min-width: 900px) {
    .regime-modules {
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
    }
  }

  .command-log {
    margin-top: 2rem;
  }

  .command-log ul {
    list-style: none;
    padding: 0;
    margin: 0;
  }

  .command-log li {
    display: grid;
    grid-template-columns: auto auto auto 1fr;
    gap: 0.75rem;
    align-items: center;
    padding: 0.5rem 0;
    border-bottom: 1px solid rgba(148, 163, 184, 0.3);
  }

  .command-log li:last-child {
    border-bottom: none;
  }

  .command-log li.error {
    color: #b91c1c;
  }

  .command-log li.ok {
    color: #15803d;
  }

  .command-log .timestamp {
    font-family: monospace;
    font-size: 0.75rem;
    color: #475569;
  }

  .command-log .module {
    font-weight: 600;
  }

  .command-log .scope {
    font-size: 0.875rem;
    color: #475569;
  }

  .command-log .detail {
    grid-column: 1 / -1;
    font-size: 0.75rem;
    color: #0f172a;
  }

  .status-bar {
    display: flex;
    flex-wrap: wrap;
    gap: 1rem;
    justify-content: space-between;
    padding: 1rem 0;
    color: #334155;
    border-top: 1px solid rgba(148, 163, 184, 0.4);
    margin-top: 2rem;
  }
</style>
