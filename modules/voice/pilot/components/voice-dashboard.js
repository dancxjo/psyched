import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';

function generateId() {
  if (typeof crypto !== 'undefined' && crypto.randomUUID) {
    return crypto.randomUUID();
  }
  return `event-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

class VoiceDashboard extends LitElement {
  static properties = {
    status: { state: true },
    voiceFeedback: { state: true },
    commandFeedback: { state: true },
    volumeFeedback: { state: true },
    voiceMessage: { state: true },
    volume: { state: true },
    lastVoice: { state: true },
    eventLog: { state: true },
  };

  static styles = css`
    :host {
      display: block;
    }
    .voice-layout {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 1rem;
    }
    .voice-card {
      background: var(--control-surface-bg);
      border: 1px solid var(--control-surface-border);
      border-radius: var(--control-surface-radius);
      padding: var(--control-surface-padding);
      box-shadow: var(--control-surface-shadow);
      display: flex;
      flex-direction: column;
      gap: 0.75rem;
    }
    .voice-card h3 {
      margin: 0;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      font-size: 0.9rem;
      color: var(--metric-title-color);
    }
    .status {
      font-size: 0.8rem;
      color: var(--lcars-muted);
    }
    form {
      display: flex;
      flex-direction: column;
      gap: 0.5rem;
    }
    label {
      display: flex;
      flex-direction: column;
      gap: 0.25rem;
      font-size: 0.75rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      color: var(--metric-label-color);
    }
    textarea, input {
      font: inherit;
      padding: 0.5rem;
      border-radius: 0.5rem;
      border: 1px solid var(--control-surface-border);
      background: rgba(0, 0, 0, 0.3);
      color: var(--lcars-text);
      font-family: var(--metric-value-font);
    }
    textarea {
      resize: vertical;
      min-height: 60px;
      font-size: 0.85rem;
    }
    button {
      align-self: flex-start;
      padding: 0.5rem 1rem;
      border-radius: 999px;
      background: var(--lcars-accent-secondary);
      color: #05070d;
      border: none;
      font-weight: 600;
      cursor: pointer;
      font-size: 0.8rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      transition: background 120ms ease;
    }
    button:hover {
      background: var(--lcars-accent);
    }
    button.secondary {
      background: rgba(88, 178, 220, 0.25);
      color: var(--lcars-text);
    }
    .control-buttons {
      display: flex;
      flex-wrap: wrap;
      gap: 0.5rem;
    }
    .voice-last {
      background: rgba(0, 0, 0, 0.3);
      border: 1px solid var(--control-surface-border);
      border-radius: 0.5rem;
      padding: 0.5rem;
      min-height: 3rem;
      overflow-wrap: anywhere;
      font-family: var(--metric-value-font);
      font-size: 0.8rem;
    }
    .event-log {
      list-style: none;
      margin: 0;
      padding: 0;
      display: flex;
      flex-direction: column;
      gap: 0.5rem;
      max-height: 240px;
      overflow-y: auto;
    }
    .event-log li {
      background: rgba(0, 0, 0, 0.3);
      border: 1px solid var(--control-surface-border);
      border-radius: 0.5rem;
      padding: 0.5rem;
      display: flex;
      flex-direction: column;
      gap: 0.25rem;
      font-size: 0.85rem;
    }
    .event-log strong {
      color: var(--lcars-accent-secondary);
    }
    .volume-control {
      display: flex;
      align-items: center;
      gap: 0.75rem;
    }
    .volume-control input[type='number'] {
      width: 80px;
    }
  `;

  constructor() {
    super();
    this.status = 'Connecting…';
    this.voiceFeedback = '';
    this.commandFeedback = '';
    this.volumeFeedback = '';
    this.voiceMessage = '';
    this.volume = 255;
    this.lastVoice = '';
    this.eventLog = [];
    this.voicePublisher = null;
    this.interruptPublisher = null;
    this.resumePublisher = null;
    this.clearPublisher = null;
    this.volumePublisher = null;
    this.sockets = [];
  }

  connectedCallback() {
    super.connectedCallback();
    this.connectVoice();
    this.connectEventStreams();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    for (const socket of this.sockets) {
      try {
        socket.close();
      } catch (_error) {
        // ignore
      }
    }
    this.sockets.length = 0;
  }

  connectVoice() {
    const socket = createTopicSocket({
      topic: '/voice',
      type: 'std_msgs/msg/String',
      role: 'subscribe',
    });
    socket.addEventListener('message', (event) => {
      const payload = JSON.parse(event.data);
      if (payload.event === 'message' && payload.data && typeof payload.data.data !== 'undefined') {
        this.lastVoice = String(payload.data.data ?? '');
        this.status = 'Live';
      }
    });
    socket.addEventListener('close', () => {
      this.status = 'Disconnected';
    });
    socket.addEventListener('error', () => {
      this.status = 'Error';
    });
    this.sockets.push(socket);
    this.ensureVoicePublisher();
  }

  connectEventStreams() {
    this.createEventListener('/voice_done', 'Playback complete');
    this.createEventListener('/voice_interrupt', 'Playback interrupted');
  }

  createEventListener(topic, label) {
    const socket = createTopicSocket({
      topic,
      type: 'std_msgs/msg/Empty',
      role: 'subscribe',
    });
    socket.addEventListener('message', () => {
      const event = {
        id: generateId(),
        label,
        topic,
        timestamp: new Date().toLocaleTimeString(),
      };
      this.eventLog = [event, ...this.eventLog].slice(0, 30);
    });
    this.sockets.push(socket);
  }

  ensureVoicePublisher() {
    if (this.voicePublisher) {
      return this.voicePublisher;
    }
    const publisher = createTopicSocket({
      topic: '/voice',
      type: 'std_msgs/msg/String',
      role: 'publish',
    });
    publisher.addEventListener('open', () => {
      this.voiceFeedback = '';
    });
    publisher.addEventListener('error', () => {
      this.voiceFeedback = 'Unable to publish to /voice';
    });
    this.voicePublisher = publisher;
    this.sockets.push(publisher);
    return publisher;
  }

  ensurePublisher(property, topic, type) {
    if (this[property]) {
      return this[property];
    }
    const publisher = createTopicSocket({
      topic,
      type,
      role: 'publish',
    });
    publisher.addEventListener('error', () => {
      this.commandFeedback = `Unable to publish to ${topic}`;
    });
    this[property] = publisher;
    this.sockets.push(publisher);
    return publisher;
  }

  handleSendVoice(event) {
    event.preventDefault();
    const text = this.voiceMessage.trim();
    if (!text) {
      this.voiceFeedback = 'Message text required for /voice';
      return;
    }
    try {
      const publisher = this.ensureVoicePublisher();
      publisher.send(JSON.stringify({ data: text }));
      this.voiceFeedback = 'Voice message sent';
      this.voiceMessage = '';
      this.lastVoice = text;
    } catch (error) {
      this.voiceFeedback = error instanceof Error ? error.message : String(error);
    }
  }

  sendInterrupt() {
    this.sendEmptyCommand('interruptPublisher', '/voice/interrupt', 'std_msgs/msg/Empty', 'Interrupt sent');
  }

  sendResume() {
    this.sendEmptyCommand('resumePublisher', '/voice/resume', 'std_msgs/msg/Empty', 'Resume sent');
  }

  sendClear() {
    this.sendEmptyCommand('clearPublisher', '/voice/clear', 'std_msgs/msg/Empty', 'Clear sent');
  }

  sendEmptyCommand(property, topic, type, successMessage) {
    try {
      const publisher = this.ensurePublisher(property, topic, type);
      publisher.send(JSON.stringify({}));
      this.commandFeedback = successMessage;
    } catch (error) {
      this.commandFeedback = error instanceof Error ? error.message : String(error);
    }
  }

  applyVolume() {
    const value = Math.max(0, Math.min(255, Number(this.volume)));
    this.volume = value;
    try {
      const publisher = this.ensurePublisher('volumePublisher', '/voice/volume', 'std_msgs/msg/UInt8');
      publisher.send(JSON.stringify({ data: value }));
      this.volumeFeedback = `Volume set to ${value}`;
    } catch (error) {
      this.volumeFeedback = error instanceof Error ? error.message : String(error);
    }
  }

  render() {
    return html`
      <div class="voice-layout">
        <article class="voice-card">
          <h3>Speak Command</h3>
          <p class="status">Status: <strong>${this.status}</strong></p>
          <form @submit=${this.handleSendVoice}>
            <label>
              Text to speak
              <textarea rows="3" .value=${this.voiceMessage} @input=${(e) => (this.voiceMessage = e.target.value)} placeholder="Type message"></textarea>
            </label>
            <button type="submit">Send to /voice</button>
            ${this.voiceFeedback ? html`<p class="status">${this.voiceFeedback}</p>` : ''}
          </form>
          <label style="margin-top: 0.5rem;">Last /voice
            <div class="voice-last">${this.lastVoice || '—'}</div>
          </label>
        </article>

        <article class="voice-card">
          <h3>Playback Controls</h3>
          <div class="control-buttons">
            <button class="secondary" @click=${this.sendInterrupt}>Interrupt</button>
            <button class="secondary" @click=${this.sendResume}>Resume</button>
            <button class="secondary" @click=${this.sendClear}>Clear Queue</button>
          </div>
          ${this.commandFeedback ? html`<p class="status">${this.commandFeedback}</p>` : ''}
          <label style="margin-top: 0.5rem;">
            Volume (0-255)
            <div class="volume-control">
              <input type="number" min="0" max="255" .value=${this.volume} @input=${(e) => (this.volume = parseInt(e.target.value))} />
              <button class="secondary" @click=${this.applyVolume}>Apply</button>
            </div>
          </label>
          ${this.volumeFeedback ? html`<p class="status">${this.volumeFeedback}</p>` : ''}
        </article>

        <article class="voice-card">
          <h3>Event Log</h3>
          <ul class="event-log">
            ${this.eventLog.length === 0
              ? html`<li>No events yet</li>`
              : this.eventLog.map(
                  (event) => html`
                    <li key="${event.id}">
                      <strong>${event.label}</strong>
                      <span class="status">${event.topic} · ${event.timestamp}</span>
                    </li>
                  `,
                )}
          </ul>
        </article>
      </div>
    `;
  }
}

customElements.define('voice-dashboard', VoiceDashboard);
