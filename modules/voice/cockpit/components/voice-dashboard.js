import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';
import { copyTextToClipboard, formatVoiceEventLogForCopy } from '/components/log-copy.js';

function generateId() {
    if (typeof crypto !== 'undefined' && crypto.randomUUID) {
        return crypto.randomUUID();
    }
    return `event-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

function createVoiceSocket(options) {
    return createTopicSocket({ module: 'voice', ...options });
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
        eventLogCopyState: { state: true },
        eventLogCopyMessage: { state: true },
    };

    static styles = [
        surfaceStyles,
        css`
      .surface-card__header {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.5rem;
      }

      .surface-card__title {
        margin: 0;
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
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--metric-label-color);
      }
      textarea,
      input {
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
      .voice-last {
        min-height: 3rem;
        overflow-wrap: anywhere;
      }
      .volume-control {
        display: flex;
        align-items: center;
        gap: 0.75rem;
      }
      .volume-control input[type='number'] {
        width: 80px;
      }
      .voice-log__topic {
        color: var(--lcars-muted);
        font-size: 0.75rem;
      }
    `,
    ];

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
        this.eventLogCopyState = 'idle';
        this.eventLogCopyMessage = '';
        this.voicePublisher = null;
        this.interruptPublisher = null;
        this.resumePublisher = null;
        this.clearPublisher = null;
        this.volumePublisher = null;
        this.sockets = [];
        this._eventLogCopyResetHandle = 0;
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
        this._clearEventLogCopyReset();
    }

    connectVoice() {
        const socket = createVoiceSocket({
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
        const socket = createVoiceSocket({
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

    async copyEventLog() {
        if (this.eventLogCopyState === 'copying') {
            return;
        }
        this.setEventLogCopyState('copying');
        try {
            const text = formatVoiceEventLogForCopy(this.eventLog);
            const success = await copyTextToClipboard(text);
            if (success) {
                this.setEventLogCopyState('copied');
            } else {
                this.setEventLogCopyState('failed', 'Clipboard unavailable. Copy the events manually.');
            }
        } catch (error) {
            const message = error instanceof Error ? error.message : String(error);
            this.setEventLogCopyState('failed', `Failed to copy event log: ${message}`);
        }
    }

    setEventLogCopyState(state, message = '') {
        this.eventLogCopyState = state;
        this.eventLogCopyMessage = message;
        this._clearEventLogCopyReset();
        if (state === 'copied' || state === 'failed') {
            this._eventLogCopyResetHandle = globalThis.setTimeout(() => {
                this.eventLogCopyState = 'idle';
                this.eventLogCopyMessage = '';
            }, 3000);
        }
    }

    _clearEventLogCopyReset() {
        if (this._eventLogCopyResetHandle) {
            globalThis.clearTimeout(this._eventLogCopyResetHandle);
            this._eventLogCopyResetHandle = 0;
        }
    }

    ensureVoicePublisher() {
        if (this.voicePublisher) {
            return this.voicePublisher;
        }
        const publisher = createVoiceSocket({
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
        const publisher = createVoiceSocket({
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
        const statusVariant = this.status === 'Live' ? 'success' : this.status === 'Error' ? 'error' : undefined;
        return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card">
          <h3 class="surface-card__title">Speak Command</h3>
          <p class="surface-status" data-variant="${statusVariant ?? ''}">Status: ${this.status}</p>
          <form @submit=${this.handleSendVoice}>
            <label>
              Text to speak
              <input type="text" .value=${this.voiceMessage} @input=${(e) => (this.voiceMessage = e.target.value)} placeholder="Type message">
            </label>
            <button class="surface-action" type="submit">Send to /voice</button>
            ${this.voiceFeedback ? html`<p class="surface-status">${this.voiceFeedback}</p>` : ''}
          </form>
          <div class="surface-panel surface-mono voice-last">${this.lastVoice || '—'}</div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Playback Controls</h3>
          <div class="surface-actions">
            <button class="surface-action" type="button" @click=${this.sendInterrupt}>Interrupt</button>
            <button class="surface-action" type="button" @click=${this.sendResume}>Resume</button>
            <button class="surface-action" type="button" @click=${this.sendClear}>Clear Queue</button>
          </div>
          ${this.commandFeedback ? html`<p class="surface-status">${this.commandFeedback}</p>` : ''}
          <label>
            Volume (0-255)
            <div class="volume-control">
              <input type="number" min="0" max="255" .value=${this.volume} @input=${(e) => (this.volume = Number(e.target.value || 0))} />
              <button class="surface-action" type="button" @click=${this.applyVolume}>Apply</button>
            </div>
          </label>
          ${this.volumeFeedback ? html`<p class="surface-status">${this.volumeFeedback}</p>` : ''}
        </article>

        <article class="surface-card">
          <header class="surface-card__header">
            <h3 class="surface-card__title">Event Log</h3>
            <button
              class="surface-action"
              type="button"
              ?disabled=${this.eventLogCopyState === 'copying' || this.eventLog.length === 0}
              @click=${() => this.copyEventLog()}
            >
              ${this.eventLogCopyState === 'copying'
                ? 'Copying…'
                : this.eventLogCopyState === 'copied'
                  ? 'Copied!'
                  : 'Copy log'}
            </button>
          </header>
          <ul class="surface-log">
            ${this.eventLog.length === 0
                ? html`<li class="surface-log__entry">No events yet</li>`
                : this.eventLog.map(
                    (event) => html`<li class="surface-log__entry" key="${event.id}">
                <strong>${event.label}</strong>
                <span class="voice-log__topic"><code>${event.topic}</code></span>
                <span class="surface-muted">${event.timestamp}</span>
              </li>`,
                )}
          </ul>
          ${this.eventLogCopyState === 'failed' && this.eventLogCopyMessage
                ? html`<p class="surface-status" data-variant="error">${this.eventLogCopyMessage}</p>`
                : ''}
        </article>
      </div>
    `;
    }
}

customElements.define('voice-dashboard', VoiceDashboard);
