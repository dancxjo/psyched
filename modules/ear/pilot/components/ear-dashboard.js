import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';
import { surfaceStyles } from '/components/pilot-style.js';
import { sampleRateFromMessage } from '/components/utils/audio.js';
import '/components/audio-oscilloscope.js';

const AUDIO_TOPIC = '/audio/raw';
const SPEECH_TOPIC = '/ear/speech_active';
const SILENCE_TOPIC = '/ear/silence';
const TRANSCRIPT_TOPIC = '/ear/hole';
const MAX_TRANSCRIPTS = 40;

function uniqueId(prefix = 'ear') {
    if (typeof crypto !== 'undefined' && crypto.randomUUID) {
        return crypto.randomUUID();
    }
    return `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

function safeToString(value) {
    if (value == null) {
        return '';
    }
    if (typeof value === 'string') {
        return value;
    }
    return String(value);
}

function byteLengthFromMessage(message) {
    const candidate = message?.data ?? message?.bytes ?? message;
    if (candidate == null) {
        return 0;
    }
    if (candidate instanceof ArrayBuffer) {
        return candidate.byteLength;
    }
    if (candidate && typeof candidate.byteLength === 'number') {
        return candidate.byteLength;
    }
    if (ArrayBuffer.isView(candidate)) {
        return candidate.byteLength;
    }
    if (Array.isArray(candidate)) {
        return candidate.length;
    }
    if (typeof candidate === 'string') {
        return candidate.length;
    }
    return 0;
}

class EarDashboard extends LitElement {
    static properties = {
        audioRecord: { state: true },
        audioStatus: { state: true },
        audioMonitoringEnabled: { state: true },
        lastAudioTimestamp: { state: true },
        audioSampleRate: { state: true },
        lastFrameByteLength: { state: true },
        speechActive: { state: true },
        silenceDetected: { state: true },
        transcripts: { state: true },
    };

    static styles = [
        surfaceStyles,
        css`
      .indicator-row {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
        align-items: center;
        margin-bottom: 0.75rem;
      }

      .oscilloscope-wrapper {
        display: grid;
        gap: 0.5rem;
      }

      .oscilloscope-wrapper[data-state='idle'] {
        min-height: 200px;
      }

      .transcript-log {
        list-style: none;
        padding: 0;
        margin: 0;
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
        max-height: 320px;
        overflow-y: auto;
      }

      .transcript-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.35rem;
      }

      .transcript-entry__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .surface-actions {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
      }

      .sensor-list {
        list-style: none;
        margin: 0;
        padding: 0;
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
      }

      .sensor-item {
        display: flex;
        align-items: center;
        justify-content: space-between;
        padding: 0.55rem 0.75rem;
        border-radius: 0.5rem;
        background: rgba(255, 255, 255, 0.05);
        font-size: 0.85rem;
      }

      .sensor-item[data-state='active'] {
        background: rgba(106, 209, 255, 0.2);
        color: var(--lcars-accent);
      }

      .sensor-item__dot {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        background: var(--lcars-accent-secondary);
      }

      .sensor-item[data-state='active'] .sensor-item__dot {
        background: var(--lcars-accent);
        box-shadow: 0 0 12px var(--lcars-accent);
      }
    `,
    ];

    constructor() {
        super();
        this.audioRecord = null;
        this.audioStatus = 'Connecting…';
        this.audioMonitoringEnabled = true;
        this.lastAudioTimestamp = 'Never';
        this.audioSampleRate = 16000;
        this.lastFrameByteLength = 0;
        this.speechActive = false;
        this.silenceDetected = true;
        this.transcripts = [];
        this._sockets = new Map();
    }

    connectedCallback() {
        super.connectedCallback();
        this._subscribeSpeech();
        this._subscribeSilence();
        this._subscribeTranscripts();
        if (this.audioMonitoringEnabled) {
            this._subscribeAudio();
        } else {
            this.audioStatus = 'Paused';
        }
    }

    disconnectedCallback() {
        super.disconnectedCallback();
        this._teardownAll();
    }

    toggleAudioMonitoring() {
        if (this.audioMonitoringEnabled) {
            this.audioMonitoringEnabled = false;
            this.audioStatus = 'Paused';
            this._closeSocket('audio');
            this.audioRecord = null;
            this.lastFrameByteLength = 0;
            return;
        }
        this.audioMonitoringEnabled = true;
        this.audioStatus = 'Connecting…';
        this._subscribeAudio();
    }

    clearTranscripts() {
        this.transcripts = [];
    }

    _subscribeAudio() {
        if (!this.audioMonitoringEnabled) {
            return;
        }
        this._openSocket(
            'audio',
            {
                topic: AUDIO_TOPIC,
                type: 'std_msgs/msg/UInt8MultiArray',
                role: 'subscribe',
            },
            (message) => {
                this.audioStatus = 'Live';
                const sampleRate = sampleRateFromMessage(message, this.audioSampleRate || 16000);
                this.audioSampleRate = sampleRate;
                this.lastFrameByteLength = byteLengthFromMessage(message);
                this.lastAudioTimestamp = new Date().toLocaleTimeString();
                this.audioRecord = {
                    last: message,
                    topic: { name: AUDIO_TOPIC },
                };
            },
        );
    }

    _subscribeSpeech() {
        this._openSocket(
            'speech',
            {
                topic: SPEECH_TOPIC,
                type: 'std_msgs/msg/Bool',
                role: 'subscribe',
            },
            (message) => {
                this.speechActive = Boolean(message?.data);
            },
        );
    }

    _subscribeSilence() {
        this._openSocket(
            'silence',
            {
                topic: SILENCE_TOPIC,
                type: 'std_msgs/msg/Bool',
                role: 'subscribe',
            },
            (message) => {
                if (typeof message?.data === 'boolean') {
                    this.silenceDetected = message.data;
                } else {
                    this.silenceDetected = Boolean(message?.data ?? true);
                }
            },
        );
    }

    _subscribeTranscripts() {
        this._openSocket(
            'transcripts',
            {
                topic: TRANSCRIPT_TOPIC,
                type: 'std_msgs/msg/String',
                role: 'subscribe',
            },
            (message) => {
                const text = safeToString(message?.data).trim();
                if (!text) {
                    return;
                }
                const entry = {
                    id: uniqueId('transcript'),
                    text,
                    timestamp: new Date().toLocaleTimeString(),
                };
                this.transcripts = [entry, ...this.transcripts].slice(0, MAX_TRANSCRIPTS);
            },
        );
    }

    _openSocket(key, options, handleMessage) {
        this._closeSocket(key);
        try {
            const socket = createTopicSocket(options);
            socket.addEventListener('open', () => {
                if (key === 'audio' && this.audioMonitoringEnabled) {
                    this.audioStatus = 'Live';
                }
            });
            socket.addEventListener('close', () => {
                if (key === 'audio') {
                    this.audioStatus = this.audioMonitoringEnabled ? 'Disconnected' : 'Paused';
                }
            });
            socket.addEventListener('error', (event) => {
                console.warn(`Ear dashboard socket error for ${options.topic}`, event);
                if (key === 'audio') {
                    this.audioStatus = 'Error';
                }
            });
            socket.addEventListener('message', (event) => {
                const payload = this._decodeTopicPayload(event);
                if (!payload) {
                    return;
                }
                handleMessage(payload);
            });
            this._sockets.set(key, socket);
        } catch (error) {
            console.warn(`Ear dashboard failed to open socket for ${options.topic}`, error);
            if (key === 'audio') {
                this.audioStatus = 'Error';
            }
        }
    }

    _closeSocket(key) {
        const socket = this._sockets.get(key);
        if (!socket) {
            return;
        }
        try {
            socket.close();
        } catch (_error) {
            // ignored
        }
        this._sockets.delete(key);
    }

    _teardownAll() {
        for (const key of this._sockets.keys()) {
            this._closeSocket(key);
        }
    }

    _decodeTopicPayload(event) {
        try {
            const payload = JSON.parse(event.data);
            if (!payload || payload.event !== 'message') {
                return null;
            }
            return payload.data;
        } catch (error) {
            console.warn('Ear dashboard failed to parse topic payload', error);
            return null;
        }
    }

    renderSensorIndicator(label, active) {
        return html`
      <li class="sensor-item" data-state="${active ? 'active' : 'idle'}">
        <span>${label}</span>
        <span class="sensor-item__dot" aria-hidden="true"></span>
      </li>
    `;
    }

    render() {
        const speechVariant = this.speechActive ? 'success' : 'muted';
        const silenceVariant = this.silenceDetected ? 'muted' : 'warning';
        const sampleRateLabel = this.audioSampleRate ? `${this.audioSampleRate} Hz` : '—';
        const audioActive = this.audioStatus === 'Live';
        return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card">
          <h3 class="surface-card__title">Stream status</h3>
          <p class="surface-note">Audio stream: <strong>${this.audioStatus}</strong></p>
          <p class="surface-note">Last frame: ${this.lastAudioTimestamp}</p>
          <div class="indicator-row">
            <span class="surface-pill" data-variant=${speechVariant}>
              ${this.speechActive ? 'Speech detected' : 'No speech'}
            </span>
            <span class="surface-pill" data-variant=${silenceVariant}>
              ${this.silenceDetected ? 'Silence' : 'Audio energy'}
            </span>
          </div>
          <div class="surface-actions">
            <button type="button" class="surface-button" @click=${() => this.toggleAudioMonitoring()}>
              ${this.audioMonitoringEnabled ? 'Pause audio monitor' : 'Resume audio monitor'}
            </button>
            <button
              type="button"
              class="surface-button surface-button--ghost"
              @click=${() => this.clearTranscripts()}
              ?disabled=${this.transcripts.length === 0}
            >
              Clear transcript log
            </button>
          </div>
        </article>

        <article class="surface-card">
          <h3 class="surface-card__title">Audio indicators</h3>
          <ul class="sensor-list">
            ${this.renderSensorIndicator('Audio stream', audioActive)}
            ${this.renderSensorIndicator('Speech detected', this.speechActive)}
            ${this.renderSensorIndicator('Audio energy', !this.silenceDetected)}
          </ul>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">PCM oscilloscope</h3>
          <p class="surface-note">Visualises frames from <code>${AUDIO_TOPIC}</code>.</p>
          <div class="oscilloscope-wrapper" data-state=${this.audioRecord ? 'ready' : 'idle'}>
            <pilot-audio-oscilloscope
              width="640"
              height="200"
              .record=${this.audioRecord ?? {}}
            ></pilot-audio-oscilloscope>
            <p class="surface-note surface-mono">
              Sample rate: ${sampleRateLabel} · Frame bytes: ${this.lastFrameByteLength}
            </p>
          </div>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Transcript log</h3>
          ${this.transcripts.length === 0
                ? html`<p class="surface-empty">Awaiting transcripts…</p>`
                : html`<ol class="transcript-log">
                ${this.transcripts.map(
                    (entry) => html`<li class="transcript-entry" key=${entry.id}>
                    <div class="transcript-entry__meta">
                      <span>${entry.timestamp}</span>
                    </div>
                    <p class="transcript-entry__text">${entry.text}</p>
                  </li>`,
                )}
              </ol>`}
        </article>
      </div>
    `;
    }
}

customElements.define('ear-dashboard', EarDashboard);
