import { LitElement, html, css, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/cockpit.js';
import { surfaceStyles } from '/components/cockpit-style.js';
import { sampleRateFromMessage } from '../utils/audio.js';
import {
  coerceTranscriptInt,
  createTranscriptDeduplicator,
  normalizeTranscriptEntry,
  transcriptSignature,
} from './ear-dashboard.helpers.js';
import '/components/audio-oscilloscope.js';

const AUDIO_TOPIC = '/audio/raw';
const SPEECH_TOPIC = '/ear/speech_active';
const SILENCE_TOPIC = '/ear/silence';
const TRANSCRIPT_TOPIC = '/ear/hole';
const TRANSCRIPT_EVENT_TOPIC = '/ear/asr_event';
const MAX_TRANSCRIPTS = 40;

function resolveStreamAction(topic) {
  if (typeof topic !== 'string') {
    return null;
  }
  const trimmed = topic.trim();
  switch (trimmed) {
    case AUDIO_TOPIC:
      return 'audio_stream';
    case SPEECH_TOPIC:
      return 'speech_activity_stream';
    case SILENCE_TOPIC:
      return 'silence_stream';
    case TRANSCRIPT_TOPIC:
      return 'transcript_stream';
    default:
      return null;
  }
}

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
    partialTranscript: { state: true },
    fakeTranscriptText: { state: true },
    fakeTranscriptFeedback: { state: true },
    fakeTranscriptFeedbackVariant: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      .dashboard-layout {
        display: grid;
        gap: 0.85rem;
      }

      .dashboard-row {
        display: grid;
        gap: 0.85rem;
        grid-template-columns: minmax(0, 1fr);
      }

      .dashboard-row--wide {
        grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
      }

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

      cockpit-audio-oscilloscope {
        display: block;
        width: 100%;
      }

      cockpit-audio-oscilloscope canvas {
        width: 100%;
        height: auto;
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

      .partial-transcript {
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem 0.75rem;
        background: rgba(255, 255, 255, 0.04);
        display: grid;
        gap: 0.35rem;
      }

      .partial-transcript-region {
        margin-bottom: 0.75rem;
      }

      .partial-transcript header {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.5rem;
      }

      .partial-transcript__label {
        font-size: 0.75rem;
        text-transform: uppercase;
        letter-spacing: 0.05em;
        color: var(--lcars-accent);
      }

      .partial-transcript__timestamp {
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .partial-transcript__text {
        margin: 0;
        font-size: 1.05rem;
        line-height: 1.45;
      }

      .partial-transcript__segments {
        list-style: none;
        margin: 0;
        padding: 0;
        display: grid;
        gap: 0.25rem;
        font-size: 0.8rem;
        color: var(--lcars-muted);
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
        flex-wrap: wrap;
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

      .transcript-form {
        display: grid;
        gap: 0.5rem;
        margin-bottom: 0.85rem;
      }

      .transcript-form label {
        display: grid;
        gap: 0.35rem;
      }

      .transcript-form textarea,
      .transcript-form input {
        width: 100%;
        min-height: 2.75rem;
        border-radius: 0.5rem;
        padding: 0.5rem 0.65rem;
        border: 1px solid var(--control-surface-border);
        background: rgba(0, 0, 0, 0.2);
        color: inherit;
        resize: vertical;
      }

      .transcript-form textarea:focus,
      .transcript-form input:focus {
        outline: 2px solid rgba(106, 209, 255, 0.6);
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
    this.partialTranscript = null;
    this.fakeTranscriptText = '';
    this.fakeTranscriptFeedback = '';
    this.fakeTranscriptFeedbackVariant = '';
    this._sockets = new Map();
    this._publishers = new Map();
    this._latestAudio = null;
    this._audioRenderScheduled = false;
    this._fakeTranscriptFeedbackResetHandle = 0;
    this._lastPartialSignature = '';
    this._deduplicator = createTranscriptDeduplicator(MAX_TRANSCRIPTS * 3);
    this._pendingManualTranscripts = [];
  }

  connectedCallback() {
    super.connectedCallback();
    this._subscribeSpeech();
    this._subscribeSilence();
    this._subscribeTranscriptEvents();
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
    this._clearFakeTranscriptFeedbackTimer();
    this.partialTranscript = null;
    this._lastPartialSignature = '';
    this._deduplicator.clear();
    this._pendingManualTranscripts = [];
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
    this.partialTranscript = null;
    this._lastPartialSignature = '';
    this._deduplicator.clear();
  }

  async injectFakeTranscript(event) {
    event.preventDefault();
    const text = this.fakeTranscriptText.trim();
    if (!text) {
      this._setFakeTranscriptFeedback('Enter text to inject a transcript line.', 'warning', true);
      return;
    }
    const publisher = this._ensureFakeTranscriptPublisher();
    if (!publisher) {
      this._setFakeTranscriptFeedback('Unable to initialise transcript publisher.', 'error');
      return;
    }
    try {
      publisher.send(JSON.stringify({ data: text }));
      this._rememberPendingManualTranscript(text);
      this.fakeTranscriptText = '';
      this._setFakeTranscriptFeedback('Fake transcript injected.', 'success', true);
      this._appendTranscriptEntry(
        {
          id: uniqueId('transcript'),
          text,
          timestamp: new Date().toLocaleTimeString(),
          startMs: null,
          endMs: null,
          segments: [],
          source: 'user',
        },
        this._buildDedupeTokens('manual', text),
      );
      this.partialTranscript = null;
      this._lastPartialSignature = '';
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this._setFakeTranscriptFeedback(`Failed to inject transcript: ${message}`, 'error');
    }
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
        // Coalesce high-frequency audio frames and update the UI at the
        // browser's animation frame rate to avoid excessive re-renders.
        this._latestAudio = message;
        if (!this._audioRenderScheduled) {
          this._audioRenderScheduled = true;
          // Use requestAnimationFrame to align updates with the display
          // refresh; this reduces latency compared to arbitrary timers and
          // allows the oscilloscope element to render smoothly.
          window.requestAnimationFrame(() => {
            const msg = this._latestAudio;
            this._latestAudio = null;
            this._audioRenderScheduled = false;
            if (!msg) {
              return;
            }
            this.audioStatus = 'Live';
            const sampleRate = sampleRateFromMessage(msg, this.audioSampleRate || 16000);
            this.audioSampleRate = sampleRate;
            this.lastFrameByteLength = byteLengthFromMessage(msg);
            this.lastAudioTimestamp = new Date().toLocaleTimeString();
            // Keep the record small — only include the last frame and topic
            // metadata. The oscilloscope component should read this property
            // and render efficiently.
            this.audioRecord = {
              last: msg,
              topic: { name: AUDIO_TOPIC },
            };
          });
        }
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

  _subscribeTranscriptEvents() {
    this._openSocket(
      'transcriptEvents',
      {
        topic: TRANSCRIPT_EVENT_TOPIC,
        type: 'std_msgs/msg/String',
        role: 'subscribe',
      },
      (message) => {
        this._handleTranscriptEvent(message);
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
        this._handleTranscriptMessage(message);
      },
    );
  }

  _handleTranscriptEvent(message) {
    const payload = this._parseTranscriptEventMessage(message);
    if (!payload) {
      return;
    }
    const segments = this._cloneSegments(payload.segments);
    const text = this._normaliseEventText(payload, segments);
    const signature = JSON.stringify({ text, segments });
    const eventName = typeof payload.event === 'string' ? payload.event.trim().toLowerCase() : '';

    if (eventName === 'partial') {
      if (!text && !segments.length) {
        return;
      }
      if (signature === this._lastPartialSignature) {
        return;
      }
      this._lastPartialSignature = signature;
      this.partialTranscript = {
        text,
        segments,
        timestamp: new Date().toLocaleTimeString(),
      };
      return;
    }

    if (eventName === 'final') {
      this._lastPartialSignature = '';
      this.partialTranscript = null;
      if (!text && !segments.length) {
        return;
      }
      const startMs = this._coerceInt(payload.start_ms ?? payload.startMs);
      const endMs = this._coerceInt(payload.end_ms ?? payload.endMs);
      const entry = {
        id: uniqueId('transcript'),
        text,
        timestamp: new Date().toLocaleTimeString(),
        startMs,
        endMs,
        segments,
        audioBase64:
          typeof payload.audio_base64 === 'string'
            ? payload.audio_base64
            : typeof payload.audioBase64 === 'string'
            ? payload.audioBase64
            : null,
        source: typeof payload.source === 'string' ? payload.source : 'event',
      };
      this._appendTranscriptEntry(entry, this._buildDedupeTokens('event', text));
      return;
    }

    if (text) {
      this._appendTranscriptEntry({
        id: uniqueId('transcript'),
        text,
        timestamp: new Date().toLocaleTimeString(),
        startMs: this._coerceInt(payload.start_ms ?? payload.startMs),
        endMs: this._coerceInt(payload.end_ms ?? payload.endMs),
        segments,
        audioBase64:
          typeof payload.audio_base64 === 'string'
            ? payload.audio_base64
            : typeof payload.audioBase64 === 'string'
            ? payload.audioBase64
            : null,
        source: typeof payload.source === 'string' ? payload.source : 'event',
      }, this._buildDedupeTokens('event', text));
    }
  }

  _handleTranscriptMessage(message) {
    const text = safeToString(message?.data ?? message);
    const trimmed = text.trim();
    if (!trimmed) {
      return;
    }
    const manual = this._consumePendingManualTranscript(trimmed);
    const tokens = this._buildDedupeTokens('raw', trimmed);
    this._appendTranscriptEntry(
      {
        id: uniqueId('transcript'),
        text: trimmed,
        timestamp: new Date().toLocaleTimeString(),
        startMs: null,
        endMs: null,
        segments: [],
        source: manual ? 'user' : 'ros',
      },
      tokens,
    );
  }

  _parseTranscriptEventMessage(message) {
    const raw = typeof message?.data === 'string' ? message.data.trim() : '';
    if (!raw) {
      return null;
    }
    try {
      return JSON.parse(raw);
    } catch (error) {
      console.warn('Ear dashboard failed to parse transcript event payload', error, raw);
      return null;
    }
  }

  _cloneSegments(rawSegments) {
    if (!Array.isArray(rawSegments)) {
      return [];
    }
    const segments = [];
    for (const segment of rawSegments) {
      if (!segment || typeof segment !== 'object') {
        continue;
      }
      const text = typeof segment.text === 'string' ? segment.text.trim() : '';
      if (!text) {
        continue;
      }
      const words = [];
      if (Array.isArray(segment.words)) {
        for (const word of segment.words) {
          if (!word || typeof word !== 'object') {
            continue;
          }
          const wordText = typeof word.text === 'string' ? word.text.trim() : '';
          if (!wordText) {
            continue;
          }
          words.push({
            text: wordText,
            startMs: this._coerceInt(word.start_ms ?? word.startMs),
            endMs: this._coerceInt(word.end_ms ?? word.endMs),
          });
        }
      }
      segments.push({
        text,
        startMs: this._coerceInt(segment.start_ms ?? segment.startMs),
        endMs: this._coerceInt(segment.end_ms ?? segment.endMs),
        words,
      });
    }
    return segments;
  }

  _normaliseEventText(payload, segments) {
    if (typeof payload?.text === 'string' && payload.text.trim()) {
      return payload.text.trim();
    }
    if (Array.isArray(segments) && segments.length) {
      const pieces = segments
        .map((segment) => (typeof segment.text === 'string' ? segment.text.trim() : ''))
        .filter((piece) => piece.length > 0);
      if (pieces.length) {
        return pieces.join(' ');
      }
    }
    return '';
  }

  _appendTranscriptEntry(entry, dedupeTokens = []) {
    const normalised = normalizeTranscriptEntry(entry);
    const finalEntry = {
      ...normalised,
      id: normalised.id || entry.id || uniqueId('transcript'),
      timestamp: normalised.timestamp || entry.timestamp || new Date().toLocaleTimeString(),
      startMs: normalised.startMs ?? entry.startMs ?? null,
      endMs: normalised.endMs ?? entry.endMs ?? null,
      segments: Array.isArray(normalised.segments) ? normalised.segments : [],
      source: normalised.source || entry.source || '',
      audioBase64:
        typeof normalised.audioBase64 === 'string'
          ? normalised.audioBase64
          : typeof entry.audioBase64 === 'string'
          ? entry.audioBase64
          : null,
    };
    const tokens = new Set();
    const primarySignature = transcriptSignature(finalEntry);
    if (primarySignature) {
      tokens.add(primarySignature);
    }
    for (const token of dedupeTokens) {
      if (typeof token === 'string' && token.trim()) {
        tokens.add(token.trim());
      }
    }
    for (const token of tokens) {
      if (this._deduplicator.has(token)) {
        return;
      }
    }
    this.transcripts = [finalEntry, ...this.transcripts].slice(0, MAX_TRANSCRIPTS);
    for (const token of tokens) {
      this._deduplicator.remember(token);
    }
  }

  _canonicalText(value) {
    const text = safeToString(value);
    if (!text) {
      return '';
    }
    return text.replace(/\s+/g, ' ').trim();
  }

  _buildDedupeTokens(kind, value) {
    const canonical = this._canonicalText(value);
    if (!canonical) {
      return [];
    }
    if (kind === 'event') {
      return [`event:${canonical}`, `text:${canonical}`];
    }
    if (kind === 'raw') {
      return [`raw:${canonical}`, `text:${canonical}`, `event:${canonical}`];
    }
    if (kind === 'manual') {
      return [`manual:${canonical}`];
    }
    return [`text:${canonical}`];
  }

  _rememberPendingManualTranscript(value) {
    const canonical = this._canonicalText(value);
    if (!canonical) {
      return;
    }
    const now = Date.now();
    this._pendingManualTranscripts = this._pendingManualTranscripts.filter(
      (entry) => entry && entry.expires > now,
    );
    this._pendingManualTranscripts.push({ text: canonical, expires: now + 5000 });
    if (this._pendingManualTranscripts.length > 12) {
      this._pendingManualTranscripts.shift();
    }
  }

  _consumePendingManualTranscript(value) {
    const canonical = this._canonicalText(value);
    if (!canonical) {
      return false;
    }
    const now = Date.now();
    this._pendingManualTranscripts = this._pendingManualTranscripts.filter(
      (entry) => entry && entry.expires > now,
    );
    const index = this._pendingManualTranscripts.findIndex((entry) => entry.text === canonical);
    if (index === -1) {
      return false;
    }
    this._pendingManualTranscripts.splice(index, 1);
    return true;
  }

  _coerceInt(value) {
    return coerceTranscriptInt(value);
  }

  _formatMilliseconds(value) {
    const number = this._coerceInt(value);
    if (number == null) {
      return null;
    }
    const seconds = number / 1000;
    if (!Number.isFinite(seconds)) {
      return null;
    }
    if (seconds < 60) {
      return `${seconds.toFixed(2)} s`;
    }
    const minutes = Math.floor(seconds / 60);
    const remainder = seconds - minutes * 60;
    return `${minutes}m ${remainder.toFixed(2)} s`;
  }

  _formatRange(start, end) {
    const startLabel = this._formatMilliseconds(start);
    const endLabel = this._formatMilliseconds(end);
    if (startLabel && endLabel) {
      return `${startLabel} → ${endLabel}`;
    }
    return startLabel || endLabel || null;
  }

  renderPartialTranscript() {
    if (!this.partialTranscript) {
      return html`<p class="surface-note">No active partial transcript.</p>`;
    }
    const { text, timestamp, segments } = this.partialTranscript;
    const resolvedText = text && text.trim() ? text.trim() : '…';
    return html`
      <div class="partial-transcript" data-state="active">
        <header>
          <span class="partial-transcript__label">Live partial</span>
          <span class="partial-transcript__timestamp">${timestamp || ''}</span>
        </header>
        <p class="partial-transcript__text">${resolvedText}</p>
        ${Array.isArray(segments) && segments.length
          ? html`<ul class="partial-transcript__segments">
              ${segments.map((segment, index) => {
                const rangeLabel = this._formatRange(segment.startMs, segment.endMs);
                return html`<li>
                  <strong>Segment ${index + 1}:</strong>
                  <span>${segment.text || ''}</span>
                  ${rangeLabel ? html`<span>(${rangeLabel})</span>` : nothing}
                </li>`;
              })}
            </ul>`
          : nothing}
      </div>
    `;
  }

  _openSocket(key, options, handleMessage) {
    this._closeSocket(key);
    try {
      const action = options?.action || resolveStreamAction(options?.topic);
      const actionArguments = {};
      if (typeof options?.topic === 'string' && options.topic.trim()) {
        actionArguments.topic = options.topic.trim();
      }
      if (Number.isFinite(options?.queueLength) && options.queueLength > 0) {
        actionArguments.queue_length = Math.floor(options.queueLength);
      }
      const socket = createTopicSocket({
        module: 'ear',
        ...options,
        ...(action
          ? {
              action,
              arguments: Object.keys(actionArguments).length ? actionArguments : undefined,
            }
          : {}),
      });
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
    for (const key of this._publishers.keys()) {
      this._closePublisher(key);
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

  _ensureFakeTranscriptPublisher() {
    const existing = this._publishers.get('fakeTranscript');
    if (existing) {
      return existing;
    }
    let socket;
    try {
      socket = createTopicSocket({
        module: 'ear',
        topic: TRANSCRIPT_TOPIC,
        type: 'std_msgs/msg/String',
        role: 'publish',
      });
    } catch (error) {
      console.warn('Ear dashboard failed to create fake transcript publisher', error);
      return null;
    }
    socket.addEventListener('error', () => {
      this._setFakeTranscriptFeedback('Transcript publisher encountered an error.', 'error');
    });
    socket.addEventListener('close', () => {
      if (this._publishers.get('fakeTranscript') === socket) {
        this._publishers.delete('fakeTranscript');
      }
    });
    this._publishers.set('fakeTranscript', socket);
    return socket;
  }

  _closePublisher(key) {
    const socket = this._publishers.get(key);
    if (!socket) {
      return;
    }
    try {
      socket.close();
    } catch (_error) {
      // ignored
    }
    this._publishers.delete(key);
  }

  _setFakeTranscriptFeedback(message, variant = '', autoClear = false) {
    this.fakeTranscriptFeedback = message;
    this.fakeTranscriptFeedbackVariant = variant;
    if (autoClear) {
      this._scheduleFakeTranscriptFeedbackClear();
    } else {
      this._clearFakeTranscriptFeedbackTimer();
    }
  }

  _scheduleFakeTranscriptFeedbackClear() {
    this._clearFakeTranscriptFeedbackTimer();
    const timerHost = typeof globalThis !== 'undefined' ? globalThis : window;
    if (!timerHost || typeof timerHost.setTimeout !== 'function') {
      return;
    }
    this._fakeTranscriptFeedbackResetHandle = timerHost.setTimeout(() => {
      this.fakeTranscriptFeedback = '';
      this.fakeTranscriptFeedbackVariant = '';
      this._fakeTranscriptFeedbackResetHandle = 0;
    }, 3000);
  }

  _clearFakeTranscriptFeedbackTimer() {
    if (!this._fakeTranscriptFeedbackResetHandle) {
      return;
    }
    const timerHost = typeof globalThis !== 'undefined' ? globalThis : window;
    if (timerHost && typeof timerHost.clearTimeout === 'function') {
      timerHost.clearTimeout(this._fakeTranscriptFeedbackResetHandle);
    }
    this._fakeTranscriptFeedbackResetHandle = 0;
  }

  render() {
    const speechVariant = this.speechActive ? 'success' : 'muted';
    const silenceVariant = this.silenceDetected ? 'muted' : 'warning';
    const sampleRateLabel = this.audioSampleRate ? `${this.audioSampleRate} Hz` : '—';
    const audioActive = this.audioStatus === 'Live';
    return html`
      <div class="dashboard-layout">
        <div class="dashboard-row dashboard-row--wide">
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
        </div>

        <div class="dashboard-row">
          <article class="surface-card surface-card--wide">
            <h3 class="surface-card__title">PCM oscilloscope</h3>
            <p class="surface-note">Visualises frames from <code>${AUDIO_TOPIC}</code>.</p>
            <div class="oscilloscope-wrapper" data-state=${this.audioRecord ? 'ready' : 'idle'}>
              <cockpit-audio-oscilloscope
                height="200"
                .record=${this.audioRecord ?? {}}
              ></cockpit-audio-oscilloscope>
              <p class="surface-note surface-mono">
                Sample rate: ${sampleRateLabel} · Frame bytes: ${this.lastFrameByteLength}
              </p>
            </div>
          </article>
        </div>

        <div class="dashboard-row">
          <article class="surface-card surface-card--wide">
            <h3 class="surface-card__title">Transcript log</h3>
            <form class="transcript-form" @submit=${(event) => this.injectFakeTranscript(event)}>
              <label>
                Inject fake ASR transcript (bleep)
                <input
                  type="text"
                  placeholder="Type a transcript line to append to the log"
                  .value=${this.fakeTranscriptText}
                  @input=${(event) => {
                    this.fakeTranscriptText = event.target.value;
                  }}
                />
              </label>
              <div class="surface-actions">
                <button type="submit" class="surface-button">Inject transcript</button>
                <button
                  type="button"
                  class="surface-button surface-button--ghost"
                  @click=${() => {
        this.fakeTranscriptText = '';
        this._setFakeTranscriptFeedback('', '');
      }}
                  ?disabled=${this.fakeTranscriptText.trim().length === 0}
                >
                  Clear input
                </button>
              </div>
              ${this.fakeTranscriptFeedback
        ? html`<p class="surface-status" data-variant=${this.fakeTranscriptFeedbackVariant || ''}>
                    ${this.fakeTranscriptFeedback}
                  </p>`
        : ''}
            </form>
            <div class="partial-transcript-region">
              ${this.renderPartialTranscript()}
            </div>
            ${this.transcripts.length === 0
        ? html`<p class="surface-empty">Awaiting transcripts…</p>`
        : html`<ol class="transcript-log">
                ${this.transcripts.map((entry) => {
          const rangeLabel = this._formatRange(entry.startMs, entry.endMs);
          const segmentCount = Array.isArray(entry.segments) ? entry.segments.length : 0;
          return html`<li class="transcript-entry" key=${entry.id}>
                    <div class="transcript-entry__meta">
                      <span>${entry.timestamp}</span>
                      ${rangeLabel ? html`<span>${rangeLabel}</span>` : nothing}
                      ${segmentCount
                        ? html`<span>${segmentCount} segment${segmentCount === 1 ? '' : 's'}</span>`
                        : nothing}
                      ${entry.source ? html`<span>${entry.source}</span>` : nothing}
                    </div>
                    <p class="transcript-entry__text">${entry.text}</p>
                  </li>`;
        })}
              </ol>`}
          </article>
        </div>
      </div>
    `;
  }
}

customElements.define('ear-dashboard', EarDashboard);
