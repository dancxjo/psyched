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
import '/components/audio-spectrogram.js';

const AUDIO_TOPIC = '/audio/raw';
const SPEECH_TOPIC = '/ear/speech_active';
const SILENCE_TOPIC = '/ear/silence';
const TRANSCRIPT_TOPIC = '/ear/hole';
const TRANSCRIPT_EVENT_TOPIC = '/ear/asr_event';
const ASR_DEBUG_TOPIC = '/ear/asr_debug';
const MAX_TRANSCRIPTS = 40;
const MAX_SERVICE_MESSAGES = 80;
const MAX_ACTIVITY_HISTORY = 12;
const MAX_ASR_SEGMENTS = 8;

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
    case ASR_DEBUG_TOPIC:
      return 'asr_debug_stream';
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
    speechHistory: { state: true },
    silenceHistory: { state: true },
    speechLastUpdate: { state: true },
    silenceLastUpdate: { state: true },
    speechLastChange: { state: true },
    silenceLastChange: { state: true },
    asrSegments: { state: true },
    serviceMessages: { state: true },
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

      .spectrogram-wrapper {
        display: grid;
        gap: 0.5rem;
      }

      .diagnostic-grid {
        display: grid;
        gap: 0.75rem;
      }

      .diagnostic-grid__rows {
        display: grid;
        gap: 0.75rem;
        grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      }

      .diagnostic-card {
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.65rem;
        display: grid;
        gap: 0.35rem;
        background: rgba(255, 255, 255, 0.02);
      }

      .diagnostic-card[data-state='active'] {
        border-color: rgba(106, 209, 255, 0.6);
        box-shadow: 0 0 12px rgba(106, 209, 255, 0.25);
      }

      .diagnostic-card__label {
        font-size: 0.8rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--lcars-muted);
      }

      .detector-history {
        display: grid;
        gap: 0.5rem;
      }

      .detector-history__columns {
        display: grid;
        gap: 0.65rem;
        grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      }

      .detector-history__list {
        list-style: none;
        margin: 0;
        padding: 0;
        display: grid;
        gap: 0.35rem;
      }

      .detector-history__list li {
        display: flex;
        justify-content: space-between;
        gap: 0.5rem;
        font-size: 0.85rem;
        border-bottom: 1px dashed rgba(255, 255, 255, 0.1);
        padding-bottom: 0.15rem;
      }

      .service-log {
        max-height: 320px;
        overflow-y: auto;
        display: grid;
        gap: 0.5rem;
        padding: 0;
        margin: 0;
        list-style: none;
      }

      .service-log__entry {
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.55rem 0.65rem;
        font-size: 0.85rem;
        display: grid;
        gap: 0.25rem;
        background: rgba(0, 0, 0, 0.25);
      }

      .service-log__entry[data-direction='outbound'] {
        border-color: rgba(165, 129, 255, 0.5);
      }

      .service-log__entry[data-direction='inbound'] {
        border-color: rgba(106, 209, 255, 0.5);
      }

      .service-log__entry header {
        display: flex;
        justify-content: space-between;
        gap: 0.5rem;
        font-size: 0.8rem;
        color: var(--lcars-muted);
      }

      .service-log__entry pre {
        margin: 0;
        font-size: 0.75rem;
        white-space: pre-wrap;
        word-break: break-word;
        background: rgba(0, 0, 0, 0.35);
        border-radius: 0.35rem;
        padding: 0.35rem;
      }

      .asr-segment-list {
        list-style: none;
        margin: 0;
        padding: 0;
        display: grid;
        gap: 0.75rem;
      }

      .asr-segment {
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.75rem;
        display: grid;
        gap: 0.35rem;
        background: rgba(0, 0, 0, 0.15);
      }

      .asr-segment header {
        display: flex;
        justify-content: space-between;
        gap: 0.5rem;
        font-size: 0.9rem;
        flex-wrap: wrap;
      }

      .asr-segment audio {
        width: 100%;
      }

      .asr-segment__meta {
        font-size: 0.8rem;
        color: var(--lcars-muted);
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

      cockpit-audio-spectrogram {
        display: block;
        width: 100%;
      }

      cockpit-audio-spectrogram canvas {
        width: 100%;
        height: auto;
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
    this.speechHistory = [];
    this.silenceHistory = [];
    this.speechLastUpdate = 0;
    this.silenceLastUpdate = 0;
    this.speechLastChange = 0;
    this.silenceLastChange = 0;
    this.asrSegments = [];
    this.serviceMessages = [];
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
    this._activityHistoryLimit = MAX_ACTIVITY_HISTORY;
    this._serviceLogLimit = MAX_SERVICE_MESSAGES;
    this._asrSegmentLimit = MAX_ASR_SEGMENTS;
    this._lastSpeechState = null;
    this._lastSilenceState = null;
    this._asrSegmentUrlMap = new Map();
  }

  connectedCallback() {
    super.connectedCallback();
    this._subscribeSpeech();
    this._subscribeSilence();
    this._subscribeTranscriptEvents();
    this._subscribeTranscripts();
    this._subscribeAsrDebug();
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
    this._clearAsrSegments();
    this.serviceMessages = [];
    this.speechHistory = [];
    this.silenceHistory = [];
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
    this._clearAsrSegments();
  }

  injectFakeTranscript(event) {
    event.preventDefault();
    const text = this.fakeTranscriptText.trim();
    if (!text) {
      this._setFakeTranscriptFeedback('Enter text to inject a transcript line.', 'warning', true);
      return;
    }
    const publisher = this._ensureFakeTranscriptPublisher();
    if (!publisher) {
      this._setFakeTranscriptFeedback('Unable to initialize transcript publisher.', 'error');
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
          const rafHost = typeof globalThis !== 'undefined' ? globalThis : {};
          const schedule =
            typeof rafHost.requestAnimationFrame === 'function'
              ? rafHost.requestAnimationFrame.bind(rafHost)
              : (callback) => setTimeout(callback, 16);
          // Use requestAnimationFrame to align updates with the display
          // refresh; this reduces latency compared to arbitrary timers and
          // allows the oscilloscope element to render smoothly.
          schedule(() => {
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
        const active = Boolean(message?.data);
        this._recordSpeechState(active);
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
        const state =
          typeof message?.data === 'boolean' ? message.data : Boolean(message?.data ?? true);
        this._recordSilenceState(state);
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

  _subscribeAsrDebug() {
    this._openSocket(
      'asrDebug',
      {
        topic: ASR_DEBUG_TOPIC,
        type: 'std_msgs/msg/String',
        role: 'subscribe',
        queueLength: 200,
      },
      (message) => {
        this._handleAsrDebugMessage(message);
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

  _handleAsrDebugMessage(message) {
    const payloadText = typeof message?.data === 'string' ? message.data.trim() : '';
    if (!payloadText) {
      return;
    }
    let payload;
    try {
      payload = JSON.parse(payloadText);
    } catch (error) {
      console.warn('Ear dashboard failed to parse ASR debug payload', error, payloadText);
      return;
    }
    const timestampMs = this._coerceTimestampMs(payload);
    const kind = typeof payload?.kind === 'string' ? payload.kind : 'message';
    const direction = typeof payload?.direction === 'string' ? payload.direction : 'status';
    const entry = {
      id: `${kind}-${timestampMs}-${Math.random().toString(16).slice(2)}`,
      kind,
      direction,
      timestampMs,
      summary: this._summariseServiceDebug(payload),
      detail: this._extractServiceDetail(payload),
    };
    this.serviceMessages = [entry, ...this.serviceMessages].slice(0, this._serviceLogLimit);
  }

  _recordSpeechState(state) {
    const now = Date.now();
    this.speechActive = state;
    this.speechLastUpdate = now;
    if (this._lastSpeechState !== state) {
      this._lastSpeechState = state;
      this.speechLastChange = now;
      const entry = { state, changedAt: now };
      this.speechHistory = [entry, ...this.speechHistory].slice(0, this._activityHistoryLimit);
    }
  }

  _recordSilenceState(state) {
    const now = Date.now();
    this.silenceDetected = state;
    this.silenceLastUpdate = now;
    if (this._lastSilenceState !== state) {
      this._lastSilenceState = state;
      this.silenceLastChange = now;
      const entry = { state, changedAt: now };
      this.silenceHistory = [entry, ...this.silenceHistory].slice(0, this._activityHistoryLimit);
    }
  }

  _coerceTimestampMs(payload) {
    const direct = this._coerceInt(payload?.timestamp_ms ?? payload?.timestampMs);
    if (direct) {
      return direct;
    }
    const seconds = Number(payload?.timestamp);
    if (Number.isFinite(seconds) && seconds > 0) {
      return Math.trunc(seconds * 1000);
    }
    return Date.now();
  }

  _summariseServiceDebug(payload) {
    const kind = typeof payload?.kind === 'string' ? payload.kind : 'message';
    if (kind === 'chunk_sent') {
      const bytes = Number(payload?.byte_length) || 0;
      const rate = Number(payload?.sample_rate) || 0;
      const channels = Number(payload?.channels) || 0;
      return `Chunk sent · ${bytes} bytes @ ${rate || '—'} Hz · ${channels || 1} channel${
        channels === 1 ? '' : 's'
      }`;
    }
    if (kind === 'message_received') {
      const raw = typeof payload?.payload === 'string' ? payload.payload.trim() : '';
      if (raw.startsWith('{')) {
        try {
          const parsed = JSON.parse(raw);
          const event = typeof parsed?.event === 'string' ? parsed.event : 'message';
          return `Received ${event}`;
        } catch {
          return 'Received JSON message';
        }
      }
      if (raw) {
        return `Message: ${raw.slice(0, 80)}${raw.length > 80 ? '…' : ''}`;
      }
      return 'Received message';
    }
    if (kind === 'connected') {
      return `Connected to ${payload?.uri || 'ASR service'}`;
    }
    if (kind === 'disconnected') {
      return 'Connection closed';
    }
    if (kind === 'queue_drained') {
      return 'Audio queue drained';
    }
    if (kind === 'error') {
      const message = typeof payload?.message === 'string' ? payload.message : 'Unhandled error';
      return `Service error: ${message}`;
    }
    if (kind === 'connecting') {
      return `Connecting to ${payload?.uri || 'ASR service'}…`;
    }
    return kind;
  }

  _extractServiceDetail(payload) {
    const kind = typeof payload?.kind === 'string' ? payload.kind : 'message';
    if (kind === 'message_received') {
      const raw = typeof payload?.payload === 'string' ? payload.payload.trim() : '';
      if (!raw) {
        return '';
      }
      return raw.length > 1000 ? `${raw.slice(0, 1000)}…` : raw;
    }
    if (kind === 'error' && typeof payload?.message === 'string') {
      return payload.message;
    }
    return '';
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
    if (finalEntry.audioBase64) {
      this._rememberAsrSegment(finalEntry);
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

  _rememberAsrSegment(entry) {
    const base64 = typeof entry?.audioBase64 === 'string' ? entry.audioBase64.trim() : '';
    if (!base64) {
      return;
    }
    const url = this._createAudioUrlFromBase64(base64);
    if (!url) {
      return;
    }
    this._revokeAsrSegmentUrl(entry.id);
    const segment = {
      id: entry.id || uniqueId('segment'),
      text: entry.text || '',
      timestamp: entry.timestamp || new Date().toLocaleTimeString(),
      startMs: entry.startMs ?? null,
      endMs: entry.endMs ?? null,
      url,
      rangeLabel: this._formatRange(entry.startMs, entry.endMs),
    };
    const filtered = this.asrSegments.filter((item) => item.id !== segment.id);
    const next = [segment, ...filtered].slice(0, this._asrSegmentLimit);
    this._pruneAsrSegmentUrls(next);
    this.asrSegments = next;
    this._asrSegmentUrlMap.set(segment.id, url);
  }

  _createAudioUrlFromBase64(encoded) {
    const decoded = typeof encoded === 'string' ? encoded.trim() : '';
    if (!decoded) {
      return null;
    }
    const scope = typeof globalThis !== 'undefined' ? globalThis : {};
    if (typeof scope.atob !== 'function' || !scope.Blob || !scope.URL?.createObjectURL) {
      return null;
    }
    try {
      const binary = scope.atob(decoded);
      const bytes = new Uint8Array(binary.length);
      for (let i = 0; i < binary.length; i += 1) {
        bytes[i] = binary.charCodeAt(i);
      }
      const blob = new Blob([bytes], { type: 'audio/wav' });
      return scope.URL.createObjectURL(blob);
    } catch (error) {
      console.warn('Ear dashboard failed to decode ASR audio segment', error);
      return null;
    }
  }

  _revokeAsrSegmentUrl(id) {
    if (!id) {
      return;
    }
    const url = this._asrSegmentUrlMap.get(id);
    if (url && globalThis.URL?.revokeObjectURL) {
      try {
        globalThis.URL.revokeObjectURL(url);
      } catch (_) {
        // ignore revoke failures
      }
    }
    this._asrSegmentUrlMap.delete(id);
  }

  _pruneAsrSegmentUrls(nextSegments) {
    const keep = new Set(nextSegments.map((segment) => segment.id));
    for (const [id, url] of this._asrSegmentUrlMap.entries()) {
      if (!keep.has(id) && globalThis.URL?.revokeObjectURL) {
        try {
          globalThis.URL.revokeObjectURL(url);
        } catch (_) {
          // ignore revoke failures
        }
        this._asrSegmentUrlMap.delete(id);
      }
    }
  }

  _clearAsrSegments() {
    for (const url of this._asrSegmentUrlMap.values()) {
      if (globalThis.URL?.revokeObjectURL) {
        try {
          globalThis.URL.revokeObjectURL(url);
        } catch (_) {
          // ignore revoke failures
        }
      }
    }
    this._asrSegmentUrlMap.clear();
    this.asrSegments = [];
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

  _formatTimestamp(ms) {
    if (!Number.isFinite(ms) || ms <= 0) {
      return '—';
    }
    return new Date(ms).toLocaleTimeString();
  }

  _formatElapsed(ms, referenceTime = Date.now()) {
    if (!Number.isFinite(ms) || ms <= 0) {
      return '—';
    }
    const delta = Math.max(0, referenceTime - ms);
    if (delta < 1000) {
      return `${delta.toFixed(0)} ms ago`;
    }
    if (delta < 60_000) {
      return `${(delta / 1000).toFixed(1)} s ago`;
    }
    const minutes = Math.floor(delta / 60_000);
    const seconds = Math.floor((delta % 60_000) / 1000);
    return `${minutes}m ${seconds.toString().padStart(2, '0')}s ago`;
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

  renderDetectorHistoryList(history, type) {
    if (!Array.isArray(history) || history.length === 0) {
      return html`<p class="surface-empty">No transitions yet.</p>`;
    }
    return html`
      <ul class="detector-history__list">
        ${history.map((entry) => {
          const label =
            type === 'speech' ? (entry.state ? 'Speech detected' : 'No speech') : entry.state
              ? 'Silence'
              : 'Audio energy';
          const timestamp = this._formatTimestamp(entry.changedAt);
          const elapsed = this._formatElapsed(entry.changedAt);
          return html`<li>
            <span>${label}</span>
            <span>${timestamp} (${elapsed})</span>
          </li>`;
        })}
      </ul>
    `;
  }

  renderServiceLog() {
    if (!this.serviceMessages.length) {
      return html`<p class="surface-empty">Awaiting service traffic…</p>`;
    }
    return html`
      <ol class="service-log">
        ${this.serviceMessages.map((entry) => html`
          <li class="service-log__entry" data-direction=${entry.direction}>
            <header>
              <span>${this._formatTimestamp(entry.timestampMs)}</span>
              <span>${entry.kind}</span>
            </header>
            <p>${entry.summary}</p>
            ${entry.detail ? html`<pre>${entry.detail}</pre>` : nothing}
          </li>
        `)}
      </ol>
    `;
  }

  renderAsrSegments() {
    if (!this.asrSegments.length) {
      return html`<p class="surface-empty">No ASR segments captured yet.</p>`;
    }
    return html`
      <ol class="asr-segment-list">
        ${this.asrSegments.map((segment, index) => html`
          <li class="asr-segment" key=${segment.id}>
            <header>
              <span>Segment #${this.asrSegments.length - index}</span>
              <span class="asr-segment__meta">${segment.timestamp}</span>
            </header>
            <p>${segment.text || '∅'}</p>
            ${segment.rangeLabel ? html`<p class="asr-segment__meta">${segment.rangeLabel}</p>` : nothing}
            <audio controls preload="metadata" src=${segment.url}></audio>
          </li>
        `)}
      </ol>
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
    const now = Date.now();
    const speechVariant = this.speechActive ? 'success' : 'muted';
    const silenceVariant = this.silenceDetected ? 'muted' : 'warning';
    const sampleRateLabel = this.audioSampleRate ? `${this.audioSampleRate} Hz` : '—';
    const audioActive = this.audioStatus === 'Live';
    const speechUpdateLabel = this._formatTimestamp(this.speechLastUpdate);
    const speechChangeAgo = this._formatElapsed(this.speechLastChange, now);
    const silenceUpdateLabel = this._formatTimestamp(this.silenceLastUpdate);
    const silenceChangeAgo = this._formatElapsed(this.silenceLastChange, now);
    const detectorsAligned = this.speechActive === !this.silenceDetected;
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
                ${this.audioMonitoringEnabled ? '⏸️ Pause audio monitor' : '▶️ Resume audio monitor'}
              </button>
              <button
                type="button"
                class="surface-button surface-button--ghost"
                @click=${() => this.clearTranscripts()}
                ?disabled=${this.transcripts.length === 0}
              >
                🧹 Clear transcript log
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

        <div class="dashboard-row dashboard-row--wide">
          <article class="surface-card">
            <h3 class="surface-card__title">Detector diagnostics</h3>
            <div class="diagnostic-grid">
              <div class="diagnostic-grid__rows">
                <div class="diagnostic-card" data-state=${this.speechActive ? 'active' : 'idle'}>
                  <span class="diagnostic-card__label">Speech detector</span>
                  <strong>${this.speechActive ? 'Speech detected' : 'No speech'}</strong>
                  <span>Last update: ${speechUpdateLabel}</span>
                  <span>Last change: ${speechChangeAgo}</span>
                </div>
                <div class="diagnostic-card" data-state=${!this.silenceDetected ? 'active' : 'idle'}>
                  <span class="diagnostic-card__label">Silence detector</span>
                  <strong>${this.silenceDetected ? 'Silence' : 'Audio energy'}</strong>
                  <span>Last update: ${silenceUpdateLabel}</span>
                  <span>Last change: ${silenceChangeAgo}</span>
                </div>
              </div>
              <p class="surface-note" data-variant=${detectorsAligned ? 'success' : 'warning'}>
                ${detectorsAligned
        ? 'Speech and silence detectors agree.'
        : 'Speech detector disagrees with the silence flag—investigate thresholds.'}
              </p>
              <div class="detector-history">
                <h4>Recent transitions</h4>
                <div class="detector-history__columns">
                  <div>
                    <h5>Speech activity</h5>
                    ${this.renderDetectorHistoryList(this.speechHistory, 'speech')}
                  </div>
                  <div>
                    <h5>Silence detector</h5>
                    ${this.renderDetectorHistoryList(this.silenceHistory, 'silence')}
                  </div>
                </div>
              </div>
            </div>
          </article>

          <article class="surface-card">
            <h3 class="surface-card__title">ASR service log</h3>
            <p class="surface-note">Websocket messages exchanged with the ASR service.</p>
            ${this.renderServiceLog()}
          </article>
        </div>

        <div class="dashboard-row dashboard-row--wide">
          <article class="surface-card">
            <h3 class="surface-card__title">Live spectrogram</h3>
            <p class="surface-note">
              Rolling FFT heatmap from <code>${AUDIO_TOPIC}</code>; oldest columns drop automatically to stay real time.
            </p>
            <div class="spectrogram-wrapper" data-state=${this.audioRecord ? 'ready' : 'idle'}>
              <cockpit-audio-spectrogram
                .height=${240}
                .record=${this.audioRecord ?? null}
                .fftSize=${512}
                .hopSize=${256}
                .minDb=${-100}
                .maxDb=${-20}
              ></cockpit-audio-spectrogram>
              <p class="surface-note surface-mono">
                Sample rate: ${sampleRateLabel} · Frame bytes: ${this.lastFrameByteLength}
              </p>
            </div>
          </article>

          <article class="surface-card">
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
            <h3 class="surface-card__title">ASR audio segments</h3>
            <p class="surface-note">
              Audio attachments returned by the ASR service for each final transcript.
            </p>
            ${this.renderAsrSegments()}
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
                <button type="submit" class="surface-button">📝 Inject transcript</button>
                <button
                  type="button"
                  class="surface-button surface-button--ghost"
                  @click=${() => {
        this.fakeTranscriptText = '';
        this._setFakeTranscriptFeedback('', '');
      }}
                  ?disabled=${this.fakeTranscriptText.trim().length === 0}
                >
                  🧼 Clear input
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
