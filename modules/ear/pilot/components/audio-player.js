import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { bytesFromMessage, sampleRateFromMessage } from '../utils/audio.js';

/**
 * Real-time audio player that buffers PCM frames and plays them back
 * using the Web Audio API.
 *
 * This component subscribes to ROS `/audio/raw` topic messages delivered via
 * the websocket bridge, converts PCM16 audio data to Float32, and plays it
 * back through the browser's audio system.
 *
 * Usage:
 * ```html
 * <pilot-audio-player
 *   .record=${audioRecord}
 *   ?autoplay=${false}
 *   bufferDuration="1.0"
 * ></pilot-audio-player>
 * ```
 *
 * @property {Object} record - Audio message record from the websocket
 * @property {Object} topic - Topic metadata (optional)
 * @property {Boolean} autoplay - Start playing automatically when data arrives
 * @property {Number} bufferDuration - Target buffer duration in seconds
 */
class PilotAudioPlayer extends LitElement {
  static properties = {
    record: { type: Object },
    topic: { type: Object },
    autoplay: { type: Boolean },
    bufferDuration: { type: Number },
    _isPlaying: { state: true },
    _isPaused: { state: true },
    _bufferLevel: { state: true },
    _totalFrames: { state: true },
  };

  static styles = css`
    :host {
      display: block;
    }

    .audio-player {
      display: flex;
      flex-direction: column;
      gap: 0.75rem;
      padding: 1rem;
      background: rgba(0, 0, 0, 0.3);
      border: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 0.5rem;
    }

    .controls {
      display: flex;
      gap: 0.5rem;
      align-items: center;
    }

    .control-button {
      padding: 0.5rem 1rem;
      border: 1px solid var(--lcars-accent, #6ad1ff);
      background: rgba(106, 209, 255, 0.1);
      color: var(--lcars-accent, #6ad1ff);
      border-radius: 0.25rem;
      cursor: pointer;
      font-size: 0.875rem;
      font-family: inherit;
      transition: all 0.2s;
    }

    .control-button:hover {
      background: rgba(106, 209, 255, 0.2);
    }

    .control-button:disabled {
      opacity: 0.5;
      cursor: not-allowed;
    }

    .control-button.active {
      background: rgba(106, 209, 255, 0.3);
    }

    .status {
      display: flex;
      flex-direction: column;
      gap: 0.25rem;
      font-size: 0.8rem;
      color: rgba(255, 255, 255, 0.7);
    }

    .status-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .buffer-indicator {
      height: 4px;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 2px;
      overflow: hidden;
      position: relative;
    }

    .buffer-fill {
      height: 100%;
      background: var(--lcars-accent, #6ad1ff);
      transition: width 0.1s ease-out;
    }

    .warning {
      color: #ffaa00;
      font-size: 0.85rem;
    }
  `;

  constructor() {
    super();
    this.record = null;
    this.topic = null;
    this.autoplay = false;
    this.bufferDuration = 1.0; // seconds
    this._isPlaying = false;
    this._isPaused = true;
    this._bufferLevel = 0;
    this._totalFrames = 0;
    this._audioContext = null;
    this._sourceNode = null;
    this._gainNode = null;
    this._pcmBuffer = [];
    this._sampleRate = 16000;
    this._channels = 1;
    this._playbackStartTime = 0;
    this._scheduledUntil = 0;
  }

  createRenderRoot() {
    return this;
  }

  connectedCallback() {
    super.connectedCallback();
    this._initAudioContext();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._cleanupAudioContext();
  }

  updated(changed) {
    if (changed.has('record')) {
      this._processAudioFrame();
    }
  }

  _initAudioContext() {
    if (this._audioContext) {
      return;
    }
    try {
      const AudioContext = window.AudioContext || window.webkitAudioContext;
      this._audioContext = new AudioContext();
      this._gainNode = this._audioContext.createGain();
      this._gainNode.connect(this._audioContext.destination);
      this._gainNode.gain.value = 0.8;
    } catch (error) {
      console.error('Failed to initialize audio context:', error);
    }
  }

  _cleanupAudioContext() {
    if (this._sourceNode) {
      try {
        this._sourceNode.stop();
      } catch (_) {
        // ignore
      }
      this._sourceNode = null;
    }
    if (this._audioContext) {
      try {
        this._audioContext.close();
      } catch (_) {
        // ignore
      }
      this._audioContext = null;
    }
  }

  _processAudioFrame() {
    const payload = this.record?.last ?? this.record?.messages?.[0];
    if (!payload) {
      return;
    }

    const candidate = payload?.data ?? payload?.bytes ?? payload;
    const bytes = bytesFromMessage(candidate);

    if (!(bytes instanceof Uint8Array) || bytes.byteLength === 0) {
      return;
    }

    // Extract sample rate from message if available
    const detectedRate = sampleRateFromMessage(payload, this._sampleRate);
    if (detectedRate !== this._sampleRate) {
      this._sampleRate = detectedRate;
    }

    // Convert PCM16 to float32
    const samples = this._pcm16ToFloat32(bytes);
    if (samples.length === 0) {
      return;
    }

    this._pcmBuffer.push(samples);
    this._totalFrames++;

    // Update buffer level indicator
    const bufferedSamples = this._pcmBuffer.reduce((sum, frame) => sum + frame.length, 0);
    const bufferedSeconds = bufferedSamples / this._sampleRate;
    this._bufferLevel = Math.min(100, (bufferedSeconds / this.bufferDuration) * 100);

    // Auto-play if enabled and context is ready
    if (this.autoplay && !this._isPlaying && this._audioContext) {
      this.play();
    }

    // Schedule playback if playing
    if (this._isPlaying && this._audioContext) {
      this._scheduleNextChunk();
    }
  }

  _pcm16ToFloat32(bytes, littleEndian = true) {
    const size = bytes.byteLength - (bytes.byteLength % 2);
    if (size <= 0) {
      return new Float32Array();
    }
    const view = new DataView(bytes.buffer, bytes.byteOffset, size);
    const sampleCount = size / 2;
    const result = new Float32Array(sampleCount);
    for (let i = 0; i < sampleCount; i += 1) {
      const raw = view.getInt16(i * 2, littleEndian);
      result[i] = Math.max(-1, Math.min(1, raw / 32768));
    }
    return result;
  }

  _scheduleNextChunk() {
    if (!this._audioContext || this._pcmBuffer.length === 0) {
      return;
    }

    const currentTime = this._audioContext.currentTime;

    // Initialize scheduling if needed
    if (this._scheduledUntil === 0) {
      this._scheduledUntil = currentTime;
    }

    // Schedule chunks to stay ahead
    const lookAhead = 0.1; // seconds
    while (this._pcmBuffer.length > 0 && this._scheduledUntil < currentTime + lookAhead) {
      const samples = this._pcmBuffer.shift();
      if (!samples || samples.length === 0) {
        continue;
      }

      const buffer = this._audioContext.createBuffer(this._channels, samples.length, this._sampleRate);
      buffer.getChannelData(0).set(samples);

      const source = this._audioContext.createBufferSource();
      source.buffer = buffer;
      source.connect(this._gainNode);
      source.start(this._scheduledUntil);

      const duration = samples.length / this._sampleRate;
      this._scheduledUntil += duration;
    }
  }

  play() {
    if (!this._audioContext) {
      this._initAudioContext();
    }
    if (!this._audioContext) {
      console.warn('Audio context not available');
      return;
    }

    // Resume audio context if suspended (browser autoplay policy)
    if (this._audioContext.state === 'suspended') {
      this._audioContext.resume().catch((error) => {
        console.error('Failed to resume audio context:', error);
      });
    }

    this._isPlaying = true;
    this._isPaused = false;
    this._playbackStartTime = this._audioContext.currentTime;
    this._scheduleNextChunk();
  }

  pause() {
    this._isPlaying = false;
    this._isPaused = true;
    this._scheduledUntil = 0;
  }

  clear() {
    this._pcmBuffer = [];
    this._scheduledUntil = 0;
    this._bufferLevel = 0;
    this._totalFrames = 0;
  }

  render() {
    const statusLabel = this._isPlaying ? 'Playing' : this._isPaused ? 'Paused' : 'Ready';
    const hasBuffer = this._pcmBuffer.length > 0;

    return html`
      <div class="audio-player">
        <div class="controls">
          <button
            type="button"
            class="control-button ${this._isPlaying ? 'active' : ''}"
            @click=${() => this.play()}
            ?disabled=${this._isPlaying}
          >
            ▶ Play
          </button>
          <button
            type="button"
            class="control-button"
            @click=${() => this.pause()}
            ?disabled=${!this._isPlaying}
          >
            ⏸ Pause
          </button>
          <button type="button" class="control-button" @click=${() => this.clear()}>
            🗑 Clear
          </button>
        </div>

        <div class="status">
          <div class="status-row">
            <span>Status: ${statusLabel}</span>
            <span>Frames: ${this._totalFrames}</span>
          </div>
          <div class="status-row">
            <span>Sample rate: ${this._sampleRate} Hz</span>
            <span>Buffered: ${hasBuffer ? this._pcmBuffer.length : 0} chunks</span>
          </div>
        </div>

        <div class="buffer-indicator">
          <div class="buffer-fill" style="width: ${this._bufferLevel}%"></div>
        </div>

        ${!this._audioContext || this._audioContext.state === 'suspended'
          ? html`<p class="warning">⚠ Click "Play" to activate audio playback</p>`
          : ''}
      </div>
    `;
  }
}

customElements.define('pilot-audio-player', PilotAudioPlayer);
