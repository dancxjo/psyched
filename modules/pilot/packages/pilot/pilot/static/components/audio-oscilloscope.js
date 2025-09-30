import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { bytesFromMessage, pcm16ToFloat32 } from '../utils/audio.js';

const DEFAULT_WIDTH = 480;
const DEFAULT_HEIGHT = 160;

/**
 * Canvas-based oscilloscope for visualising PCM audio payloads in real time.
 */
class PilotAudioOscilloscope extends LitElement {
  static properties = {
    record: { type: Object },
    topic: { type: Object },
    width: { type: Number },
    height: { type: Number },
    _samples: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this.topic = null;
    this.width = DEFAULT_WIDTH;
    this.height = DEFAULT_HEIGHT;
    this._samples = new Float32Array();
    this._lastByteLength = 0;
    this._decodeWarningIssued = false;
  }

  createRenderRoot() {
    return this;
  }

  firstUpdated() {
    this._canvas = this.querySelector('canvas');
    this._context = this._canvas?.getContext('2d') ?? null;
    this._renderWaveform();
  }

  updated(changed) {
    if (changed.has('record')) {
      this._samples = this._extractSamples();
      this._renderWaveform();
    }
  }

  _extractSamples() {
    const payload = this.record?.last ?? this.record?.messages?.[0];
    if (!payload) {
      this._lastByteLength = 0;
      return new Float32Array();
    }

    const candidate = payload?.data ?? payload?.bytes ?? payload;
    const bytes = bytesFromMessage(candidate);

    if (!(bytes instanceof Uint8Array) || bytes.byteLength === 0) {
      this._lastByteLength = 0;
      if (!this._decodeWarningIssued) {
        const summary = this._describePayload(payload, candidate);
        console.warn('Audio oscilloscope: unable to decode PCM payload', summary);
        this._decodeWarningIssued = true;
      }
      return new Float32Array();
    }

    this._lastByteLength = bytes.byteLength;
    this._decodeWarningIssued = false;
    return pcm16ToFloat32(bytes);
  }

  _renderWaveform() {
    if (!this._context || !this._canvas) {
      this._canvas = this.querySelector('canvas');
      this._context = this._canvas?.getContext('2d') ?? null;
    }
    if (!this._context || !this._canvas) {
      return;
    }
    const ctx = this._context;
    const { width, height } = this._canvas;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = 'rgba(255, 255, 255, 0.05)';
    ctx.fillRect(0, 0, width, height);
    ctx.strokeStyle = '#6ad1ff';
    ctx.lineWidth = 2;

    const samples = this._samples;
    const sampleCount = samples?.length ?? 0;

    this.dataset.sampleCount = String(sampleCount);
    this.dataset.byteLength = String(this._lastByteLength ?? 0);

    let peak = 0;
    for (let i = 0; i < sampleCount; i += 1) {
      const magnitude = Math.abs(samples[i]);
      if (magnitude > peak) {
        peak = magnitude;
      }
    }
    this.dataset.peak = peak ? peak.toFixed(3) : '0';
    this.dataset.state = sampleCount ? 'ready' : 'empty';

    if (!sampleCount) {
      ctx.font = '14px sans-serif';
      ctx.fillStyle = '#999';
      ctx.fillText('Awaiting audio frames…', 12, height / 2);
      return;
    }

    ctx.beginPath();
    const step = sampleCount / width;
    for (let x = 0; x < width; x += 1) {
      const index = Math.floor(x * step);
      const sample = samples[index] ?? 0;
      const y = (1 - (sample + 1) / 2) * height;
      if (x === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  }

  _describePayload(payload, candidate) {
    const toSummary = (value) => {
      if (value == null) {
        return { type: String(value) };
      }
      if (Array.isArray(value)) {
        return { type: 'Array', length: value.length };
      }
      if (value instanceof ArrayBuffer) {
        return { type: 'ArrayBuffer', byteLength: value.byteLength };
      }
      if (ArrayBuffer.isView(value)) {
        return { type: value.constructor?.name ?? 'TypedArray', length: value.length ?? value.byteLength };
      }
      if (typeof value === 'string') {
        return { type: 'string', length: value.length };
      }
      if (typeof value === 'object') {
        return { type: value.constructor?.name ?? 'Object', keys: Object.keys(value) };
      }
      return { type: typeof value };
    };

    return {
      payload: toSummary(payload),
      candidate: toSummary(candidate),
    };
  }

  render() {
    return html`
      <div class="audio-oscilloscope">
        <canvas width=${this.width} height=${this.height} role="img" aria-label="Audio waveform"></canvas>
      </div>
    `;
  }
}

customElements.define('pilot-audio-oscilloscope', PilotAudioOscilloscope);
