import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { bytesFromMessage, pcm16ToFloat32, sampleRateFromMessage } from '../utils/audio.js';

const DEFAULT_WIDTH = 480;
const DEFAULT_HEIGHT = 240;
const DEFAULT_FFT_SIZE = 512;
const DEFAULT_HOP_SIZE = 256;
const MAX_QUEUE_MULTIPLIER = 8;
const COLUMN_PIXEL_WIDTH = 1;
const globalScope = typeof globalThis !== 'undefined' ? globalThis : {};

/**
 * Lightweight PCM spectrogram renderer.
 *
 * The component consumes UInt8MultiArray payloads from the cockpit websocket
 * bridge, decodes the PCM frames, and renders a rolling frequency heatmap.
 * Incoming audio is reduced as it arrives to avoid buffering large queues,
 * keeping latency low even when frames burst.
 */
class CockpitAudioSpectrogram extends LitElement {
  static properties = {
    record: { type: Object },
    topic: { type: Object },
    width: { type: Number },
    height: { type: Number },
    fftSize: { type: Number, attribute: 'fft-size' },
    hopSize: { type: Number, attribute: 'hop-size' },
    minDb: { type: Number, attribute: 'min-db' },
    maxDb: { type: Number, attribute: 'max-db' },
  };

  constructor() {
    super();
    this.record = null;
    this.topic = null;
    this.width = DEFAULT_WIDTH;
    this.height = DEFAULT_HEIGHT;
    this.fftSize = DEFAULT_FFT_SIZE;
    this.hopSize = DEFAULT_HOP_SIZE;
    this.minDb = -100;
    this.maxDb = -20;
    this.sampleRate = 16000;

    this._sampleQueue = [];
    this._processingScheduled = false;
    this._renderRequested = false;
    this._resizeObserver = null;
    this._hannWindow = this._buildHannWindow(this.fftSize);
    this._colorLut = this._buildColorLut();
    if (typeof document !== 'undefined') {
      this._historyCanvas = document.createElement('canvas');
      this._historyCanvas.width = DEFAULT_WIDTH;
      this._historyCanvas.height = DEFAULT_HEIGHT;
      this._historyCtx = this._historyCanvas.getContext('2d');
      if (this._historyCtx) {
        this._historyCtx.imageSmoothingEnabled = false;
      }
    } else {
      this._historyCanvas = null;
      this._historyCtx = null;
    }
    this._canvas = null;
    this._context = null;
    this._historyColumns = 0;
    this._lastColumnTimestamp = 0;
  }

  createRenderRoot() {
    return this;
  }

  firstUpdated() {
    this._canvas = this.querySelector('canvas');
    this._context = this._canvas?.getContext('2d') ?? null;
    if (!this._historyCanvas && typeof document !== 'undefined') {
      this._historyCanvas = document.createElement('canvas');
      this._historyCtx = this._historyCanvas.getContext('2d');
    }
    this._observeResize();
    this._syncCanvasDimensions();
    this._resetHistory();
    this._requestRender(true);
  }

  disconnectedCallback() {
    if (this._resizeObserver) {
      this._resizeObserver.disconnect();
      this._resizeObserver = null;
    }
    super.disconnectedCallback();
  }

  updated(changed) {
    if (changed.has('fftSize')) {
      this._normalizeFftSize();
      this._hannWindow = this._buildHannWindow(this.fftSize);
    }
    if (changed.has('hopSize')) {
      this._normalizeHopSize();
    }
    if (changed.has('width') || changed.has('height')) {
      this._syncCanvasDimensions();
      this._resetHistory();
      this._requestRender(true);
    }
    if (changed.has('record')) {
      this._handleRecord();
    }
  }

  _observeResize() {
    const ResizeObserverCtor = globalScope.ResizeObserver;
    if (typeof ResizeObserverCtor === 'undefined') {
      return;
    }
    if (this._resizeObserver) {
      return;
    }
    this._resizeObserver = new ResizeObserverCtor((entries) => {
      for (const entry of entries) {
        if (entry.target !== this) {
          continue;
        }
        const nextWidth = Math.floor(entry.contentRect?.width ?? 0);
        if (!nextWidth) {
          continue;
        }
        if (nextWidth !== this.width) {
          this.width = nextWidth;
        }
      }
    });
    this._resizeObserver.observe(this);
  }

  _syncCanvasDimensions() {
    if (!this._canvas) {
      this._canvas = this.querySelector('canvas');
    }
    if (!this._canvas) {
      return;
    }
    const pixelRatio = typeof globalScope.devicePixelRatio === 'number' ? globalScope.devicePixelRatio : 1;
    const width = Math.max(32, Math.floor(this.width || DEFAULT_WIDTH));
    const height = Math.max(64, Math.floor(this.height || DEFAULT_HEIGHT));
    if (this._canvas.width !== width * pixelRatio) {
      this._canvas.width = width * pixelRatio;
    }
    if (this._canvas.height !== height * pixelRatio) {
      this._canvas.height = height * pixelRatio;
    }
    this._canvas.style.width = `${width}px`;
    this._canvas.style.height = `${height}px`;
    if (this._historyCanvas) {
      if (this._historyCanvas.width !== width) {
        this._historyCanvas.width = width;
      }
      if (this._historyCanvas.height !== height) {
        this._historyCanvas.height = height;
      }
    }
  }

  _resetHistory() {
    if (!this._historyCtx || !this._historyCanvas) {
      return;
    }
    this._historyCtx.imageSmoothingEnabled = false;
    this._historyCtx.fillStyle = 'rgba(0, 0, 0, 1)';
    this._historyCtx.fillRect(0, 0, this._historyCanvas.width, this._historyCanvas.height);
    this._historyColumns = 0;
  }

  _handleRecord() {
    if (!this.record) {
      return;
    }
    const payload = this.record.last ?? this.record.messages?.[0];
    if (!payload) {
      return;
    }
    const source = payload.data ?? payload.bytes ?? payload;
    const bytes = bytesFromMessage(source);
    if (!(bytes instanceof Uint8Array) || bytes.byteLength === 0) {
      return;
    }
    this.sampleRate = sampleRateFromMessage(payload, this.sampleRate || 16000);
    const samples = pcm16ToFloat32(bytes);
    if (!samples.length) {
      return;
    }
    this._appendSamples(samples);
    this.dataset.sampleRate = String(this.sampleRate);
    this.dataset.byteLength = String(bytes.byteLength);
    this._scheduleProcessing();
  }

  _appendSamples(samples) {
    if (!samples?.length) {
      return;
    }
    for (let i = 0; i < samples.length; i += 1) {
      this._sampleQueue.push(samples[i]);
    }
    const maxQueue = this.fftSize * MAX_QUEUE_MULTIPLIER;
    if (this._sampleQueue.length > maxQueue) {
      // Drop the oldest samples to keep latency low.
      this._sampleQueue.splice(0, this._sampleQueue.length - maxQueue);
    }
  }

  _scheduleProcessing() {
    if (this._processingScheduled) {
      return;
    }
    this._processingScheduled = true;
    const callback = () => {
      this._processingScheduled = false;
      this._drainSampleQueue();
    };
    if (typeof globalScope.requestAnimationFrame === 'function') {
      globalScope.requestAnimationFrame(callback);
    } else {
      setTimeout(callback, 0);
    }
  }

  _drainSampleQueue() {
    if (this._sampleQueue.length < this.fftSize) {
      return;
    }
    while (this._sampleQueue.length >= this.fftSize) {
      const frame = new Float32Array(this.fftSize);
      for (let i = 0; i < this.fftSize; i += 1) {
        frame[i] = this._sampleQueue[i];
      }
      const hop = Math.min(this.hopSize, this._sampleQueue.length);
      this._sampleQueue.splice(0, hop);
      const magnitudes = this._computeMagnitudes(frame);
      this._ingestColumn(magnitudes);
    }
  }

  _ingestColumn(magnitudes) {
    if (!magnitudes?.length || !this._historyCtx) {
      return;
    }
    this._scrollHistory();
    this._paintHistoryColumn(magnitudes);
    this._historyColumns = Math.min(this._historyColumns + 1, this._historyCanvas?.width ?? 0);
    const perf = typeof performance !== 'undefined' ? performance : null;
    this._lastColumnTimestamp = perf?.now ? perf.now() : Date.now();
    this.dataset.columns = String(this._historyColumns);
    this.dataset.updatedAt = String(Math.floor(this._lastColumnTimestamp));
    this._requestRender();
  }

  _scrollHistory() {
    if (!this._historyCanvas || !this._historyCtx) {
      return;
    }
    const width = this._historyCanvas.width;
    if (!width) {
      return;
    }
    this._historyCtx.drawImage(this._historyCanvas, -COLUMN_PIXEL_WIDTH, 0);
    this._historyCtx.fillStyle = 'rgba(0, 0, 0, 1)';
    this._historyCtx.fillRect(width - COLUMN_PIXEL_WIDTH, 0, COLUMN_PIXEL_WIDTH, this._historyCanvas.height);
  }

  _paintHistoryColumn(magnitudes) {
    if (!this._historyCanvas || !this._historyCtx) {
      return;
    }
    const height = this._historyCanvas.height;
    const width = this._historyCanvas.width;
    if (!height || !width) {
      return;
    }
    const binCount = magnitudes.length;
    if (!binCount) {
      return;
    }
    const binHeight = height / binCount;
    const x = width - COLUMN_PIXEL_WIDTH;
    for (let index = 0; index < binCount; index += 1) {
      const value = magnitudes[index];
      const clamped = Math.max(0, Math.min(1, value));
      const y = Math.floor(height - (index + 1) * binHeight);
      const size = Math.max(1, Math.ceil(binHeight));
      this._historyCtx.fillStyle = this._colorFromValue(clamped);
      this._historyCtx.fillRect(x, y, COLUMN_PIXEL_WIDTH, size);
    }
  }

  _requestRender(force = false) {
    if (this._renderRequested && !force) {
      return;
    }
    this._renderRequested = true;
    const callback = () => {
      this._renderRequested = false;
      this._renderSpectrogram();
    };
    if (typeof globalScope.requestAnimationFrame === 'function') {
      globalScope.requestAnimationFrame(callback);
    } else {
      setTimeout(callback, 16);
    }
  }

  _renderSpectrogram() {
    if (!this._context || !this._canvas) {
      return;
    }
    const ctx = this._context;
    const canvasWidth = this._canvas.width;
    const canvasHeight = this._canvas.height;
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    if (!this._historyColumns) {
      this._renderFallback();
      return;
    }
    ctx.drawImage(
      this._historyCanvas,
      0,
      0,
      this._historyCanvas.width,
      this._historyCanvas.height,
      0,
      0,
      canvasWidth,
      canvasHeight,
    );
  }

  _renderFallback() {
    if (!this._context || !this._canvas) {
      return;
    }
    const { width, height } = this._canvas;
    this._context.fillStyle = 'rgba(0, 0, 0, 1)';
    this._context.fillRect(0, 0, width, height);
    this._context.fillStyle = '#777';
    this._context.font = `${Math.max(12, Math.floor(height / 16))}px sans-serif`;
    this._context.textBaseline = 'middle';
    this._context.fillText('Awaiting audio frames…', 18, height / 2);
  }

  _computeMagnitudes(frame) {
    const windowed = new Float32Array(frame.length);
    const hann = this._hannWindow;
    for (let i = 0; i < frame.length; i += 1) {
      const multiplier = hann?.[i] ?? 1;
      windowed[i] = frame[i] * multiplier;
    }
    const { real, imag } = this._fft(windowed);
    const binCount = real.length / 2;
    const magnitudes = new Float32Array(binCount);
    const scale = 1 / (frame.length / 2);
    for (let bin = 0; bin < binCount; bin += 1) {
      const re = real[bin];
      const im = imag[bin];
      const magnitude = Math.sqrt(re * re + im * im) * scale;
      const db = 20 * Math.log10(magnitude + 1e-7);
      const normalized = (db - this.minDb) / (this.maxDb - this.minDb);
      magnitudes[bin] = Math.max(0, Math.min(1, normalized));
    }
    return magnitudes;
  }

  _fft(input) {
    const n = input.length;
    const real = new Float32Array(input);
    const imag = new Float32Array(n);
    // Bit-reversal permutation
    for (let i = 1, j = 0; i < n; i += 1) {
      let bit = n >> 1;
      for (; j & bit; bit >>= 1) {
        j ^= bit;
      }
      j ^= bit;
      if (i < j) {
        const tempReal = real[i];
        real[i] = real[j];
        real[j] = tempReal;
        const tempImag = imag[i];
        imag[i] = imag[j];
        imag[j] = tempImag;
      }
    }

    for (let len = 2; len <= n; len <<= 1) {
      const halfLen = len >> 1;
      const angle = -2 * Math.PI / len;
      const wLenReal = Math.cos(angle);
      const wLenImag = Math.sin(angle);
      for (let start = 0; start < n; start += len) {
        let wReal = 1;
        let wImag = 0;
        for (let j = 0; j < halfLen; j += 1) {
          const uReal = real[start + j];
          const uImag = imag[start + j];
          const vReal = real[start + j + halfLen];
          const vImag = imag[start + j + halfLen];

          const tempReal = vReal * wReal - vImag * wImag;
          const tempImag = vReal * wImag + vImag * wReal;

          real[start + j] = uReal + tempReal;
          imag[start + j] = uImag + tempImag;
          real[start + j + halfLen] = uReal - tempReal;
          imag[start + j + halfLen] = uImag - tempImag;

          const nextWReal = wReal * wLenReal - wImag * wLenImag;
          const nextWImag = wReal * wLenImag + wImag * wLenReal;
          wReal = nextWReal;
          wImag = nextWImag;
        }
      }
    }
    return { real, imag };
  }

  _normalizeFftSize() {
    let size = Number(this.fftSize);
    if (!Number.isFinite(size) || size < 32) {
      size = DEFAULT_FFT_SIZE;
    }
    // Round to the nearest power of two.
    size = 2 ** Math.round(Math.log2(size));
    this.fftSize = Math.max(64, size);
  }

  _normalizeHopSize() {
    let hop = Number(this.hopSize);
    if (!Number.isFinite(hop) || hop <= 0) {
      hop = DEFAULT_HOP_SIZE;
    }
    this.hopSize = Math.min(Math.max(16, hop), this.fftSize);
  }

  _buildHannWindow(size) {
    const window = new Float32Array(size);
    for (let i = 0; i < size; i += 1) {
      window[i] = 0.5 * (1 - Math.cos((2 * Math.PI * i) / (size - 1)));
    }
    return window;
  }

  _buildColorLut() {
    const table = new Array(256);
    for (let i = 0; i < 256; i += 1) {
      const t = i / 255;
      const hue = 240 - (t * 240);
      const saturation = 85;
      const lightness = 12 + (t * 50);
      table[i] = `hsl(${hue.toFixed(1)}, ${saturation}%, ${lightness}%)`;
    }
    return table;
  }

  _colorFromValue(value) {
    const index = Math.max(0, Math.min(255, Math.round(value * 255)));
    return this._colorLut[index];
  }

  render() {
    return html`
      <div class="audio-spectrogram">
        <canvas width=${this.width} height=${this.height} role="img" aria-label="Audio spectrogram"></canvas>
      </div>
    `;
  }
}

customElements.define('cockpit-audio-spectrogram', CockpitAudioSpectrogram);
