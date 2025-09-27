import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { bytesFromMessage, sampleRateFromMessage } from '../utils/audio.js';

const MAX_CLIPS = 10;

/**
 * Render a scrollable list of audio segments with WAV playback controls.
 *
 * The widget expects ByteMultiArray-style ROS messages delivered via the pilot
 * websocket bridge.  Each clip is converted to a WAV blob so operators can
 * review the captured PCM directly in the browser.
 */
class PilotAudioWaveform extends LitElement {
  static properties = {
    record: { type: Object },
    topic: { type: Object },
    _clips: { state: true },
    _error: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this.topic = null;
    this._clips = [];
    this._error = '';
  }

  createRenderRoot() {
    return this;
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._releaseClips();
  }

  updated(changed) {
    if (changed.has('record')) {
      this._rebuildClips();
    }
  }

  _releaseClips() {
    for (const clip of this._clips) {
      if (clip?.url) {
        try {
          URL.revokeObjectURL(clip.url);
        } catch (error) {
          console.warn('Failed to release audio URL', error);
        }
      }
    }
    this._clips = [];
  }

  _rebuildClips() {
    const messages = Array.isArray(this.record?.messages) ? this.record.messages : [];
    if (!messages.length) {
      this._releaseClips();
      this._error = '';
      this.requestUpdate();
      return;
    }

    const priorUrls = this._clips.map((clip) => clip.url).filter(Boolean);
    const clips = [];
    const total = messages.length;
    const limit = Math.min(MAX_CLIPS, total);
    for (let index = 0; index < limit; index += 1) {
      const message = messages[index];
      const clip = this._buildClip(message, index, total);
      if (clip) {
        clips.push(clip);
      }
    }

    for (const url of priorUrls) {
      try {
        URL.revokeObjectURL(url);
      } catch (error) {
        console.warn('Failed to revoke audio blob URL', error);
      }
    }

    this._clips = clips;
    this._error = clips.length === 0 ? 'Audio payload unavailable' : '';
    this.requestUpdate();
  }

  _buildClip(message, index, total) {
    const payload = message?.data ?? message?.bytes ?? message;
    const bytes = bytesFromMessage(payload);
    if (!(bytes?.length)) {
      return null;
    }
    const sampleRate = sampleRateFromMessage(message, 16000);
    const duration = sampleRate > 0 ? (bytes.byteLength / 2) / sampleRate : 0;
    const url = this._createWavUrl(bytes, sampleRate);
    const labelIndex = total - index;
    return {
      index,
      url,
      duration,
      sampleRate,
      size: bytes.byteLength,
      label: `Segment #${labelIndex}`,
    };
  }

  _createWavUrl(bytes, sampleRate) {
    const header = new ArrayBuffer(44);
    const view = new DataView(header);

    this._writeString(view, 0, 'RIFF');
    view.setUint32(4, 36 + bytes.byteLength, true);
    this._writeString(view, 8, 'WAVE');
    this._writeString(view, 12, 'fmt ');
    view.setUint32(16, 16, true);
    view.setUint16(20, 1, true);
    view.setUint16(22, 1, true);
    view.setUint32(24, sampleRate, true);
    view.setUint32(28, sampleRate * 2, true);
    view.setUint16(32, 2, true);
    view.setUint16(34, 16, true);
    this._writeString(view, 36, 'data');
    view.setUint32(40, bytes.byteLength, true);

    const blob = new Blob([header, bytes], { type: 'audio/wav' });
    return URL.createObjectURL(blob);
  }

  _writeString(view, offset, text) {
    for (let i = 0; i < text.length; i += 1) {
      view.setUint8(offset + i, text.charCodeAt(i));
    }
  }

  _renderClip(clip) {
    const seconds = clip.duration > 0 ? clip.duration.toFixed(2) : '0.00';
    const sizeKb = (clip.size / 1024).toFixed(1);
    return html`
      <li class="audio-clip">
        <header>
          <span class="clip-label">${clip.label}</span>
          <span class="clip-meta">${seconds}s · ${clip.sampleRate} Hz · ${sizeKb} KiB</span>
        </header>
        <audio controls preload="metadata" src=${clip.url}></audio>
        <div class="clip-actions">
          <a href=${clip.url} download=${`pcm-segment-${clip.index + 1}.wav`}>Download WAV</a>
        </div>
      </li>
    `;
  }

  render() {
    if (!this.record) {
      return html`<div class="inactive">Not subscribed</div>`;
    }
    if (this._error) {
      return html`<div class="audio-waveform"><p class="empty">${this._error}</p></div>`;
    }
    if (!this._clips.length) {
      return html`<div class="audio-waveform"><p class="empty">Awaiting audio segments…</p></div>`;
    }
    return html`
      <div class="audio-waveform">
        <ol>
          ${this._clips.map((clip) => this._renderClip(clip))}
        </ol>
      </div>
    `;
  }
}

customElements.define('pilot-audio-waveform', PilotAudioWaveform);
