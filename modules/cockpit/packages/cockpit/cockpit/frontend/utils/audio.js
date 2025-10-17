/**
 * Audio utility helpers for decoding ROS ByteMultiArray payloads into PCM data.
 *
 * These helpers are intentionally dependency-free so they can be shared between
 * Web Components and any future unit tests.  Examples below assume mono 16 kHz
 * PCM data encoded as signed 16-bit little-endian samples.
 */

/**
 * Attempt to coerce an arbitrary payload into a {@link Uint8Array} view.
 *
 * @param {unknown} message - Raw data from the websocket bridge.
 * @returns {Uint8Array|null} Byte view or `null` when the payload cannot be interpreted.
 *
 * @example
 * const payload = { data: [0, 255, 1, 254] };
 * const bytes = bytesFromMessage(payload);
 * if (bytes) {
 *   console.log(bytes.length); // 4
 * }
 */
export function bytesFromMessage(message) {
  if (message == null) {
    return null;
  }
  if (message instanceof Uint8Array) {
    return message;
  }
  if (message instanceof ArrayBuffer) {
    return new Uint8Array(message);
  }
  if (ArrayBuffer.isView(message)) {
    return new Uint8Array(
      message.buffer.slice(message.byteOffset, message.byteOffset + message.byteLength),
    );
  }
  if (typeof message === 'string') {
    try {
      const decoded = atob(message);
      const bytes = new Uint8Array(decoded.length);
      for (let i = 0; i < decoded.length; i += 1) {
        bytes[i] = decoded.charCodeAt(i) & 0xff;
      }
      return bytes;
    } catch (_) {
      return null;
    }
  }
  if (Array.isArray(message)) {
    const bytes = new Uint8Array(message.length);
    for (let i = 0; i < message.length; i += 1) {
      const value = Number(message[i]);
      if (!Number.isFinite(value)) {
        return null;
      }
      bytes[i] = value & 0xff;
    }
    return bytes;
  }
  if (typeof message === 'object') {
    if (Array.isArray(message.data)) {
      return bytesFromMessage(message.data);
    }
    if (message.type === 'Buffer' && Array.isArray(message.data)) {
      return bytesFromMessage(message.data);
    }
    if (message.data instanceof ArrayBuffer || ArrayBuffer.isView(message.data)) {
      return bytesFromMessage(message.data);
    }
  }
  return null;
}

/**
 * Convert signed 16-bit PCM bytes into normalized floating-point samples.
 *
 * @param {Uint8Array|null} bytes - PCM payload with interleaved 16-bit samples.
 * @param {boolean} [littleEndian=true] - Whether the PCM payload is little endian.
 * @returns {Float32Array} Normalized samples in the range [-1.0, 1.0].
 *
 * @example
 * const pcm = new Uint8Array([0x00, 0x80]); // -32768 in little endian
 * const samples = pcm16ToFloat32(pcm);
 * console.log(samples[0]); // -1
 */
export function pcm16ToFloat32(bytes, littleEndian = true) {
  if (!(bytes instanceof Uint8Array)) {
    return new Float32Array();
  }
  const size = bytes.byteLength - (bytes.byteLength % 2);
  if (size <= 0) {
    return new Float32Array();
  }
  const view = new DataView(bytes.buffer, bytes.byteOffset, size);
  const sampleCount = size / 2;
  const result = new Float32Array(sampleCount);
  for (let i = 0; i < sampleCount; i += 1) {
    const raw = view.getInt16(i * 2, littleEndian);
    // Clamp to [-1, 1] to avoid stray infinities from corrupted payloads.
    const normalized = Math.max(-1, Math.min(1, raw / 32768));
    result[i] = normalized;
  }
  return result;
}

/**
 * Attempt to read a sample rate hint from a payload.
 *
 * @param {unknown} message - Raw message payload delivered by the websocket.
 * @param {number} [fallback=16000] - Default sample rate when no hint is present.
 * @returns {number} A positive sample rate.
 *
 * @example
 * const message = { info: { sample_rate: 22050 } };
 * const rate = sampleRateFromMessage(message);
 * console.log(rate); // 22050
 */
export function sampleRateFromMessage(message, fallback = 16000) {
  const candidates = [];
  if (message && typeof message === 'object') {
    const info = message.info ?? message.metadata ?? {};
    if (typeof info.sample_rate !== 'undefined') {
      candidates.push(info.sample_rate);
    }
    if (typeof message.sample_rate !== 'undefined') {
      candidates.push(message.sample_rate);
    }
    if (Array.isArray(message.layout?.dim)) {
      for (const dim of message.layout.dim) {
        if (dim && typeof dim.label === 'string' && dim.label.toLowerCase().includes('sample')) {
          candidates.push(dim.size);
        }
      }
    }
  }
  for (const candidate of candidates) {
    const rate = Number(candidate);
    if (Number.isFinite(rate) && rate > 0) {
      return rate;
    }
  }
  return fallback;
}

export default {
  bytesFromMessage,
  pcm16ToFloat32,
  sampleRateFromMessage,
};

// Optionally register this helper in the runtime registry so other components
// can access it dynamically via `/utils/registry.js`.
try {
  import('./registry.js').then((r) => {
    try {
      r.exportsify('audio', exports.default || module?.exports);
    } catch (_e) {
      // swallow
    }
  }).catch(() => { });
} catch (_) {
  // ignore; registration is optional
}
