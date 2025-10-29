/**
 * Utilities for sanitising Ear module dashboard form submissions.
 *
 * These helpers keep the Lit component implementation focused on rendering
 * and event wiring while providing deterministic, easily testable logic for
 * request payload construction.
 */

const SUPPORTED_BACKENDS = new Set(['console', 'faster_whisper', 'service']);

/**
 * Normalise the backend identifier selected by the operator.
 *
 * @param {string | null | undefined} value Raw backend name from a form control.
 * @returns {string} Lowercase backend identifier that defaults to ``console``.
 */
export function normalizeBackend(value) {
  const slug = String(value ?? '').trim().toLowerCase();
  if (SUPPORTED_BACKENDS.has(slug)) {
    return slug;
  }
  return 'console';
}

/**
 * Clamp an integer into an acceptable range.
 *
 * @param {number | string | null | undefined} value Candidate numeric value.
 * @param {{ min: number, max: number, defaultValue: number }} options Bounds and default.
 * @returns {number} Integer constrained within ``options.min`` and ``options.max``.
 */
export function clampInteger(value, options) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) {
    return Math.trunc(options.defaultValue);
  }
  const integer = Math.trunc(parsed);
  if (integer < options.min) {
    return options.min;
  }
  if (integer > options.max) {
    return options.max;
  }
  return integer;
}

/**
 * Build a normalised configuration payload for the Ear backend.
 *
 * The helper validates integer fields and removes empty optional values so the
 * backend only receives meaningful configuration options.
 *
 * @param {object} draft Raw values captured from the dashboard form.
 * @param {string} draft.backend Selected backend identifier.
 * @param {string} [draft.serviceUri] Optional URI for the streaming backend.
 * @param {string} [draft.language] Optional transcription language hint.
 * @param {number|string} [draft.beamSize] Beam search width for Whisper models.
 * @param {number|string} [draft.sampleRate] Audio sample rate hint in hertz.
 * @param {number|string} [draft.channels] Number of PCM channels being streamed.
 * @returns {{ ok: true, value: EarConfigPayload } | { ok: false, error: string }}
 */
export function buildEarConfigPayload(draft) {
  const backend = normalizeBackend(draft.backend);
  const sampleRate = clampInteger(draft.sampleRate, {
    min: 8_000,
    max: 192_000,
    defaultValue: 16_000,
  });
  if (sampleRate < 8_000 || sampleRate > 192_000) {
    return { ok: false, error: 'Sample rate must be between 8 kHz and 192 kHz.' };
  }
  const channels = clampInteger(draft.channels, {
    min: 1,
    max: 8,
    defaultValue: 1,
  });
  if (channels < 1 || channels > 8) {
    return { ok: false, error: 'Channels must be between 1 and 8.' };
  }
  const payload = {
    backend,
    sample_rate: sampleRate,
    channels,
    backend_options: {},
  };

  const language = String(draft.language ?? '').trim();
  if (language) {
    payload.backend_options.language = language;
  }

  const beamSize = clampInteger(draft.beamSize, {
    min: 1,
    max: 20,
    defaultValue: 5,
  });
  payload.backend_options.beam_size = beamSize;

  if (backend === 'service') {
    const serviceUri = String(draft.serviceUri ?? '').trim();
    if (!serviceUri) {
      return { ok: false, error: 'A service URI is required when using the service backend.' };
    }
    payload.backend_options.service_uri = serviceUri;
  }

  return { ok: true, value: payload };
}

/**
 * @typedef {object} EarConfigPayload
 * @property {string} backend Selected backend slug.
 * @property {number} sample_rate Requested audio sample rate in hertz.
 * @property {number} channels Number of audio channels to stream.
 * @property {Record<string, unknown>} backend_options Backend-specific options.
 */

/**
 * Attempt to coerce a value into a finite integer.
 *
 * @param {unknown} value Candidate numeric input.
 * @returns {number | null} Normalised integer or ``null`` when coercion fails.
 */
export function coerceTranscriptInt(value) {
  if (typeof value === 'number') {
    return Number.isFinite(value) ? Math.trunc(value) : null;
  }
  if (typeof value === 'string') {
    const trimmed = value.trim();
    if (!trimmed) {
      return null;
    }
    const parsed = Number(trimmed);
    return Number.isFinite(parsed) ? Math.trunc(parsed) : null;
  }
  return null;
}

function normaliseWord(word) {
  if (!word || typeof word !== 'object') {
    return null;
  }
  const text = typeof word.text === 'string' ? word.text.trim() : '';
  const startMs = coerceTranscriptInt(word.startMs ?? word.start_ms);
  const endMs = coerceTranscriptInt(word.endMs ?? word.end_ms);
  if (!text && startMs == null && endMs == null) {
    return null;
  }
  return {
    text,
    startMs,
    endMs,
  };
}

function normaliseSegment(segment) {
  if (!segment || typeof segment !== 'object') {
    return null;
  }
  const text = typeof segment.text === 'string' ? segment.text.trim() : '';
  const startMs = coerceTranscriptInt(segment.startMs ?? segment.start_ms);
  const endMs = coerceTranscriptInt(segment.endMs ?? segment.end_ms);
  const words = Array.isArray(segment.words)
    ? segment.words.map(normaliseWord).filter((word) => word !== null)
    : [];
  if (!text && startMs == null && endMs == null && words.length === 0) {
    return null;
  }
  return {
    text,
    startMs,
    endMs,
    words,
  };
}

function coerceTimestamp(value) {
  if (value instanceof Date) {
    return value.toLocaleTimeString();
  }
  if (typeof value === 'string') {
    return value.trim();
  }
  if (typeof value === 'number' && Number.isFinite(value)) {
    return new Date(value).toLocaleTimeString();
  }
  return '';
}

function resolveAudioAttachment(entry) {
  const primary = typeof entry.audioBase64 === 'string' ? entry.audioBase64.trim() : '';
  if (primary) {
    return primary;
  }
  const alt = typeof entry.audio_base64 === 'string' ? entry.audio_base64.trim() : '';
  return alt || null;
}

function safeText(value) {
  if (typeof value === 'string') {
    return value.trim();
  }
  if (typeof value === 'number' && Number.isFinite(value)) {
    return String(value);
  }
  return '';
}

/**
 * Normalise a transcript entry so it can be rendered consistently.
 *
 * @param {object} entry Raw transcript payload captured from the websocket.
 * @returns {{
 *   id?: string,
 *   text: string,
 *   timestamp: string,
 *   startMs: number | null,
 *   endMs: number | null,
 *   segments: Array<{ text: string, startMs: number | null, endMs: number | null, words: Array<{ text: string, startMs: number | null, endMs: number | null }> }>,
 *   source: string,
 *   audioBase64: string | null,
 * }} Clean transcript entry.
 */
export function normalizeTranscriptEntry(entry) {
  const payload = entry && typeof entry === 'object' ? entry : {};
  const text = safeText(payload.text ?? payload.data);
  const segments = Array.isArray(payload.segments)
    ? payload.segments.map(normaliseSegment).filter((segment) => segment !== null)
    : [];
  const source = typeof payload.source === 'string' ? payload.source.trim() : '';

  return {
    id: typeof payload.id === 'string' ? payload.id : undefined,
    text,
    timestamp: coerceTimestamp(payload.timestamp ?? payload.time ?? ''),
    startMs: coerceTranscriptInt(payload.startMs ?? payload.start_ms),
    endMs: coerceTranscriptInt(payload.endMs ?? payload.end_ms),
    segments,
    source,
    audioBase64: resolveAudioAttachment(payload),
  };
}

/**
 * Create a stable signature representing a transcript entry.
 *
 * @param {object} entry Transcript entry compatible with {@link normalizeTranscriptEntry}.
 * @returns {string} Signature string suitable for deduplication.
 */
export function transcriptSignature(entry) {
  const normalised = normalizeTranscriptEntry(entry);
  const signaturePayload = {
    text: normalised.text,
    startMs: normalised.startMs,
    endMs: normalised.endMs,
    source: normalised.source,
    segments: normalised.segments.map((segment) => ({
      text: segment.text,
      startMs: segment.startMs,
      endMs: segment.endMs,
      words: segment.words.map((word) => ({
        text: word.text,
        startMs: word.startMs,
        endMs: word.endMs,
      })),
    })),
  };
  return JSON.stringify(signaturePayload);
}

/**
 * Track previously seen transcript signatures with bounded capacity.
 *
 * @param {number} [capacity=120] Maximum number of signatures to retain.
 */
export function createTranscriptDeduplicator(capacity = 120) {
  const limit = Number.isFinite(capacity) && capacity > 0 ? Math.trunc(capacity) : 120;
  const queue = [];
  const seen = new Set();

  return {
    has(signature) {
      return typeof signature === 'string' && seen.has(signature);
    },
    remember(signature) {
      if (typeof signature !== 'string' || !signature) {
        return;
      }
      if (seen.has(signature)) {
        return;
      }
      queue.push(signature);
      seen.add(signature);
      while (queue.length > limit) {
        const oldest = queue.shift();
        if (oldest) {
          seen.delete(oldest);
        }
      }
    },
    clear() {
      queue.length = 0;
      seen.clear();
    },
  };
}
