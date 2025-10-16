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
