/**
 * Helper utilities for the GPS dashboard.
 */

const RESET_MODES = new Set(['hot', 'warm', 'cold']);

/**
 * Normalise the requested GPS reset mode.
 *
 * @param {string | null | undefined} value Raw mode string.
 * @returns {'hot' | 'warm' | 'cold'}
 */
export function normalizeResetMode(value) {
  const slug = String(value ?? '').trim().toLowerCase();
  if (RESET_MODES.has(slug)) {
    return /** @type {'hot' | 'warm' | 'cold'} */ (slug);
  }
  return 'hot';
}

/**
 * Construct a payload for a GPS reset request.
 *
 * @param {object} draft Form values.
 * @param {string} draft.mode Reset mode.
 * @param {string} [draft.note] Optional operator note.
 * @returns {{ ok: true, value: GpsResetPayload } | { ok: false, error: string }}
 */
export function buildGpsResetPayload(draft) {
  const mode = normalizeResetMode(draft.mode);
  const note = String(draft.note ?? '').trim();
  if (note.length > 240) {
    return { ok: false, error: 'Notes must be 240 characters or fewer.' };
  }
  return { ok: true, value: { mode, note: note || null } };
}

/**
 * @typedef {object} GpsResetPayload
 * @property {'hot' | 'warm' | 'cold'} mode Reset mode requested.
 * @property {string | null} note Optional operator note.
 */
