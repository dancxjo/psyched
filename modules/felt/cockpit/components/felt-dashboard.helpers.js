/**
 * Helper utilities for the Felt dashboard.
 */

/**
 * Clamp a floating point value.
 *
 * @param {number|string|null|undefined} value Raw input value.
 * @param {{ min: number, max: number, defaultValue: number }} options Range metadata.
 * @returns {number}
 */
export function clampFloat(value, options) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) {
    return options.defaultValue;
  }
  if (parsed < options.min) return options.min;
  if (parsed > options.max) return options.max;
  return Number(parsed.toFixed(3));
}

/**
 * Build a payload describing a requested FeelingIntent override.
 *
 * @param {object} draft Form values captured from the dashboard.
 * @param {number|string} draft.valence Emotional valence [-1, 1].
 * @param {number|string} draft.arousal Emotional arousal [0, 1].
 * @param {number|string} draft.stance Navigational stance [0, 1].
 * @param {string} draft.context Free-form context string.
 * @returns {{ ok: true, value: FeltIntentPayload } | { ok: false, error: string }}
 */
export function buildFeltIntentPayload(draft) {
  const valence = clampFloat(draft.valence, { min: -1, max: 1, defaultValue: 0 });
  const arousal = clampFloat(draft.arousal, { min: 0, max: 1, defaultValue: 0.2 });
  const stance = clampFloat(draft.stance, { min: 0, max: 1, defaultValue: 0.5 });
  const context = String(draft.context ?? '').trim();
  if (!context) {
    return { ok: false, error: 'Context is required to broadcast a feeling intent.' };
  }
  return {
    ok: true,
    value: { valence, arousal, stance, context },
  };
}

/**
 * @typedef {object} FeltIntentPayload
 * @property {number} valence Normalised valence value.
 * @property {number} arousal Normalised arousal value.
 * @property {number} stance Goal seeking stance weighting.
 * @property {string} context Narrative context associated with the intent.
 */
