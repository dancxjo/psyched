/**
 * Helper utilities for the Faces dashboard.
 */

/**
 * Clamp a numeric value into an inclusive range.
 *
 * @param {number|string|null|undefined} value Input from the UI.
 * @param {{ min: number, max: number, defaultValue: number }} options Range metadata.
 * @returns {number}
 */
export function clampNumber(value, options) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) {
    return options.defaultValue;
  }
  const numeric = parsed < options.min ? options.min : parsed > options.max ? options.max : parsed;
  return Number.isInteger(options.defaultValue) ? Math.round(numeric * 1000) / 1000 : numeric;
}

/**
 * Build a payload describing updated face detector settings.
 *
 * @param {object} draft Raw values from the dashboard.
 * @param {number|string} draft.threshold Detection threshold between 0 and 1.
 * @param {number|string} draft.window Temporal smoothing window in frames.
 * @param {boolean} draft.publishCrops Toggle for crop publishing.
 * @param {boolean} draft.publishEmbeddings Toggle for embedding publishing.
 * @returns {{ ok: true, value: FacesSettingsPayload } | { ok: false, error: string }}
 */
export function buildFacesSettingsPayload(draft) {
  const threshold = clampNumber(draft.threshold, { min: 0, max: 1, defaultValue: 0.6 });
  if (threshold <= 0 || threshold > 1) {
    return { ok: false, error: 'Detection threshold must be between 0 and 1.' };
  }
  const window = Math.max(1, Math.trunc(Number(draft.window ?? 15)));
  return {
    ok: true,
    value: {
      detection_threshold: Number(threshold.toFixed(3)),
      smoothing_window: window,
      publish_crops: Boolean(draft.publishCrops),
      publish_embeddings: Boolean(draft.publishEmbeddings),
    },
  };
}

/**
 * @typedef {object} FacesSettingsPayload
 * @property {number} detection_threshold Detector confidence threshold.
 * @property {number} smoothing_window Temporal smoothing window in frames.
 * @property {boolean} publish_crops Whether face crops should be published.
 * @property {boolean} publish_embeddings Whether embeddings should be emitted.
 */
