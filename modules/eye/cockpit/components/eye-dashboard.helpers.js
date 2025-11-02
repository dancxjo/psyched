/**
 * Helper utilities for the Eye dashboard.
 */

const DEPTH_MODES = new Set(['disabled', 'depth', 'aligned_depth']);

/**
 * Normalise a depth streaming mode value.
 *
 * @param {string | null | undefined} value Raw string from the dashboard form.
 * @returns {'disabled' | 'depth' | 'aligned_depth'}
 */
export function normalizeDepthMode(value) {
  const slug = String(value ?? '').trim().toLowerCase();
  if (DEPTH_MODES.has(slug)) {
    return /** @type {'disabled' | 'depth' | 'aligned_depth'} */ (slug);
  }
  return 'disabled';
}

/**
 * Convert arbitrary input into an integer within safe bounds.
 *
 * @param {number | string | null | undefined} value Candidate numeric value.
 * @param {{ min: number, max: number, defaultValue: number }} options Range metadata.
 * @returns {number}
 */
export function clampSetting(value, options) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) {
    return options.defaultValue;
  }
  const integer = Math.trunc(parsed);
  if (integer < options.min) return options.min;
  if (integer > options.max) return options.max;
  return integer;
}

/**
 * Build a payload describing the requested Eye sensor configuration.
 *
 * @param {object} draft Form values.
 * @param {number|string} draft.width Desired image width.
 * @param {number|string} draft.height Desired image height.
 * @param {number|string} draft.frameRate Requested frame rate.
 * @param {string} draft.depthMode Depth streaming mode slug.
 * @param {boolean} draft.alignDepth Whether to align depth with color frames.
 * @param {number|string} draft.exposure Manual exposure in microseconds.
 * @param {number|string} draft.gain Manual gain value.
 * @param {boolean} draft.autoExposure Whether auto exposure is enabled.
 * @returns {{ ok: true, value: EyeSettingsPayload } | { ok: false, error: string }}
 */
export function buildEyeSettingsPayload(draft) {
  const width = clampSetting(draft.width, { min: 320, max: 1920, defaultValue: 1280 });
  const height = clampSetting(draft.height, { min: 240, max: 1080, defaultValue: 720 });
  const frameRate = clampSetting(draft.frameRate, { min: 5, max: 60, defaultValue: 15 });

  if (width * height > 1920 * 1080) {
    return { ok: false, error: 'Requested resolution exceeds supported capture size.' };
  }

  const payload = {
    resolution: { width, height },
    frame_rate: frameRate,
    depth_mode: normalizeDepthMode(draft.depthMode),
    align_depth: Boolean(draft.alignDepth),
    auto_exposure: Boolean(draft.autoExposure),
  };

  if (!payload.auto_exposure) {
    payload.manual = {
      exposure: clampSetting(draft.exposure, { min: 10, max: 33_000, defaultValue: 3_000 }),
      gain: clampSetting(draft.gain, { min: 1, max: 128, defaultValue: 32 }),
    };
  }

  return { ok: true, value: payload };
}

/**
 * @typedef {object} EyeSettingsPayload
 * @property {{ width: number, height: number }} resolution Requested resolution.
 * @property {number} frame_rate Requested frame rate.
 * @property {'disabled' | 'depth' | 'aligned_depth'} depth_mode Depth streaming mode.
 * @property {boolean} align_depth Whether to align depth with color frames.
 * @property {boolean} auto_exposure Whether automatic exposure is active.
 * @property {{ exposure: number, gain: number } | undefined} manual Manual overrides when ``auto_exposure`` is ``false``.
 */
