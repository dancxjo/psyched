/**
 * Helpers for extracting scalar values from ROS topic payloads.
 */

/**
 * Attempt to coerce a payload into a numeric value.
 *
 * @param {unknown} value - Message payload to interpret.
 * @param {number} [fallback=0] - Value returned when coercion fails.
 * @returns {number} Parsed number or the fallback.
 */
export function extractNumeric(value, fallback = 0) {
  if (typeof value === 'number' && Number.isFinite(value)) {
    return value;
  }
  if (typeof value === 'string') {
    const parsed = Number.parseFloat(value);
    return Number.isFinite(parsed) ? parsed : fallback;
  }
  if (Array.isArray(value) && value.length === 1) {
    return extractNumeric(value[0], fallback);
  }
  if (value && typeof value === 'object') {
    if (typeof value.data !== 'undefined') {
      return extractNumeric(value.data, fallback);
    }
    if (typeof value.value !== 'undefined') {
      return extractNumeric(value.value, fallback);
    }
  }
  return fallback;
}

/**
 * Coerce a payload into a boolean value.
 *
 * @param {unknown} value - Message payload to interpret.
 * @param {boolean} [fallback=false] - Value returned when coercion fails.
 * @returns {boolean} Parsed boolean value.
 */
export function extractBoolean(value, fallback = false) {
  if (typeof value === 'boolean') {
    return value;
  }
  if (typeof value === 'number') {
    return value !== 0;
  }
  if (typeof value === 'string') {
    const lowered = value.trim().toLowerCase();
    if (lowered === 'true' || lowered === 'yes' || lowered === 'on') {
      return true;
    }
    if (lowered === 'false' || lowered === 'no' || lowered === 'off') {
      return false;
    }
  }
  if (value && typeof value === 'object' && typeof value.data !== 'undefined') {
    return extractBoolean(value.data, fallback);
  }
  return fallback;
}

/**
 * Extract a textual representation from a message payload.
 *
 * @param {unknown} value - Message payload to describe.
 * @returns {string} Human readable text.
 */
export function extractText(value) {
  if (value == null) {
    return '';
  }
  if (typeof value === 'string') {
    return value;
  }
  if (typeof value === 'number' || typeof value === 'boolean') {
    return String(value);
  }
  if (value && typeof value === 'object' && typeof value.data !== 'undefined') {
    return extractText(value.data);
  }
  try {
    return JSON.stringify(value, null, 2);
  } catch (error) {
    return String(value);
  }
}

export default {
  extractNumeric,
  extractBoolean,
  extractText,
};
