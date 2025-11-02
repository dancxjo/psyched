/**
 * Helper utilities for the GPS dashboard.
 */

const RESET_MODES = new Set(['hot', 'warm', 'cold']);

/**
 * Normalize the requested GPS reset mode.
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

const FIX_STATUS_MAP = new Map([
  [-1, { text: 'Awaiting satellite lock', tone: 'warning', hasFix: false }],
  [0, { text: '3D fix', tone: 'success', hasFix: true }],
  [1, { text: 'SBAS fix', tone: 'success', hasFix: true }],
  [2, { text: 'GBAS fix', tone: 'success', hasFix: true }],
]);

function coerceNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function formatLatitude(value) {
  const hemisphere = value >= 0 ? 'N' : 'S';
  return `${Math.abs(value).toFixed(4)}°${hemisphere}`;
}

function formatLongitude(value) {
  const hemisphere = value >= 0 ? 'E' : 'W';
  return `${Math.abs(value).toFixed(4)}°${hemisphere}`;
}

/**
 * Convert a ROS 2 ``builtin_interfaces/msg/Time`` stamp to a ``Date``.
 *
 * @param {{ sec?: number, nanosec?: number } | undefined | null} stamp
 * @returns {Date | null}
 */
export function rosTimeToDate(stamp) {
  if (!stamp || (!Number.isFinite(stamp.sec) && !Number.isFinite(stamp.nanosec))) {
    return null;
  }
  const seconds = Number(stamp.sec ?? 0);
  const nanoseconds = Number(stamp.nanosec ?? 0);
  const millis = seconds * 1000 + Math.floor(nanoseconds / 1_000_000);
  if (!Number.isFinite(millis)) {
    return null;
  }
  return new Date(millis);
}

function resolveStatus(payload) {
  const rawStatus = payload?.status;
  const code = typeof rawStatus === 'object' && rawStatus
    ? Number(rawStatus.status)
    : Number(rawStatus);
  if (Number.isFinite(code) && FIX_STATUS_MAP.has(code)) {
    return { code, ...FIX_STATUS_MAP.get(code) };
  }
  return { code: null, text: 'Awaiting satellite lock', tone: 'warning', hasFix: false };
}

/**
 * Summarize a ``NavSatFix`` message for cockpit presentation.
 *
 * @param {Record<string, unknown> | null | undefined} message
 * @returns {{
 *   statusText: string,
 *   tone: 'success' | 'warning' | 'error',
 *   latitude: string | null,
 *   longitude: string | null,
 *   altitude: string | null,
 *   timestamp: Date | null,
 *   hasFix: boolean,
 *   eventSummary: string | null,
 * }}
 */
export function describeNavSatFix(message) {
  const status = resolveStatus(message ?? {});
  const latitudeValue = message ? coerceNumber(message.latitude) : null;
  const longitudeValue = message ? coerceNumber(message.longitude) : null;
  const altitudeValue = message ? coerceNumber(message.altitude) : null;

  if (!status.hasFix || latitudeValue === null || longitudeValue === null) {
    return {
      statusText: status.text,
      tone: status.hasFix ? 'warning' : status.tone,
      latitude: null,
      longitude: null,
      altitude: null,
      timestamp: null,
      hasFix: false,
      eventSummary: 'Fix lost; awaiting satellite lock.',
    };
  }

  const latitude = formatLatitude(latitudeValue);
  const longitude = formatLongitude(longitudeValue);
  const altitude = Number.isFinite(altitudeValue) ? `${altitudeValue.toFixed(1)} m` : '—';
  const timestamp = message?.header ? rosTimeToDate(message.header.stamp) : null;
  const summary = `Fix update: ${latitude} ${longitude} alt=${altitude} (${status.text}).`;

  return {
    statusText: status.text,
    tone: status.tone,
    latitude,
    longitude,
    altitude,
    timestamp,
    hasFix: true,
    eventSummary: summary,
  };
}

/**
 * @typedef {object} GpsResetPayload
 * @property {'hot' | 'warm' | 'cold'} mode Reset mode requested.
 * @property {string | null} note Optional operator note.
 */
