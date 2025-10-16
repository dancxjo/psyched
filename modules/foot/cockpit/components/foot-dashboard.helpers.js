/**
 * Utility helpers for the foot dashboard.
 *
 * The helpers remain intentionally stateless so they can be unit-tested with
 * Node's built-in test runner. Keep any WebSocket or DOM concerns within the
 * dashboard component itself.
 */

export const PARAMETER_TYPES = {
  NOT_SET: 0,
  BOOL: 1,
  INTEGER: 2,
  DOUBLE: 3,
  STRING: 4,
  BYTE_ARRAY: 5,
  BOOL_ARRAY: 6,
  INTEGER_ARRAY: 7,
  DOUBLE_ARRAY: 8,
  STRING_ARRAY: 9,
  BYTE: 10,
};

/**
 * Build a ROS2 parameter message for :srv:`rcl_interfaces/srv/SetParameters`.
 *
 * @param {Record<string, string>} values keyed by parameter name.
 * @returns {Array<object>} List of parameter structures expected by ROS 2.
 *
 * @example
 * buildParameterRequest({
 *   publish_tf: 'true',
 *   loop_hz: '10.0',
 * })
 * // → [
 * //   { name: 'publish_tf', value: { type: PARAMETER_TYPES.BOOL, bool_value: true } },
 * //   { name: 'loop_hz', value: { type: PARAMETER_TYPES.DOUBLE, double_value: 10 } }
 * // ]
 */
export function buildParameterRequest(values) {
  if (!values || typeof values !== 'object') {
    return [];
  }

  const entries = [];
  for (const [key, rawValue] of Object.entries(values)) {
    const name = key.trim();
    if (!name) {
      continue;
    }
    const prepared = prepareParameterValue(rawValue);
    if (prepared) {
      entries.push({ name, value: prepared });
    }
  }
  return entries;
}

/**
 * Normalise form input into a ``ParameterValue`` object.
 *
 * ``false`` is treated as a boolean, ``42`` as an integer, ``3.14`` as a
 * floating point value, and all other inputs as strings.
 *
 * @param {unknown} value form input
 * @returns {object | null}
 */
export function prepareParameterValue(value) {
  if (value == null) {
    return null;
  }
  const text = String(value).trim();
  if (text === '') {
    return null;
  }
  if (text === 'true' || text === 'false') {
    return {
      type: PARAMETER_TYPES.BOOL,
      bool_value: text === 'true',
    };
  }
  if (/^-?\d+$/.test(text)) {
    const integerValue = Number.parseInt(text, 10);
    return {
      type: PARAMETER_TYPES.INTEGER,
      integer_value: integerValue,
    };
  }
  if (/^-?\d*\.\d+$/.test(text)) {
    const doubleValue = Number.parseFloat(text);
    return {
      type: PARAMETER_TYPES.DOUBLE,
      double_value: doubleValue,
    };
  }
  return {
    type: PARAMETER_TYPES.STRING,
    string_value: text,
  };
}

/**
 * Format a :msg:`sensor_msgs/msg/JointState` into a compact summary.
 *
 * @param {object} message
 * @returns {string}
 */
export function formatJointState(message) {
  if (!message || typeof message !== 'object') {
    return 'joint state unavailable';
  }
  const positions = Array.isArray(message.position) ? message.position : [];
  if (!positions.length) {
    return 'joint state unavailable';
  }
  const display = positions
    .slice(0, 2)
    .map((value, index) => `wheel ${index + 1}: ${Number(value).toFixed(3)} rad`);
  return display.join(' · ');
}

/**
 * Format :msg:`nav_msgs/msg/Odometry` readings for human consumption.
 *
 * @param {object} message
 * @returns {string}
 */
export function formatOdometry(message) {
  if (!message || typeof message !== 'object') {
    return 'odometry unavailable';
  }
  const pose = message.pose?.pose ?? {};
  const position = pose.position ?? {};
  const x = Number(position.x ?? 0).toFixed(3);
  const y = Number(position.y ?? 0).toFixed(3);
  const linear = message.twist?.twist?.linear ?? {};
  const angular = message.twist?.twist?.angular ?? {};
  const vx = Number(linear.x ?? 0).toFixed(3);
  const wz = Number(angular.z ?? 0).toFixed(3);
  return `x ${x} m · y ${y} m · v ${vx} m/s · ω ${wz} rad/s`;
}

/**
 * Describe the payload of :msg:`rcl_interfaces/msg/ParameterEvent` succinctly.
 *
 * @param {object} event
 * @returns {string}
 */
export function formatParameterEvent(event) {
  if (!event || typeof event !== 'object') {
    return 'parameter update unavailable';
  }
  const parts = [];
  const appendParameters = (label, parameters) => {
    if (!Array.isArray(parameters) || !parameters.length) {
      return;
    }
    const values = parameters
      .map((parameter) => `${parameter.name ?? 'unknown'}=${parameter.value?.string_value ?? parameter.value?.integer_value ?? parameter.value?.double_value ?? parameter.value?.bool_value ?? '—'}`)
      .join(', ');
    parts.push(`${label}: ${values}`);
  };
  appendParameters('new', event.new_parameters);
  appendParameters('changed', event.changed_parameters);
  appendParameters('deleted', event.deleted_parameters);
  return parts.length ? parts.join(' · ') : 'parameter event received';
}

/**
 * Parse a song sheet expressed as ``note,duration`` pairs separated by newlines
 * or semicolons.
 *
 * @param {string} sheet
 * @returns {Array<{note: number, duration: number}>}
 */
export function parseSongSheet(sheet) {
  if (!sheet) {
    return [];
  }
  const entries = [];
  for (const chunk of String(sheet).split(/[,;\n]+/g)) {
    // We'll rebuild pairs using a sliding window below.
  }
  const tokens = String(sheet)
    .split(/[\n;]+/)
    .map((line) => line.trim())
    .filter(Boolean);
  for (const token of tokens) {
    const parts = token.split(',').map((part) => part.trim()).filter(Boolean);
    if (parts.length !== 2) {
      continue;
    }
    const note = Number(parts[0]);
    const duration = Number(parts[1]);
    if (Number.isFinite(note) && Number.isFinite(duration)) {
      entries.push({ note, duration });
    }
  }
  return entries;
}

/**
 * Convert ASCII text into the ``std_msgs/msg/UInt8MultiArray`` payload accepted
 * by ``/set_ascii``.
 *
 * @param {string} text
 * @returns {{ data: number[] }}
 */
export function toAsciiPayload(text) {
  if (!text) {
    return { data: [] };
  }
  const codes = Array.from(String(text))
    .slice(0, 4)
    .map((character) => character.charCodeAt(0));
  return { data: codes };
}

/**
 * Build a power LED payload from colour and intensity sliders.
 *
 * @param {number} color between 0 (green) and 255 (red)
 * @param {number} intensity between 0 and 255
 * @returns {{ data: number[] }}
 */
export function buildPowerLedPayload(color, intensity) {
  const colourValue = clampByte(color);
  const intensityValue = clampByte(intensity);
  return { data: [colourValue, intensityValue] };
}

function clampByte(value) {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) {
    return 0;
  }
  return Math.min(255, Math.max(0, Math.round(numeric)));
}
