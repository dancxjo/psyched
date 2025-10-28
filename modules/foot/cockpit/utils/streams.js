const STREAM_ACTION_ENTRIES = [
  ['battery/charge', 'battery_charge_stream'],
  ['battery/capacity', 'battery_capacity_stream'],
  ['battery/charge_ratio', 'battery_charge_ratio_stream'],
  ['battery/voltage', 'battery_voltage_stream'],
  ['battery/current', 'battery_current_stream'],
  ['battery/temperature', 'battery_temperature_stream'],
  ['battery/charging_state', 'battery_charging_state_stream'],
  ['bumper', 'bumper_stream'],
  ['cliff', 'cliff_stream'],
  ['wheeldrop', 'wheeldrop_stream'],
  ['clean_button', 'clean_button_stream'],
  ['spot_button', 'spot_button_stream'],
  ['dock_button', 'dock_button_stream'],
  ['day_button', 'day_button_stream'],
  ['hour_button', 'hour_button_stream'],
  ['minute_button', 'minute_button_stream'],
  ['ir_omni', 'ir_omni_stream'],
  ['joint_states', 'joint_states_stream'],
  ['mode', 'mode_stream'],
  ['odom', 'odom_stream'],
  ['main_brush_motor', 'main_brush_motor_stream'],
  ['side_brush_motor', 'side_brush_motor_stream'],
  ['vacuum_motor', 'vacuum_motor_stream'],
  ['diagnostics', 'diagnostics_stream'],
  ['parameter_events', 'parameter_events_stream'],
  ['tf', 'tf_stream'],
  ['tf_static', 'tf_static_stream'],
];

const STREAM_ACTIONS = new Map(STREAM_ACTION_ENTRIES);

/**
 * Return a canonical topic name stripped of leading slashes and extraneous
 * whitespace so cockpit components can perform reliable lookups.
 *
 * @example
 * normaliseTopic('/battery/charge');
 * // => 'battery/charge'
 *
 * @param {unknown} topic - Topic identifier from a dashboard configuration.
 * @returns {string} Canonicalised topic or an empty string when not coercible.
 */
export function normaliseTopic(topic) {
  if (typeof topic !== 'string') {
    return '';
  }
  const trimmed = topic.trim();
  if (!trimmed) {
    return '';
  }
  return trimmed.replace(/^\/+/, '');
}

/**
 * Resolve the cockpit action name responsible for streaming *topic* telemetry.
 *
 * The mapping mirrors the ``stream-topic`` definitions in
 * ``modules/foot/cockpit/api/actions.json`` so publishers and dashboards can
 * ask the backend for an authorised websocket.
 *
 * @example
 * streamActionForTopic('battery/voltage');
 * // => 'battery_voltage_stream'
 *
 * @param {unknown} topic - Topic identifier using ROS-style naming.
 * @returns {string|null} Action name or ``null`` when the topic is unknown.
 */
export function streamActionForTopic(topic) {
  const normalised = normaliseTopic(topic);
  return STREAM_ACTIONS.get(normalised) ?? null;
}

export const streamActionEntries = [...STREAM_ACTION_ENTRIES];
