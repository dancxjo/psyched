/**
 * Helpers for working with module topic metadata and websocket records.
 */

export function topicIdentifier(topic) {
  if (!topic) return '';
  return topic.name || topic.topic || `${topic}`;
}

export function topicKey(moduleName, topic) {
  return `${moduleName}:${topicIdentifier(topic)}`;
}

const DEFAULT_PROFILE = Object.freeze({
  mode: 'immediate',
  interval: 0,
  frame: false,
  collapse: false,
});

const PRESENTATION_PROFILES = Object.freeze({
  image: { mode: 'throttle', interval: 0, frame: true, collapse: true },
  depth: { mode: 'throttle', interval: 0, frame: true, collapse: true },
  map: { mode: 'throttle', interval: 180, frame: false, collapse: true },
  path: { mode: 'throttle', interval: 150, frame: false, collapse: true },
  waveform: { mode: 'batch', interval: 220, frame: false, collapse: false },
  oscilloscope: { mode: 'batch', interval: 160, frame: false, collapse: false },
});

const TYPE_PROFILES = new Map(
  Object.entries({
    'sensor_msgs/msg/image': PRESENTATION_PROFILES.image,
    'sensor_msgs/msg/compressedimage': PRESENTATION_PROFILES.image,
    'sensor_msgs/msg/pointcloud2': PRESENTATION_PROFILES.map,
    'nav_msgs/msg/occupancygrid': PRESENTATION_PROFILES.map,
    'nav_msgs/msg/path': PRESENTATION_PROFILES.path,
  }).filter(([, profile]) => Boolean(profile)),
);

function normalise(value) {
  return typeof value === 'string' ? value.trim().toLowerCase() : '';
}

function cloneProfile(profile) {
  return { ...profile };
}

/**
 * Determine how aggressively to throttle incoming websocket payloads for a topic.
 *
 * High-frequency feeds such as camera frames or PCM samples can overwhelm the
 * frontend if every message triggers a re-render.  The returned profile guides
 * the pilot shell so it can batch or collapse updates without losing the most
 * recent state.
 *
 * Examples:
 *   >>> topicUpdateProfile({ presentation: 'image' })
 *   { mode: 'throttle', interval: 0, frame: true, collapse: true }
 *
 * @param {object} topic Topic metadata from the module catalog.
 * @returns {{mode: string, interval: number, frame: boolean, collapse: boolean}}
 */
export function topicUpdateProfile(topic) {
  const presentation = normalise(topic?.presentation);
  if (presentation && PRESENTATION_PROFILES[presentation]) {
    return cloneProfile(PRESENTATION_PROFILES[presentation]);
  }

  const type = normalise(topic?.type);
  if (type && TYPE_PROFILES.has(type)) {
    return cloneProfile(TYPE_PROFILES.get(type));
  }

  return cloneProfile(DEFAULT_PROFILE);
}
