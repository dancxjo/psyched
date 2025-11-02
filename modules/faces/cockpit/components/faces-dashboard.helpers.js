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
 * Normalise a payload emitted on the face sensation stream (/sensations) or the
 * legacy /vision/face_detected topic into a cockpit-friendly shape.
 *
 * @param {unknown} raw Payload returned from the websocket bridge.
 * @returns {{ ok: true, value: FaceTriggerEvent } | { ok: false, error: string }}
 */
export function parseFaceTriggerPayload(raw) {
  const sensation = extractSensationPayload(raw);
  if (sensation) {
    return parseSensation(sensation);
  }

  const legacy = unwrapDataField(raw);
  const text = typeof legacy === 'string' ? legacy.trim() : '';
  if (!text) {
    return { ok: false, error: 'Trigger payload was empty.', reason: 'empty' };
  }
  return parseLegacyTrigger(text);
}

function unwrapDataField(raw) {
  if (raw && typeof raw === 'object') {
    if (typeof raw.data === 'string') {
      return raw.data;
    }
    if (raw.data && typeof raw.data === 'object' && typeof raw.data.data === 'string') {
      return raw.data.data;
    }
  }
  if (typeof raw === 'string') {
    return raw;
  }
  return '';
}

function normaliseString(value, fallback = '') {
  if (typeof value === 'string') {
    const trimmed = value.trim();
    if (trimmed) {
      return trimmed;
    }
  }
  if (typeof value === 'number' && Number.isFinite(value)) {
    return String(value);
  }
  return fallback;
}

function extractSensationPayload(raw) {
  if (raw && typeof raw === 'object') {
    if (typeof raw.json_payload === 'string') {
      return raw;
    }
    if (raw.data && typeof raw.data === 'object') {
      const inner = raw.data;
      if (typeof inner.json_payload === 'string') {
        return inner;
      }
    }
  }
  return null;
}

function parseSensation(message) {
  const kind = normaliseString(message.kind);
  if (kind && kind.toLowerCase() !== 'face') {
    return { ok: false, error: 'Sensation did not describe a face event.', reason: 'ignored' };
  }

  const payloadText = normaliseString(message.json_payload);
  if (!payloadText) {
    return { ok: false, error: 'Sensation payload was empty.', reason: 'empty' };
  }

  let parsed;
  try {
    parsed = JSON.parse(payloadText);
  } catch (_error) {
    return { ok: false, error: 'Sensation payload was not valid JSON.', reason: 'invalid-json' };
  }
  if (!parsed || typeof parsed !== 'object') {
    return { ok: false, error: 'Sensation payload was not a JSON object.', reason: 'invalid-json' };
  }

  const name =
    normaliseString(parsed.name) ||
    normaliseString(parsed.label) ||
    'Unknown face';
  const memoryId = normaliseString(parsed.memory_id);
  const vectorId = normaliseString(parsed.vector_id);
  const collection = normaliseString(parsed.collection, normaliseString(message.collection_hint));
  const cropTopic =
    normaliseString(parsed.crop_topic) ||
    normaliseString(parsed.crops_topic) ||
    normaliseString(parsed.topic) ||
    normaliseString(message.topic);

  const noteParts = [];
  const confidence = Number(parsed.confidence);
  if (Number.isFinite(confidence)) {
    noteParts.push(`confidence ${(confidence * 100).toFixed(1)}%`);
  }
  const embeddingDim = Number(parsed.embedding_dim);
  if (Number.isInteger(embeddingDim) && embeddingDim > 0) {
    noteParts.push(`${embeddingDim}-dim vector`);
  }
  const note = noteParts.join(' · ');

  return {
    ok: true,
    value: {
      name,
      memoryId,
      vectorId,
      collection,
      cropTopic,
      note,
      raw: payloadText,
    },
  };
}

function parseLegacyTrigger(text) {
  let parsed;
  try {
    parsed = JSON.parse(text);
  } catch (_error) {
    return { ok: false, error: 'Trigger payload was not valid JSON.', reason: 'invalid-json' };
  }
  if (!parsed || typeof parsed !== 'object') {
    return { ok: false, error: 'Trigger payload was not a JSON object.', reason: 'invalid-json' };
  }
  const name = normaliseString(parsed.name, 'Unknown');
  const memoryId = normaliseString(parsed.memory_id);
  const vectorId = normaliseString(parsed.vector_id);
  const collection = normaliseString(parsed.collection);
  return {
    ok: true,
    value: {
      name,
      memoryId,
      vectorId,
      collection,
      cropTopic: '',
      raw: text,
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

/**
 * @typedef {object} FaceTriggerEvent
 * @property {string} name Human-readable label associated with the detection.
 * @property {string} memoryId Identifier returned by the memory service.
 * @property {string} vectorId Embedding vector identifier.
 * @property {string} collection Backing collection name.
 * @property {string=} cropTopic ROS topic that publishes cropped face detections.
 * @property {string} raw Raw JSON payload string.
 * @property {string=} note Contextual note about the event (confidence, embedding dims, etc).
 */
