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
  const numeric = parsed < options.min
    ? options.min
    : parsed > options.max
    ? options.max
    : parsed;
  return Number.isInteger(options.defaultValue)
    ? Math.round(numeric * 1000) / 1000
    : numeric;
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
  const threshold = clampNumber(draft.threshold, {
    min: 0,
    max: 1,
    defaultValue: 0.6,
  });
  if (threshold <= 0 || threshold > 1) {
    return { ok: false, error: "Detection threshold must be between 0 and 1." };
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
 * Normalize a payload emitted on the face sensation stream (/sensations) or the
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
  const text = typeof legacy === "string" ? legacy.trim() : "";
  if (!text) {
    return { ok: false, error: "Trigger payload was empty.", reason: "empty" };
  }
  return parseLegacyTrigger(text);
}

function unwrapDataField(raw) {
  if (raw && typeof raw === "object") {
    if (typeof raw.data === "string") {
      return raw.data;
    }
    if (
      raw.data && typeof raw.data === "object" &&
      typeof raw.data.data === "string"
    ) {
      return raw.data.data;
    }
  }
  if (typeof raw === "string") {
    return raw;
  }
  return "";
}

function normaliseString(value, fallback = "") {
  if (typeof value === "string") {
    const trimmed = value.trim();
    if (trimmed) {
      return trimmed;
    }
  }
  if (typeof value === "number" && Number.isFinite(value)) {
    return String(value);
  }
  return fallback;
}

function normaliseStringList(value) {
  if (Array.isArray(value)) {
    const seen = new Set();
    const output = [];
    for (const entry of value) {
      const text = normaliseString(entry);
      if (text && !seen.has(text)) {
        seen.add(text);
        output.push(text);
      }
    }
    return output;
  }
  const single = normaliseString(value);
  return single ? [single] : [];
}

function dedupeStrings(values) {
  const seen = new Set();
  const output = [];
  for (const entry of values) {
    if (!seen.has(entry)) {
      seen.add(entry);
      output.push(entry);
    }
  }
  return output;
}

function normaliseIdentitySummary(raw) {
  if (!raw || typeof raw !== "object") {
    return null;
  }
  const id = normaliseString(raw.id) ||
    normaliseString(raw.identity_id) ||
    normaliseString(raw.uuid);
  const name = normaliseString(raw.name);
  if (!id && !name) {
    return null;
  }
  const summary = {};
  if (id) summary.id = id;
  if (name) summary.name = name;
  const aliases = dedupeStrings([
    ...normaliseStringList(raw.aliases),
    ...normaliseStringList(raw.labels),
  ]);
  if (aliases.length) summary.aliases = aliases;
  let signatures = normaliseStringList(raw.signatures);
  const history = normaliseStringList(raw.signature_history);
  if (history.length) {
    signatures = dedupeStrings(signatures.concat(history));
  }
  const signatureHint = normaliseString(raw.signature);
  if (signatureHint) {
    signatures = dedupeStrings(signatures.concat([signatureHint]));
  }
  if (signatures.length) summary.signatures = signatures;
  let memoryIds = normaliseStringList(raw.memory_ids);
  const memoryHint = normaliseString(raw.memory_id);
  if (memoryHint) {
    memoryIds = dedupeStrings(memoryIds.concat([memoryHint]));
  }
  if (memoryIds.length) summary.memoryIds = memoryIds;
  const confidence = Number(raw.confidence ?? raw.score);
  if (Number.isFinite(confidence)) {
    summary.confidence = confidence;
  }
  return Object.keys(summary).length ? summary : null;
}

function resolveIdentitySummary(payload) {
  if (!payload || typeof payload !== "object") {
    return null;
  }
  const direct = normaliseIdentitySummary(payload.identity);
  if (direct) {
    return direct;
  }
  if (Array.isArray(payload.identities)) {
    for (const entry of payload.identities) {
      const summary = normaliseIdentitySummary(entry);
      if (summary) {
        return summary;
      }
    }
  }
  const name = normaliseString(payload.name);
  if (name) {
    const id = normaliseString(payload.memory_id) ||
      normaliseString(payload.vector_id) ||
      name;
    return { id, name };
  }
  return null;
}

function normaliseIdentityMatches(raw) {
  if (!Array.isArray(raw)) {
    return [];
  }
  const matches = [];
  for (const entry of raw) {
    if (!entry || typeof entry !== "object") {
      continue;
    }
    const match = {};
    const memoryId = normaliseString(entry.memory_id);
    if (memoryId) {
      match.memoryId = memoryId;
    }
    const score = Number(entry.score);
    if (Number.isFinite(score)) {
      match.score = score;
    }
    const signature = normaliseString(entry.signature);
    if (signature) {
      match.signature = signature;
    }
    const identity = normaliseIdentitySummary(entry.identity);
    if (identity) {
      match.identity = identity;
    }
    if (Object.keys(match).length) {
      matches.push(match);
    }
  }
  return matches;
}

function extractSensationPayload(raw) {
  if (raw && typeof raw === "object") {
    if (typeof raw.json_payload === "string") {
      return raw;
    }
    if (raw.data && typeof raw.data === "object") {
      const inner = raw.data;
      if (typeof inner.json_payload === "string") {
        return inner;
      }
    }
  }
  return null;
}

function parseSensation(message) {
  const kind = normaliseString(message.kind);
  if (kind && kind.toLowerCase() !== "face") {
    return {
      ok: false,
      error: "Sensation did not describe a face event.",
      reason: "ignored",
    };
  }

  const payloadText = normaliseString(message.json_payload);
  if (!payloadText) {
    return {
      ok: false,
      error: "Sensation payload was empty.",
      reason: "empty",
    };
  }

  let parsed;
  try {
    parsed = JSON.parse(payloadText);
  } catch (_error) {
    return {
      ok: false,
      error: "Sensation payload was not valid JSON.",
      reason: "invalid-json",
    };
  }
  if (!parsed || typeof parsed !== "object") {
    return {
      ok: false,
      error: "Sensation payload was not a JSON object.",
      reason: "invalid-json",
    };
  }

  const signature = normaliseString(parsed.signature);
  const memoryHint = normaliseString(parsed.memory_hint) ||
    (signature ? `mem-${signature}` : "");
  const vectorHint = normaliseString(parsed.vector_hint) ||
    signature;
  const identity = resolveIdentitySummary(parsed);
  const matches = normaliseIdentityMatches(parsed.matches);
  const identityName = identity ? normaliseString(identity.name) : "";
  let memoryId = normaliseString(parsed.memory_id);
  if (!memoryId && identity && Array.isArray(identity.memoryIds) && identity.memoryIds.length) {
    memoryId = normaliseString(identity.memoryIds[0]);
  }
  memoryId = memoryId || memoryHint;
  const name = identityName ||
    normaliseString(parsed.name) ||
    normaliseString(parsed.label) ||
    "Unknown face";
  const vectorId = normaliseString(parsed.vector_id) || vectorHint;
  const collection = normaliseString(
    parsed.collection,
    normaliseString(message.collection_hint),
  );
  const cropTopic = normaliseString(parsed.crop_topic) ||
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
  const identityConfidence = Number(
    parsed.identity_confidence ?? (identity ? identity.confidence : undefined),
  );
  if (Number.isFinite(identityConfidence)) {
    noteParts.push(`identity ${(identityConfidence * 100).toFixed(1)}%`);
  }
  if (
    memoryId && memoryId === memoryHint &&
    normaliseString(parsed.memory_id) === ""
  ) {
    noteParts.push(`memory hash ${memoryId}`);
  }
  if (
    vectorId && vectorId === vectorHint &&
    normaliseString(parsed.vector_id) === ""
  ) {
    noteParts.push(`vector hash ${vectorId}`);
  }
  const note = noteParts.join(" · ");

  return {
    ok: true,
    value: {
      name,
      memoryId,
      vectorId,
      collection,
      cropTopic,
      note,
      signature,
      memoryHint,
      vectorHint,
      vectorPreview: Array.isArray(message.vector)
        ? message.vector.slice(0, 8)
        : Array.isArray(parsed.vector_preview)
        ? parsed.vector_preview.slice(0, 8)
        : [],
      raw: payloadText,
      identity,
      identityConfidence: Number.isFinite(identityConfidence)
        ? identityConfidence
        : undefined,
      matches,
    },
  };
}

function parseLegacyTrigger(text) {
  let parsed;
  try {
    parsed = JSON.parse(text);
  } catch (_error) {
    return {
      ok: false,
      error: "Trigger payload was not valid JSON.",
      reason: "invalid-json",
    };
  }
  if (!parsed || typeof parsed !== "object") {
    return {
      ok: false,
      error: "Trigger payload was not a JSON object.",
      reason: "invalid-json",
    };
  }
  const name = normaliseString(parsed.name, "Unknown");
  const signature = normaliseString(parsed.signature);
  const memoryHint = normaliseString(parsed.memory_hint) ||
    (signature ? `mem-${signature}` : "");
  const vectorHint = normaliseString(parsed.vector_hint) ||
    signature;
  const memoryId = normaliseString(parsed.memory_id) || memoryHint;
  const vectorId = normaliseString(parsed.vector_id) || vectorHint;
  const collection = normaliseString(parsed.collection);
  return {
    ok: true,
    value: {
      name,
      memoryId,
      vectorId,
      collection,
      cropTopic: "",
      signature,
      memoryHint,
      vectorHint,
      vectorPreview: [],
      raw: text,
      identity: null,
      identityConfidence: undefined,
      matches: [],
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
 * @typedef {object} FaceIdentitySummary
 * @property {string} id Unique identifier for the resolved identity.
 * @property {string=} name Human readable label for the identity.
 * @property {string[]=} aliases Known aliases or alternate names.
 * @property {string[]=} signatures Embedding signatures linked to the identity.
 * @property {string[]=} memoryIds Historical memory identifiers associated with the identity.
 * @property {number=} confidence Similarity score (0-1) for the identity match.
 */

/**
 * @typedef {object} FaceIdentityMatch
 * @property {string=} memoryId Memory identifier returned by the recall service.
 * @property {number=} score Similarity score for the match.
 * @property {string=} signature Embedding signature associated with the match.
 * @property {FaceIdentitySummary=} identity Structured identity summary for the match.
 */

/**
 * @typedef {object} FaceTriggerEvent
 * @property {string} name Human-readable label associated with the detection.
 * @property {string} memoryId Identifier returned by the memory service.
 * @property {string} vectorId Embedding vector identifier.
 * @property {string=} memoryHint Fallback identifier derived from the embedding signature.
 * @property {string=} vectorHint Fallback vector identifier derived from the embedding signature.
 * @property {string=} signature Short hash describing the embedding.
 * @property {number[]} vectorPreview Sample of the embedding values for quick inspection.
 * @property {string} collection Backing collection name.
 * @property {string=} cropTopic ROS topic that publishes cropped face detections.
 * @property {string} raw Raw JSON payload string.
 * @property {string=} note Contextual note about the event (confidence, embedding dims, etc).
 * @property {FaceIdentitySummary|null} identity Best-match identity summary, when available.
 * @property {number|undefined} identityConfidence Confidence score for the resolved identity (0-1).
 * @property {FaceIdentityMatch[]} matches Additional recall matches surfaced by the memory service.
 */
