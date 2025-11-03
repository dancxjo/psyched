/**
 * Helper utilities for the Memory dashboard.
 */

/**
 * Build a payload for querying the memory service.
 *
 * @param {object} draft Form values.
 * @param {string} draft.query Natural language query string.
 * @param {number|string} draft.topK Number of results requested.
 * @returns {{ ok: true, value: MemoryQueryPayload } | { ok: false, error: string }}
 */
export function buildMemoryQueryPayload(draft) {
  const query = String(draft.query ?? '').trim();
  if (!query) {
    return { ok: false, error: 'Query text is required.' };
  }
  const topK = Math.max(1, Math.min(20, Math.trunc(Number(draft.topK ?? 5))));
  return { ok: true, value: { query, top_k: topK } };
}

/**
 * Build a payload for storing a new memory entry.
 *
 * @param {object} draft Form values.
 * @param {string} draft.title Short summary of the memory.
 * @param {string} draft.body Detailed content of the memory.
 * @param {string} [draft.tags] Optional comma separated tags.
 * @returns {{ ok: true, value: MemoryStorePayload } | { ok: false, error: string }}
 */
export function buildMemoryStorePayload(draft) {
  const title = String(draft.title ?? '').trim();
  const body = String(draft.body ?? '').trim();
  if (!title || !body) {
    return { ok: false, error: 'Title and body are required to persist a memory.' };
  }
  const tags = (String(draft.tags ?? '')
    .split(',')
    .map((tag) => tag.trim())
    .filter(Boolean));
  return { ok: true, value: { title, body, tags } };
}

/**
 * Normalize raw recall records returned by the memory service into
 * dashboard-friendly entries.
 *
 * @param {RawRecallResult[] | undefined | null} rawResults Raw JSON response from the recall action.
 * @returns {NormalisedRecallEntry[]}
 */
export function normaliseRecallResults(rawResults) {
  if (!Array.isArray(rawResults)) {
    return [];
  }
  const entries = [];
  for (const raw of rawResults) {
    const normalised = _normaliseRecallResult(raw);
    if (normalised) {
      entries.push(normalised);
    }
  }
  return entries;
}

function _normaliseRecallResult(raw) {
  if (!raw || typeof raw !== 'object') {
    return null;
  }
  const memoryId = _safeString(raw.memory_id);
  const score = _safeNumber(raw.score);
  const metadata = _parseMetadata(raw.json_metadata);
  const title = _resolveTitle(metadata);
  const body = _resolveBody(metadata);
  const tags = _collectTags(metadata);
  const timestamp = _resolveTimestamp(metadata);
  return {
    memoryId,
    timestamp,
    title,
    body,
    score,
    tags,
  };
}

function _parseMetadata(raw) {
  if (raw && typeof raw === 'object') {
    return raw;
  }
  if (typeof raw !== 'string') {
    return {};
  }
  const text = raw.trim();
  if (!text) {
    return {};
  }
  try {
    const parsed = JSON.parse(text);
    return typeof parsed === 'object' && parsed !== null ? parsed : {};
  } catch (_error) {
    return {};
  }
}

function _safeString(value) {
  if (typeof value !== 'string') {
    return '';
  }
  return value.trim();
}

function _safeNumber(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : 0;
}

function _resolveTitle(metadata) {
  const identity = metadata.identity && typeof metadata.identity === 'object'
    ? metadata.identity
    : null;
  const identityName = identity ? _safeString(identity.name) : '';
  const candidates = [
    identityName,
    metadata.name,
    metadata.title,
    metadata.memory_tag,
    metadata.topic,
    metadata.label,
    metadata.summary,
    metadata.body,
    metadata.kind,
  ];
  const title = _firstNonEmpty(candidates) || 'Memory entry';
  return _truncate(title, 120);
}

function _resolveBody(metadata) {
  const candidates = [
    metadata.summary,
    metadata.body,
    metadata.spoken_sentence,
    metadata.thought_sentence,
    metadata.situation_overview,
    metadata.notes,
    metadata.description,
    metadata.observation,
  ];
  const body = _firstNonEmpty(candidates) || '';
  return _truncate(body, 280);
}

function _collectTags(metadata) {
  const bucket = new Set();
  const arraySources = [metadata.tags, metadata.labels, metadata.source_topics];
  for (const source of arraySources) {
    if (!Array.isArray(source)) {
      continue;
    }
    for (const tag of source) {
      const text = _safeString(tag);
      if (text) {
        bucket.add(text);
      }
    }
  }
  const singleSources = [
    metadata.memory_tag,
    metadata.memory_collection_text,
    metadata.kind,
  ];
  for (const single of singleSources) {
    const text = _safeString(single);
    if (text) {
      bucket.add(text);
    }
  }
  if (metadata.identity && typeof metadata.identity === 'object') {
    const identityName = _safeString(metadata.identity.name);
    if (identityName) {
      bucket.add(identityName);
    }
    if (Array.isArray(metadata.identity.aliases)) {
      for (const alias of metadata.identity.aliases) {
        const text = _safeString(alias);
        if (text) {
          bucket.add(text);
        }
      }
    }
  }
  const metadataName = _safeString(metadata.name);
  if (metadataName) {
    bucket.add(metadataName);
  }
  return Array.from(bucket);
}

function _resolveTimestamp(metadata) {
  const keys = [
    'timestamp',
    'observed_at',
    'recorded_at',
    'created_at',
    'occurred_at',
    'time',
  ];
  const raw = _firstNonEmpty(keys.map((key) => metadata[key]));
  return raw || '';
}

function _firstNonEmpty(values) {
  for (const value of values) {
    const text = _safeString(value);
    if (text) {
      return text;
    }
  }
  return '';
}

function _truncate(text, limit) {
  if (text.length <= limit) {
    return text;
  }
  return `${text.slice(0, limit)}…`;
}

/**
 * @typedef {object} MemoryQueryPayload
 * @property {string} query Natural language query text.
 * @property {number} top_k Maximum number of results requested.
 */

/**
 * @typedef {object} MemoryStorePayload
 * @property {string} title Memory title.
 * @property {string} body Memory content.
 * @property {string[]} tags Optional tag list.
 */

/**
 * @typedef {object} RawRecallResult
 * @property {string} memory_id Identifier of the recalled memory node.
 * @property {number|string} score Similarity score returned by Qdrant.
 * @property {string|object} json_metadata Metadata emitted by the memory service.
 */

/**
 * @typedef {object} NormalisedRecallEntry
 * @property {string} memoryId Parsed memory identifier.
 * @property {string} timestamp Raw timestamp string, when available.
 * @property {string} title Human readable title derived from metadata.
 * @property {string} body Concise summary extracted from metadata.
 * @property {number} score Numeric similarity score.
 * @property {string[]} tags Collected tag list describing the memory.
 */
