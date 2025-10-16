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
