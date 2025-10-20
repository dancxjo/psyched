/**
 * Build a copy-ready string from raw module log lines.
 *
 * @param {unknown[]} lines - Log lines returned by the backend.
 * @param {{ truncated?: boolean, updatedAt?: string | null }} [options]
 * @returns {string} Plaintext content ready for clipboard operations.
 *
 * @example
 * ```js
 * const text = formatModuleLogForCopy(['ready', 'running'], { truncated: true });
 * // text === 'Older entries truncated.\nready\nrunning'
 * ```
 */
export function formatModuleLogForCopy(lines, options = {}) {
  const { truncated = false, updatedAt = null } = options;
  const entries = Array.isArray(lines) ? lines : [];
  const content = entries.length
    ? entries.map((line) => (typeof line === 'string' ? line : String(line ?? ''))).join('\n')
    : 'No log entries captured yet.';
  const metadata = [];
  if (updatedAt) {
    metadata.push(`Updated at: ${updatedAt}`);
  }
  if (truncated) {
    metadata.push('Older entries truncated.');
  }
  return metadata.length ? `${metadata.join(' · ')}\n${content}` : content;
}

/**
 * Convert command log entries into a clipboard-friendly string.
 *
 * @param {Array<{time?: string, type?: string, message?: string, detail?: string}>} entries
 * @returns {string}
 *
 * @example
 * ```js
 * formatCommandLogForCopy([{ time: '10:00', type: 'pose', message: 'Sent goal' }]);
 * // => '[10:00] pose — Sent goal'
 * ```
 */
export function formatCommandLogForCopy(entries) {
  if (!Array.isArray(entries) || !entries.length) {
    return 'No commands dispatched yet.';
  }
  return entries
    .map((entry) => {
      const time = entry?.time ? String(entry.time) : '—';
      const type = entry?.type ? String(entry.type) : 'event';
      const message = entry?.message ? String(entry.message) : '';
      const detail = entry?.detail ? String(entry.detail) : '';
      const lines = [`[${time}] ${type} — ${message}`.trimEnd()];
      if (detail) {
        lines.push(detail);
      }
      return lines.join('\n');
    })
    .join('\n\n');
}

/**
 * Convert voice module events into a copyable payload.
 *
 * @param {Array<{label?: string, topic?: string, timestamp?: string}>} events
 * @returns {string}
 */
export function formatVoiceEventLogForCopy(events) {
  if (!Array.isArray(events) || !events.length) {
    return 'No events yet.';
  }
  return events
    .map((event, index) => {
      const label = event?.label ? String(event.label) : `Event ${index + 1}`;
      const topic = event?.topic ? String(event.topic) : 'unknown topic';
      const timestamp = event?.timestamp ? String(event.timestamp) : '—';
      return `[${timestamp}] ${label} (${topic})`;
    })
    .join('\n');
}

/**
 * Serialise conversation history for clipboard export.
 *
 * @param {Array<{role?: string, speaker?: string, content?: unknown, confidence?: number}>} messages
 * @returns {string}
 */
export function formatConversationHistoryForCopy(messages) {
  if (!Array.isArray(messages) || !messages.length) {
    return 'No conversation yet.';
  }
  return messages
    .map((entry, index) => {
      const role = (entry?.role ? String(entry.role) : 'unknown').toLowerCase();
      const speaker = entry?.speaker ? ` speaker:${String(entry.speaker)}` : '';
      const confidence = typeof entry?.confidence === 'number'
        ? ` confidence:${Math.round(entry.confidence * 100)}%`
        : '';
      const content = typeof entry?.content === 'string'
        ? entry.content
        : JSON.stringify(entry?.content ?? '');
      return `#${index + 1} role:${role}${speaker}${confidence}\n${content}`;
    })
    .join('\n\n');
}

/**
 * Attempt to copy text to the clipboard, falling back to a best-effort
 * `document.execCommand('copy')` invocation when the async Clipboard API is
 * unavailable.
 *
 * @param {string} text - The text to copy.
 * @param {{ clipboard?: { writeText?: (value: string) => Promise<void> | void }, document?: Document }} [dependencies]
 * @returns {Promise<boolean>} Resolves with `true` when the browser confirmed the copy.
 *
 * @example
 * ```js
 * await copyTextToClipboard('diagnostics ready');
 * ```
 */
export async function copyTextToClipboard(text, dependencies = {}) {
  const value = typeof text === 'string' ? text : String(text ?? '');
  const clipboard = dependencies.clipboard ?? globalThis?.navigator?.clipboard ?? null;
  if (clipboard && typeof clipboard.writeText === 'function') {
    await clipboard.writeText(value);
    return true;
  }
  const documentRef = dependencies.document ?? globalThis?.document ?? null;
  if (!documentRef || typeof documentRef.createElement !== 'function' || !documentRef.body) {
    return false;
  }
  try {
    const textarea = documentRef.createElement('textarea');
    textarea.value = value;
    textarea.setAttribute('readonly', '');
    textarea.style.position = 'absolute';
    textarea.style.left = '-9999px';
    documentRef.body.appendChild(textarea);
    if (typeof textarea.select === 'function') {
      textarea.select();
    }
    if (typeof textarea.setSelectionRange === 'function') {
      textarea.setSelectionRange(0, value.length);
    }
    const executed = typeof documentRef.execCommand === 'function'
      ? documentRef.execCommand('copy')
      : false;
    documentRef.body.removeChild(textarea);
    return Boolean(executed);
  } catch (_error) {
    return false;
  }
}
