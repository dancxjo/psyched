/**
 * Extract the trailing segment from a ROS topic path.
 *
 * @param {string} streamName - ROS topic published by the conversant module.
 * @param {string} [fallback=''] - Value returned when no usable segment is found.
 * @returns {string} Thread identifier derived from the topic path.
 */
export function extractThreadIdFromStream(streamName, fallback = '') {
  if (typeof streamName !== 'string') {
    return fallback;
  }
  const parts = streamName
    .split('/')
    .map((part) => part.trim())
    .filter((part) => part.length > 0);
  if (!parts.length) {
    return fallback;
  }
  return parts[parts.length - 1];
}

/**
 * Normalise a list of conversation messages for rendering.
 *
 * @param {Array<{ role: string, content: string, timestamp: string, intent?: string, metadata?: Record<string, unknown> }>} messages
 * @returns {Array<{ role: string, content: string, timestamp: string, intent?: string, metadata?: Record<string, unknown> }>}
 */
export function normaliseMessages(messages) {
  if (!Array.isArray(messages)) {
    return [];
  }
  const normalised = [];
  for (const message of messages) {
    if (!message || typeof message !== 'object') {
      continue;
    }
    const role = typeof message.role === 'string' ? message.role.trim() : '';
    const content = typeof message.content === 'string' ? message.content : '';
    const timestamp = typeof message.timestamp === 'string' ? message.timestamp : '';
    if (!role || !content || !timestamp) {
      continue;
    }
    const entry = {
      role,
      content,
      timestamp,
    };
    if (message.intent && typeof message.intent === 'string' && message.intent.trim()) {
      entry.intent = message.intent.trim();
    }
    if (message.metadata && typeof message.metadata === 'object') {
      entry.metadata = { ...message.metadata };
    }
    normalised.push(entry);
  }
  return normalised;
}

/**
 * Parse the JSON payload published on a conversation thread topic.
 *
 * @param {string} payload - JSON payload from the ROS bridge.
 * @returns {{ threadId: string, userId: string, messages: ReturnType<typeof normaliseMessages> } | null}
 */
export function parseConversationSnapshot(payload) {
  if (typeof payload !== 'string' || !payload.trim()) {
    return null;
  }
  let parsed;
  try {
    parsed = JSON.parse(payload);
  } catch (_error) {
    return null;
  }
  if (!parsed || typeof parsed !== 'object') {
    return null;
  }
  const threadId = typeof parsed.thread_id === 'string' && parsed.thread_id.trim()
    ? parsed.thread_id.trim()
    : '';
  const userId = typeof parsed.user_id === 'string' && parsed.user_id.trim()
    ? parsed.user_id.trim()
    : '';
  const messages = normaliseMessages(parsed.messages);
  if (!threadId || !userId || !Array.isArray(parsed.messages)) {
    return null;
  }
  return {
    threadId,
    userId,
    messages,
  };
}
