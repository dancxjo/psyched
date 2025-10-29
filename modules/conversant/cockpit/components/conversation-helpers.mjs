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

/**
 * Parse and normalise the LLM debug log payload published by Conversant.
 *
 * @param {string} payload - JSON payload emitted on the llm_log topic.
 * @returns {{ threadId: string, timestamp: string, systemMessage: string, hint: string, source: string, response: string, responseIntent: string, responseEscalate: boolean, chatMessages: Array<{ role: string, content: string }> } | null}
 */
export function parseLlmLog(payload) {
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
  const timestamp = typeof parsed.timestamp === 'string' ? parsed.timestamp : '';
  const systemMessage = typeof parsed.system_message === 'string' ? parsed.system_message : '';
  const hint = typeof parsed.hint === 'string' ? parsed.hint : '';
  const source = typeof parsed.source === 'string' ? parsed.source : '';
  const response = typeof parsed.response === 'string' ? parsed.response : '';
  const responseIntent = typeof parsed.response_intent === 'string' ? parsed.response_intent : '';
  const responseEscalate = typeof parsed.response_escalate === 'boolean'
    ? parsed.response_escalate
    : Boolean(parsed.response_escalate);

  if (!threadId || !systemMessage) {
    return null;
  }

  const chatMessagesRaw = Array.isArray(parsed.chat_messages) ? parsed.chat_messages : [];
  const chatMessages = [];
  for (const entry of chatMessagesRaw) {
    if (!entry || typeof entry !== 'object') {
      continue;
    }
    const role = typeof entry.role === 'string' && entry.role.trim() ? entry.role.trim() : '';
    const content = typeof entry.content === 'string' ? entry.content : '';
    if (!role || !content) {
      continue;
    }
    chatMessages.push({ role, content });
  }

  return {
    threadId,
    timestamp,
    systemMessage,
    hint,
    source,
    response,
    responseIntent,
    responseEscalate,
    chatMessages,
  };
}
