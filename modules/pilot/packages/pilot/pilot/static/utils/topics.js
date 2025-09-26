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
