/**
 * Helper utilities for the Pilot dashboard.
 */

/**
 * Convert a ROS time stamp object into a JavaScript Date.
 *
 * @param {{ sec?: number, nanosec?: number } | null | undefined} stamp
 * @returns {Date | null}
 */
export function parseRosStamp(stamp) {
  if (!stamp || typeof stamp !== "object") {
    return null;
  }
  const sec = Number(stamp.sec);
  const nanosec = Number(stamp.nanosec);
  if (!Number.isFinite(sec)) {
    return null;
  }
  const nanoseconds = Number.isFinite(nanosec) ? nanosec : 0;
  const millis = sec * 1000 + Math.floor(nanoseconds / 1e6);
  const result = new Date(millis);
  return Number.isNaN(result.getTime()) ? null : result;
}

/**
 * Normalise a FeelingIntent ROS message into a cockpit-friendly shape.
 *
 * @param {object | null | undefined} message
 * @returns {{
 *   id: string | null,
 *   stamp: Date | null,
 *   attitudeEmoji: string,
 *   spokenSentence: string,
 *   thoughtSentence: string,
 *   commandScript: string,
 *   goals: string[],
 *   moodDelta: string,
 *   sourceTopics: string[],
 *   memory: { emoji: string, text: string, raw: string },
 *   episodeId: string,
 *   situationId: string,
 * } | null}
 */
export function normaliseFeelingIntent(message) {
  if (!message || typeof message !== "object") {
    return null;
  }
  const cleanString = (
    value,
  ) => (typeof value === "string" ? value.trim() : "");
  const stamp = parseRosStamp(message.stamp);
  const goals = Array.isArray(message.goals)
    ? message.goals.filter((goal) => typeof goal === "string" && goal.trim())
      .map((goal) => goal.trim())
    : [];
  const sourceTopics = Array.isArray(message.source_topics)
    ? message.source_topics
      .filter((topic) => typeof topic === "string" && topic.trim())
      .map((topic) => topic.trim())
    : [];
  const episodeId = cleanString(message.episode_id);
  const situationId = cleanString(message.situation_id);
  const id = episodeId || situationId ||
    (stamp ? `intent-${stamp.getTime()}` : null);

  return {
    id,
    stamp,
    attitudeEmoji: cleanString(message.attitude_emoji),
    spokenSentence: cleanString(message.spoken_sentence),
    thoughtSentence: cleanString(message.thought_sentence),
    commandScript: cleanString(message.command_script),
    goals,
    moodDelta: cleanString(message.mood_delta),
    sourceTopics,
    memory: {
      emoji: cleanString(message.memory_collection_emoji),
      text: cleanString(message.memory_collection_text),
      raw: cleanString(message.memory_collection_raw),
    },
    episodeId,
    situationId,
  };
}

/**
 * Normalise the debug snapshot payload emitted by the pilot module.
 *
 * @param {object | null | undefined} snapshot
 * @returns {{
 *   status: string,
 *   heartbeat: Date | null,
 *   config: {
 *     debounceSeconds: number | null,
 *     windowSeconds: number | null,
 *     contextTopics: string[],
 *     sensationTopics: string[],
 *   },
 *   recentSensations: Array<{
 *     topic: string,
 *     kind: string,
 *     hint: string,
 *     jsonPayload: string,
 *     vectorLength: number,
 *   }>,
 *   scripts: Array<{
 *     id: string,
 *     status: string,
 *     startedAt: string,
 *     finishedAt: string,
 *     error: string,
 *     source: string,
 *     usedActions: string[],
 *     actions: Array<{
 *       action: string,
 *       status: string,
 *       response: unknown,
 *       timestamp: string,
 *     }>,
 *   }>,
 *   lastLLM: string,
 *   prompt: string,
 *   logs: string[],
 *   errors: string[],
 * }}
 */
export function normaliseDebugSnapshot(snapshot) {
  const cleanString = (
    value,
  ) => (typeof value === "string" ? value.trim() : "");
  const cleanStringArray = (value) =>
    Array.isArray(value)
      ? Array.from(
        new Set(
          value.filter((entry) => typeof entry === "string" && entry.trim())
            .map((entry) => entry.trim()),
        ),
      )
      : [];
  const data = snapshot && typeof snapshot === "object" ? snapshot : {};
  const status = cleanString(data.status) || "unknown";

  const heartbeatRaw = cleanString(data.heartbeat);
  let heartbeat = null;
  if (heartbeatRaw) {
    const parsed = new Date(heartbeatRaw);
    heartbeat = Number.isNaN(parsed.getTime()) ? null : parsed;
  }

  const configRaw = data.config && typeof data.config === "object"
    ? data.config
    : {};
  const debounceSecondsRaw = Number(configRaw.debounce_seconds);
  const windowSecondsRaw = Number(configRaw.window_seconds);

  const recentSensationsRaw = Array.isArray(data.recent_sensations)
    ? data.recent_sensations
    : [];
  const recentSensations = recentSensationsRaw
    .map((entry) => {
      if (!entry || typeof entry !== "object") {
        return null;
      }
      const topic = cleanString(entry.topic);
      if (!topic) {
        return null;
      }
      const vectorLength = Number.isFinite(Number(entry.vector_len))
        ? Number(entry.vector_len)
        : Array.isArray(entry.vector)
        ? entry.vector.length
        : 0;
      return {
        topic,
        kind: cleanString(entry.kind),
        hint: cleanString(entry.collection_hint),
        jsonPayload: cleanString(entry.json_payload),
        vectorLength,
      };
    })
    .filter(Boolean);

  const scriptsRaw = Array.isArray(data.scripts) ? data.scripts : [];
  const scripts = scriptsRaw
    .map((entry) => {
      if (!entry || typeof entry !== "object") {
        return null;
      }
      const actionsRaw = Array.isArray(entry.actions) ? entry.actions : [];
      const actions = actionsRaw
        .map((action) => {
          if (!action || typeof action !== "object") {
            return null;
          }
          return {
            action: cleanString(action.action),
            status: cleanString(action.status) || "unknown",
            response: action.response,
            timestamp: cleanString(action.timestamp),
          };
        })
        .filter(Boolean);
      return {
        id: cleanString(entry.id) || "",
        status: cleanString(entry.status) || "unknown",
        startedAt: cleanString(entry.started_at),
        finishedAt: cleanString(entry.finished_at),
        error: cleanString(entry.error),
        source: cleanString(entry.source),
        usedActions: cleanStringArray(entry.used_actions),
        actions,
      };
    })
    .filter(Boolean);

  const logs = cleanStringArray(data.logs);
  const errors = cleanStringArray(data.errors);
  const lastLLM = cleanString(data.last_llm);
  const prompt = cleanString(data.last_prompt);

  return {
    status,
    heartbeat,
    config: {
      debounceSeconds: Number.isFinite(debounceSecondsRaw)
        ? debounceSecondsRaw
        : null,
      windowSeconds: Number.isFinite(windowSecondsRaw)
        ? windowSecondsRaw
        : null,
      contextTopics: cleanStringArray(configRaw.context_topics),
      sensationTopics: cleanStringArray(configRaw.sensation_topics),
    },
    recentSensations,
    scripts,
    lastLLM,
    prompt,
    logs,
    errors,
  };
}

const COLLAPSED_STORAGE_KEY = "psyched::pilot-dashboard::collapsed";

function resolveStorage(storage) {
  if (storage && typeof storage === "object") {
    return storage;
  }
  if (typeof window !== "undefined" && window.localStorage) {
    return window.localStorage;
  }
  return null;
}

/**
 * Read the persisted set of collapsed pilot dashboard sections.
 *
 * @example
 * loadCollapsedCards({
 *   getItem: () => '["pilot-status"]',
 * });
 * // => Set { 'pilot-status' }
 *
 * @param {{ getItem?: (key: string) => string | null } | null | undefined} storage
 * @returns {Set<string>} Known collapsed section identifiers.
 */
export function loadCollapsedCards(storage) {
  const resolved = resolveStorage(storage);
  if (!resolved || typeof resolved.getItem !== "function") {
    return new Set();
  }

  try {
    const raw = resolved.getItem(COLLAPSED_STORAGE_KEY);
    if (!raw) {
      return new Set();
    }
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) {
      return new Set();
    }
    const entries = [];
    for (const value of parsed) {
      if (typeof value !== "string") {
        continue;
      }
      const trimmed = value.trim();
      if (!trimmed || entries.includes(trimmed)) {
        continue;
      }
      entries.push(trimmed);
    }
    return new Set(entries);
  } catch (_error) {
    return new Set();
  }
}

/**
 * Persist the collapsed pilot dashboard section identifiers.
 *
 * @example
 * persistCollapsedCards(new Set(['pilot-status']));
 * // => true (when storage is available)
 *
 * @param {Iterable<string>} ids Section identifiers to persist.
 * @param {{ setItem?: (key: string, value: string) => void } | null | undefined} storage
 * @returns {boolean} ``true`` when the identifiers were persisted.
 */
export function persistCollapsedCards(ids, storage) {
  const resolved = resolveStorage(storage);
  if (!resolved || typeof resolved.setItem !== "function") {
    return false;
  }

  const unique = [];
  for (const value of ids || []) {
    if (typeof value !== "string") {
      continue;
    }
    const trimmed = value.trim();
    if (!trimmed || unique.includes(trimmed)) {
      continue;
    }
    unique.push(trimmed);
  }

  try {
    resolved.setItem(COLLAPSED_STORAGE_KEY, JSON.stringify(unique));
    return true;
  } catch (_error) {
    return false;
  }
}
