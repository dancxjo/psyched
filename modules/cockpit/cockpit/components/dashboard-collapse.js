const STORAGE_PREFIX = "psyched::cockpit::dashboard::collapsed::";
const DEFAULTS_PREFIX = "psyched::cockpit::dashboard::defaults::";

function resolveStorageArea(storage) {
  if (storage && typeof storage === "object") {
    return storage;
  }
  if (typeof window !== "undefined" && window.localStorage) {
    return window.localStorage;
  }
  return null;
}

/**
 * Normalize a dashboard scope identifier for use when persisting collapse state.
 *
 * @example
 * normaliseDashboardScope(' Module-Pilot ');
 * // => 'module-pilot'
 *
 * @param {string | null | undefined} scope Raw scope identifier supplied by callers.
 * @returns {string} Slug suitable for use in localStorage keys.
 */
export function normaliseDashboardScope(scope) {
  const raw = typeof scope === "string"
    ? scope
    : scope != null
      ? String(scope)
      : "";
  const trimmed = raw.trim().toLowerCase();
  const slug = trimmed.replace(/[^a-z0-9]+/g, "-").replace(/^-+|-+$/g, "");
  return slug || "cockpit";
}

function composeStorageKey(scope) {
  return `${STORAGE_PREFIX}${normaliseDashboardScope(scope)}`;
}

function composeDefaultsKey(scope) {
  return `${DEFAULTS_PREFIX}${normaliseDashboardScope(scope)}`;
}

/**
 * Convert a card identifier or heading into a predictable slug.
 *
 * @example
 * normaliseCardId(' Pilot Status ');
 * // => 'pilot-status'
 *
 * @param {string | null | undefined} value Candidate identifier.
 * @returns {string} Normalised slug. Returns an empty string when no slug can be produced.
 */
export function normaliseCardId(value) {
  if (typeof value !== "string") {
    if (value == null) {
      return "";
    }
    if (typeof value.textContent === "string") {
      return normaliseCardId(value.textContent);
    }
    return normaliseCardId(String(value));
  }
  const trimmed = value.trim().toLowerCase();
  const slug = trimmed.replace(/[^a-z0-9]+/g, "-").replace(/^-+|-+$/g, "");
  return slug;
}

/**
 * Read the persisted set of collapsed card identifiers for a given scope.
 *
 * @example
 * loadCollapsedCardSet('module-pilot', storage);
 * // => Set { 'pilot-status' }
 *
 * @param {string | null | undefined} scope Dashboard scope identifier.
 * @param {{ getItem?: (key: string) => string | null } | null | undefined} [storage]
 *   Optional storage provider; defaults to window.localStorage in the browser.
 * @returns {Set<string>} Known collapsed card identifiers for the requested scope.
 */
export function loadCollapsedCardSet(scope, storage) {
  const resolved = resolveStorageArea(storage);
  if (!resolved || typeof resolved.getItem !== "function") {
    return new Set();
  }

  try {
    const raw = resolved.getItem(composeStorageKey(scope));
    if (!raw) {
      return new Set();
    }
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) {
      return new Set();
    }
    const entries = [];
    for (const candidate of parsed) {
      const slug = normaliseCardId(candidate);
      if (!slug || entries.includes(slug)) {
        continue;
      }
      entries.push(slug);
    }
    return new Set(entries);
  } catch (_error) {
    return new Set();
  }
}

function loadDefaultAppliedSet(scope, storage) {
  const resolved = resolveStorageArea(storage);
  if (!resolved || typeof resolved.getItem !== "function") {
    return new Set();
  }

  try {
    const raw = resolved.getItem(composeDefaultsKey(scope));
    if (!raw) {
      return new Set();
    }
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) {
      return new Set();
    }
    const entries = [];
    for (const candidate of parsed) {
      const slug = normaliseCardId(candidate);
      if (!slug || entries.includes(slug)) {
        continue;
      }
      entries.push(slug);
    }
    return new Set(entries);
  } catch (_error) {
    return new Set();
  }
}

/**
 * Persist the collapsed card identifiers for a dashboard scope.
 *
 * @example
 * persistCollapsedCardSet('module-pilot', ['pilot-status']);
 * // => true (when storage is available)
 *
 * @param {string | null | undefined} scope Dashboard scope identifier.
 * @param {Iterable<string>} ids Card identifiers to persist.
 * @param {{ setItem?: (key: string, value: string) => void } | null | undefined} [storage]
 *   Optional storage provider; defaults to window.localStorage in the browser.
 * @returns {boolean} ``true`` when the identifiers were persisted.
 */
export function persistCollapsedCardSet(scope, ids, storage) {
  const resolved = resolveStorageArea(storage);
  if (!resolved || typeof resolved.setItem !== "function") {
    return false;
  }

  const unique = [];
  for (const value of ids || []) {
    const slug = normaliseCardId(value);
    if (!slug || unique.includes(slug)) {
      continue;
    }
    unique.push(slug);
  }

  try {
    resolved.setItem(composeStorageKey(scope), JSON.stringify(unique));
    return true;
  } catch (_error) {
    return false;
  }
}

function persistDefaultAppliedSet(scope, ids, storage) {
  const resolved = resolveStorageArea(storage);
  if (!resolved || typeof resolved.setItem !== "function") {
    return false;
  }

  const unique = [];
  for (const value of ids || []) {
    const slug = normaliseCardId(value);
    if (!slug || unique.includes(slug)) {
      continue;
    }
    unique.push(slug);
  }

  try {
    resolved.setItem(composeDefaultsKey(scope), JSON.stringify(unique));
    return true;
  } catch (_error) {
    return false;
  }
}

/**
 * Create a helper that manages collapsed card state for a dashboard scope.
 *
 * @example
 * const manager = createCollapsedCardManager('module-pilot');
 * manager.setCollapsed('pilot-status', true);
 * manager.toggle('pilot-intent');
 *
 * @param {string | null | undefined} scope Dashboard scope identifier.
 * @param {Storage | { getItem?: (key: string) => string | null, setItem?: (key: string, value: string) => void } | null} [storage]
 *   Optional storage provider; defaults to window.localStorage in the browser.
 * @returns {{
 *   scope: string,
 *   storageKey: string,
 *   getCollapsedIds: () => Set<string>,
 *   isCollapsed: (id: string) => boolean,
 *   setCollapsed: (id: string, collapsed: boolean) => Set<string>,
 *   toggle: (id: string) => Set<string>,
 * }}
 */
export function createCollapsedCardManager(scope, storage) {
  const normalisedScope = normaliseDashboardScope(scope);
  const resolvedStorage = resolveStorageArea(storage);
  let collapsed = loadCollapsedCardSet(normalisedScope, resolvedStorage);
  let defaultsApplied = loadDefaultAppliedSet(normalisedScope, resolvedStorage);

  const sync = (next) => {
    collapsed = next;
    persistCollapsedCardSet(normalisedScope, next, resolvedStorage);
    return new Set(collapsed);
  };

  const syncDefaults = (next) => {
    defaultsApplied = next;
    persistDefaultAppliedSet(normalisedScope, next, resolvedStorage);
    return new Set(defaultsApplied);
  };

  return {
    scope: normalisedScope,
    storageKey: composeStorageKey(normalisedScope),
    getCollapsedIds() {
      return new Set(collapsed);
    },
    getDefaultAppliedIds() {
      return new Set(defaultsApplied);
    },
    isCollapsed(id) {
      const slug = normaliseCardId(id);
      return slug ? collapsed.has(slug) : false;
    },
    isDefaultApplied(id) {
      const slug = normaliseCardId(id);
      return slug ? defaultsApplied.has(slug) : false;
    },
    setCollapsed(id, collapsedState) {
      const slug = normaliseCardId(id);
      if (!slug) {
        return new Set(collapsed);
      }
      const next = new Set(collapsed);
      if (collapsedState) {
        next.add(slug);
      } else {
        next.delete(slug);
      }
      return sync(next);
    },
    markDefaultApplied(id) {
      const slug = normaliseCardId(id);
      if (!slug) {
        return new Set(defaultsApplied);
      }
      if (defaultsApplied.has(slug)) {
        return new Set(defaultsApplied);
      }
      const next = new Set(defaultsApplied);
      next.add(slug);
      return syncDefaults(next);
    },
    toggle(id) {
      const slug = normaliseCardId(id);
      if (!slug) {
        return new Set(collapsed);
      }
      const next = new Set(collapsed);
      if (next.has(slug)) {
        next.delete(slug);
      } else {
        next.add(slug);
      }
      return sync(next);
    },
  };
}

export const __test__ = {
  composeStorageKey,
  resolveStorageArea,
};
