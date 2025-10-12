/**
 * Utilities for normalising module regime metadata.
 */

export const DEFAULT_REGIME = 'general';

export const REGIME_ORDER = [
  'system',
  'audio',
  'conversation',
  'navigation',
  'behavior',
  'perception',
  'mobility',
];

export const REGIME_LABELS = {
  system: 'System',
  audio: 'Audio',
  conversation: 'Conversation',
  navigation: 'Navigation',
  behavior: 'Behavior',
  perception: 'Perception',
  mobility: 'Mobility',
  general: 'General',
};

/**
 * Coerce a module's regime declaration into a list of strings.
 */
export function normalizeRegimes(value) {
  if (Array.isArray(value)) {
    const cleaned = value.map((item) => `${item}`.trim()).filter(Boolean);
    return cleaned.length ? cleaned : [DEFAULT_REGIME];
  }
  if (typeof value === 'string') {
    const trimmed = value.trim();
    return trimmed ? [trimmed] : [DEFAULT_REGIME];
  }
  if (value == null) {
    return [DEFAULT_REGIME];
  }
  const coerced = `${value}`.trim();
  return coerced ? [coerced] : [DEFAULT_REGIME];
}

/**
 * Present a human-friendly name for a regime slug.
 */
export function formatRegimeName(slug) {
  if (slug in REGIME_LABELS) {
    return REGIME_LABELS[slug];
  }
  return slug
    .split(/[-_\s]+/)
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(' ');
}

/**
 * Group modules by regime while respecting the preferred ordering.
 */
export function buildRegimeGroups(modules) {
  const groups = new Map();
  for (const module of modules) {
    const regimes = normalizeRegimes(module?.regimes);
    for (const regime of regimes) {
      if (!groups.has(regime)) {
        groups.set(regime, []);
      }
      groups.get(regime).push(module);
    }
  }

  const orderedKeys = [
    ...REGIME_ORDER,
    ...Array.from(groups.keys()).filter((regime) => !REGIME_ORDER.includes(regime)).sort(),
  ];

  return orderedKeys
    .filter((regime) => groups.has(regime))
    .map((regime) => ({
      regime,
      label: formatRegimeName(regime),
      modules: groups.get(regime),
    }));
}
