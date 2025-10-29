/**
 * Build navigation section descriptors for the cockpit sidebar.
 *
 * The helper mirrors the module sections rendered by <cockpit-app> so the
 * Alpine-powered navigation can keep pace even when the custom element fires
 * its update event before Alpine attaches listeners.
 *
 * @example
 * buildNavigationSections([{ name: 'imu', slug: 'imu', has_cockpit: true }])
 * // => [
 * //      { id: 'cockpit-config', label: 'Module Configuration', index: 0, url: '/config/' },
 * //      { id: 'module-imu', label: 'imu', index: 1, url: '/modules/imu/' }
 * //    ]
 *
 * @param {Array<object>} modules - Module payloads from the /api/modules endpoint.
 * @returns {Array<object>} Ordered navigation section descriptors.
 */
export function buildNavigationSections(modules) {
  const sections = [
    {
      id: 'cockpit-config',
      label: 'Module Configuration',
      index: 0,
      url: '/config/',
    },
  ];

  if (!Array.isArray(modules)) {
    return sections;
  }

  const collator = typeof Intl !== 'undefined' && typeof Intl.Collator === 'function'
    ? new Intl.Collator(undefined, { numeric: true, sensitivity: 'base' })
    : null;

  const prepared = [];

  for (const module of modules) {
    if (!module || typeof module !== 'object') {
      continue;
    }

    const name = typeof module.name === 'string' ? module.name.trim() : '';
    const slug = normaliseModuleSlug(module);
    if (!slug) {
      continue;
    }

    const displayName = typeof module.display_name === 'string' && module.display_name.trim()
      ? module.display_name.trim()
      : name || slug;

    const entry = {
      id: `module-${slug}`,
      label: displayName,
    };

    const dashboardUrl = typeof module.dashboard_url === 'string' && module.dashboard_url.trim()
      ? module.dashboard_url.trim()
      : '';
    const hasCockpit = Boolean(module.has_cockpit);

    if (dashboardUrl) {
      entry.url = dashboardUrl;
    } else if (hasCockpit) {
      entry.url = `/modules/${name || slug}/`;
    }

    prepared.push(entry);
  }

  prepared.sort((a, b) => {
    if (collator) {
      return collator.compare(a.label, b.label);
    }
    return a.label.localeCompare(b.label);
  });

  let index = 1;
  for (const entry of prepared) {
    sections.push({
      ...entry,
      index: index++,
    });
  }

  return sections;
}

/**
 * Derive a slug suitable for DOM identifiers and anchor fragments.
 *
 * @param {object | null | undefined} module Module metadata from /api/modules.
 * @returns {string} Sanitised slug or an empty string when one cannot be derived.
 */
export function normaliseModuleSlug(module) {
  if (!module || typeof module !== 'object') {
    return '';
  }
  const rawSlug = typeof module.slug === 'string' ? module.slug : '';
  const rawName = typeof module.name === 'string' ? module.name : '';
  const rawDisplay = typeof module.display_name === 'string' ? module.display_name : '';
  const source = rawSlug.trim() || rawName.trim() || rawDisplay.trim();
  if (!source) {
    return '';
  }
  const normalised = source
    .replace(/[_\s]+/g, '-')
    .replace(/[^\p{Letter}\p{Number}-]+/gu, '-')
    .replace(/-+/g, '-')
    .replace(/^-|-$/g, '')
    .toLowerCase();
  return normalised;
}
