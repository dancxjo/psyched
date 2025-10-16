const LOOPBACK_HOSTS = new Set(['127.0.0.1', 'localhost', '::1', '', '0.0.0.0']);

/**
 * Normalise host metadata returned by the cockpit backend.
 *
 * @param {object | null | undefined} hostRaw Host payload from /api/modules.
 * @returns {{ name: string, shortname: string }} Display-friendly metadata.
 */
export function normaliseHostMetadata(hostRaw) {
  const host = typeof hostRaw === 'object' && hostRaw !== null ? hostRaw : {};
  const rawName = typeof host.name === 'string' && host.name.trim() ? host.name.trim() : '';
  const rawShortname = typeof host.shortname === 'string' && host.shortname.trim()
    ? host.shortname.trim()
    : '';
  const shortname = rawShortname || (rawName ? rawName.split('.')[0] : '') || 'host';
  const name = rawName || shortname;
  return { name, shortname };
}

/**
 * Summarise module metadata for cockpit dashboards.
 *
 * @param {Array<object>} modulesRaw Modules array from /api/modules.
 * @returns {{ total: number, withCockpit: number, withoutCockpit: number, modules: Array<object> }}
 */
export function summariseModules(modulesRaw) {
  const modules = Array.isArray(modulesRaw) ? modulesRaw : [];
  const normalised = [];
  const seen = new Set();

  for (const entry of modules) {
    if (!entry || typeof entry !== 'object') {
      continue;
    }
    const name = typeof entry.name === 'string' ? entry.name.trim() : '';
    const slugCandidate = typeof entry.slug === 'string' ? entry.slug.trim() : '';
    const slug = slugCandidate || name;
    if (!slug) {
      continue;
    }
    if (seen.has(slug)) {
      continue;
    }
    seen.add(slug);

    const displayName = typeof entry.display_name === 'string' && entry.display_name.trim()
      ? entry.display_name.trim()
      : name || slug;
    const description = typeof entry.description === 'string' ? entry.description.trim() : '';
    const hasCockpit = Boolean(entry.has_cockpit);
    const dashboardUrlRaw = typeof entry.dashboard_url === 'string' ? entry.dashboard_url.trim() : '';
    const dashboardUrl = dashboardUrlRaw || (hasCockpit && name ? `/modules/${name}/` : '');

    const systemd = normaliseSystemdStatus(entry.systemd);

    normalised.push({ name, slug, displayName, description, hasCockpit, dashboardUrl, systemd });
  }

  normalised.sort((a, b) => a.displayName.localeCompare(b.displayName, undefined, { sensitivity: 'base' }));

  const withCockpit = normalised.filter((module) => module.hasCockpit).length;
  const total = normalised.length;
  const withoutCockpit = total - withCockpit;

  return { total, withCockpit, withoutCockpit, modules: normalised };
}

/**
 * Normalise bridge configuration so dashboards can surface operator hints.
 *
 * @param {object | null | undefined} bridgeRaw Bridge metadata from /api/modules.
 * @param {Location | { protocol?: string, hostname?: string, href?: string } | null} [location]
 *   Optional location-like value used to resolve relative rosbridge URLs during tests.
 * @returns {{
 *   mode: string,
 *   rosbridgeUri: string,
 *   effectiveRosbridgeUri: string,
 *   videoBase: string,
 *   videoPort: number | null,
 * }}
 */
export function normaliseBridgeSettings(bridgeRaw, location = typeof window !== 'undefined' ? window.location : undefined) {
  const bridge = typeof bridgeRaw === 'object' && bridgeRaw !== null ? bridgeRaw : {};
  const mode = typeof bridge.mode === 'string' && bridge.mode.trim() ? bridge.mode.trim() : 'rosbridge';
  const rosbridgeUri = typeof bridge.rosbridge_uri === 'string' && bridge.rosbridge_uri.trim()
    ? bridge.rosbridge_uri.trim()
    : 'ws://127.0.0.1:9090';
  const videoBase = typeof bridge.video_base === 'string' && bridge.video_base.trim() ? bridge.video_base.trim() : '';
  const videoPort = Number.isFinite(bridge.video_port) ? Number(bridge.video_port) : null;

  const effectiveRosbridgeUri = resolveRosbridgeUri(rosbridgeUri, location);

  return { mode, rosbridgeUri, effectiveRosbridgeUri, videoBase, videoPort };
}

function resolveRosbridgeUri(rawUri, location) {
  const fallbackProtocol = location && location.protocol === 'https:' ? 'wss:' : 'ws:';
  const fallbackHost = location && location.hostname ? location.hostname : '127.0.0.1';
  const baseHref = location && location.href ? location.href : undefined;

  try {
    const url = new URL(rawUri, baseHref);
    if (url.protocol === 'http:' || url.protocol === 'https:') {
      url.protocol = fallbackProtocol;
    }
    if (LOOPBACK_HOSTS.has(url.hostname)) {
      url.hostname = fallbackHost;
    }
    if (!url.port) {
      const parsedPort = extractPort(rawUri, baseHref);
      url.port = parsedPort || '9090';
    }
    if (url.protocol !== 'ws:' && url.protocol !== 'wss:') {
      url.protocol = fallbackProtocol;
    }
    return url.toString();
  } catch (_error) {
    return `${fallbackProtocol}//${fallbackHost}:9090`;
  }
}

function extractPort(rawUri, baseHref) {
  try {
    const url = new URL(rawUri, baseHref);
    return url.port;
  } catch (_error) {
    return '';
  }
}

export function normaliseSystemdStatus(rawStatus) {
  const status = typeof rawStatus === 'object' && rawStatus !== null ? rawStatus : {};
  const supported = Boolean(status.supported);
  const exists = Boolean(status.exists && supported);
  const active = exists && Boolean(status.active);
  const enabled = exists && Boolean(status.enabled);

  const unit = typeof status.unit === 'string' ? status.unit.trim() : '';
  const loadState = typeof status.load_state === 'string' ? status.load_state.trim() : '';
  const activeState = typeof status.active_state === 'string' ? status.active_state.trim() : '';
  const subState = typeof status.sub_state === 'string' ? status.sub_state.trim() : '';
  const unitFileState = typeof status.unit_file_state === 'string' ? status.unit_file_state.trim() : '';
  const message = typeof status.message === 'string' ? status.message.trim() : '';

  return {
    supported,
    exists,
    active,
    enabled,
    unit,
    loadState: loadState || (supported ? 'unknown' : ''),
    activeState: activeState || (supported ? 'inactive' : ''),
    subState: subState || (supported ? 'dead' : ''),
    unitFileState: unitFileState || (supported ? 'disabled' : ''),
    message,
  };
}

export const __test__ = { resolveRosbridgeUri, normaliseSystemdStatus };
