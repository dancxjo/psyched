import { resolve } from "$std/path/mod.ts";

import type { NavigationLink } from "../navigation_types.ts";
import { hostsRoot as defaultHostsRoot } from "./paths.ts";
import {
  determineEnabledModules,
  determineEnabledServices,
  resolveHostConfig,
} from "./host_config.ts";

export interface PrimaryNavigationOptions {
  hostname?: string;
  hostsDir?: string;
}

interface NavigationComputation {
  links: NavigationLink[];
  path?: string;
  mtimeMs?: number;
}

interface CacheEntry {
  hostname: string;
  hostsDir: string;
  path: string;
  mtimeMs: number;
  links: NavigationLink[];
}

const DEFAULT_HOSTS_DIR = resolve(defaultHostsRoot());

const STATIC_PREFIX_LINKS: ReadonlyArray<NavigationLink> = [
  { href: "/", label: "Home" },
];

const STATIC_SUFFIX_LINKS: ReadonlyArray<NavigationLink> = [
  { href: "/psh/host", label: "Host" },
  { href: "/psh/sys", label: "Systemd" },
];

const MODULE_MANAGEMENT_LINK: NavigationLink = {
  href: "/psh/mod",
  label: "Modules",
};

const SERVICE_MANAGEMENT_LINK: NavigationLink = {
  href: "/psh/srv",
  label: "Services",
};

const MODULE_NAV_LINKS: Readonly<Record<string, NavigationLink>> = {
  pilot: { href: "/modules/pilot", label: "Pilot" },
  chat: { href: "/modules/chat", label: "Chat" },
  ear: { href: "/modules/ear", label: "Ear" },
  imu: { href: "/modules/imu", label: "IMU" },
  faces: { href: "/modules/faces", label: "Faces" },
  foot: { href: "/modules/foot", label: "Foot" },
  gps: { href: "/modules/gps", label: "GPS" },
  eye: { href: "/modules/eye", label: "Eye" },
  memory: { href: "/modules/memory", label: "Memory" },
  nav: { href: "/modules/nav", label: "Nav" },
  voice: { href: "/modules/voice", label: "Voice" },
  viscera: { href: "/modules/viscera", label: "Viscera" },
  wifi: { href: "/modules/wifi", label: "Wi-Fi" },
  will: { href: "/modules/will", label: "Will" },
};

let cache: CacheEntry | undefined;

function defaultLabel(name: string): string {
  if (!name) return "";
  const parts = name.split(/[-_]/).filter(Boolean);
  if (!parts.length) return name;
  return parts.map((part) => {
    if (part.toUpperCase() === part) {
      return part;
    }
    if (part.length <= 3 && part.toLowerCase() === part) {
      return part.toUpperCase();
    }
    return part[0].toUpperCase() + part.slice(1);
  }).join(" ");
}

function moduleLinkFor(name: string): NavigationLink {
  const meta = MODULE_NAV_LINKS[name];
  if (meta) {
    return { ...meta };
  }
  return { href: `/modules/${name}`, label: defaultLabel(name) };
}

function cloneLink(link: NavigationLink): NavigationLink {
  return { href: link.href, label: link.label };
}

function defaultNavigation(): NavigationLink[] {
  return [
    { href: "/", label: "Home" },
    { href: "/modules/pilot", label: "Pilot" },
    { href: "/psh/host", label: "Host" },
    { href: "/psh/mod", label: "Modules" },
    { href: "/psh/srv", label: "Services" },
    { href: "/psh/sys", label: "Systemd" },
  ];
}

function buildNavigation(
  moduleLinks: NavigationLink[],
  options: { hasModules: boolean; hasServices: boolean },
): NavigationLink[] {
  const links: NavigationLink[] = [];
  links.push(...STATIC_PREFIX_LINKS.map(cloneLink));
  links.push(...moduleLinks.map(cloneLink));
  if (options.hasModules) {
    links.push(cloneLink(MODULE_MANAGEMENT_LINK));
  }
  if (options.hasServices) {
    links.push(cloneLink(SERVICE_MANAGEMENT_LINK));
  }
  links.push(...STATIC_SUFFIX_LINKS.map(cloneLink));
  return links;
}

function computeNavigation(
  hostname: string,
  hostsDir: string,
): NavigationComputation {
  const resolved = resolveHostConfig({ hostname, hostsDir });
  if (!resolved) {
    return { links: defaultNavigation() };
  }

  try {
    const modules = determineEnabledModules(resolved.raw, { includePilot: true });
    const services = determineEnabledServices(resolved.raw);
    const moduleLinks = modules.map(moduleLinkFor);
    return {
      links: buildNavigation(moduleLinks, {
        hasModules: modules.length > 0,
        hasServices: services.length > 0,
      }),
      path: resolved.path,
      mtimeMs: resolved.mtimeMs,
    };
  } catch (error) {
    console.warn(`Failed to read navigation from ${resolved.path}`, error);
    return { links: defaultNavigation() };
  }
}

export function primaryNavigationLinks(
  options: PrimaryNavigationOptions = {},
): NavigationLink[] {
  const hostname = options.hostname ?? Deno.hostname();
  const hostsDir = options.hostsDir
    ? resolve(options.hostsDir)
    : DEFAULT_HOSTS_DIR;

  if (cache && cache.hostname === hostname && cache.hostsDir === hostsDir) {
    try {
      const stat = Deno.statSync(cache.path);
      const mtimeMs = stat.mtime?.getTime() ?? 0;
      if (mtimeMs === cache.mtimeMs) {
        return cache.links.map(cloneLink);
      }
    } catch (error) {
      if (!(error instanceof Deno.errors.NotFound)) {
        console.warn(
          `Failed to stat cached host config at ${cache.path}`,
          error,
        );
      }
    }
  }

  const computed = computeNavigation(hostname, hostsDir);
  if (computed.path) {
    cache = {
      hostname,
      hostsDir,
      path: computed.path,
      mtimeMs: computed.mtimeMs ?? 0,
      links: computed.links.map(cloneLink),
    };
  } else {
    cache = undefined;
  }
  return computed.links;
}

export const __test__ = {
  resetCache(): void {
    cache = undefined;
  },
};
