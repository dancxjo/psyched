import { extname, join, resolve } from "$std/path/mod.ts";
import { parse as parseJsonc } from "$std/jsonc/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { parse as parseYaml } from "$std/yaml/mod.ts";

import type { NavigationLink } from "../navigation_types.ts";
import { hostsRoot as defaultHostsRoot } from "./paths.ts";

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

const HOST_CONFIG_EXTENSIONS = [
  ".json",
  ".jsonc",
  ".yaml",
  ".yml",
  ".toml",
] as const;

const STATIC_PREFIX_LINKS: ReadonlyArray<NavigationLink> = [
  { href: "/", label: "Home" },
  { href: "/modules/pilot", label: "Pilot" },
];

const STATIC_SUFFIX_LINKS: ReadonlyArray<NavigationLink> = [
  { href: "/psh/host", label: "Host" },
  { href: "/psh/mod", label: "Modules" },
  { href: "/psh/srv", label: "Services" },
  { href: "/psh/sys", label: "Systemd" },
];

const MODULE_DISPLAY_ORDER = [
  "chat",
  "ear",
  "imu",
  "faces",
  "foot",
  "gps",
  "eye",
  "memory",
  "nav",
  "voice",
  "viscera",
  "wifi",
  "will",
] as const;

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

function isRecord(value: unknown): value is Record<string, unknown> {
  return Boolean(value) && typeof value === "object" && !Array.isArray(value);
}

function pathExists(path: string): boolean {
  try {
    Deno.statSync(path);
    return true;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return false;
    }
    throw error;
  }
}

function findHostConfig(
  hostname: string,
  hostsDir: string,
): string | undefined {
  for (const extension of HOST_CONFIG_EXTENSIONS) {
    const candidate = join(hostsDir, `${hostname}${extension}`);
    if (pathExists(candidate)) {
      return candidate;
    }
  }
  return undefined;
}

function loadRawHostConfig(path: string): Record<string, unknown> {
  const text = Deno.readTextFileSync(path);
  const extension = extname(path).toLowerCase();
  let parsed: unknown;
  switch (extension) {
    case ".json":
    case ".jsonc":
      parsed = parseJsonc(text);
      break;
    case ".yaml":
    case ".yml":
      parsed = parseYaml(text);
      break;
    case ".toml":
    default:
      parsed = parseToml(text);
      break;
  }
  if (!isRecord(parsed)) {
    throw new Error(
      `Expected host manifest at ${path} to parse to an object, received ${typeof parsed}`,
    );
  }
  return parsed;
}

function coerceBoolean(value: unknown): boolean | undefined {
  if (typeof value === "boolean") return value;
  if (typeof value === "number") {
    if (Number.isNaN(value)) return undefined;
    return value !== 0;
  }
  if (typeof value === "string") {
    const normalized = value.trim().toLowerCase();
    if (!normalized) return undefined;
    if (["true", "yes", "on", "1"].includes(normalized)) return true;
    if (["false", "no", "off", "0"].includes(normalized)) return false;
    return undefined;
  }
  return undefined;
}

function moduleLaunchEnabled(raw: unknown): boolean {
  const direct = coerceBoolean(raw);
  if (direct !== undefined) {
    return direct;
  }
  if (isRecord(raw)) {
    const enabled = coerceBoolean(raw.enabled);
    if (enabled !== undefined) {
      return enabled;
    }
    if (raw.arguments !== undefined) {
      return true;
    }
    return Object.keys(raw).length > 0;
  }
  return false;
}

function gatherModuleEntries(
  raw: Record<string, unknown>,
): Map<string, unknown> {
  const modules = new Map<string, unknown>();

  const host = raw.host;
  if (isRecord(host)) {
    const declared = host.modules;
    if (Array.isArray(declared)) {
      for (const entry of declared) {
        if (typeof entry !== "string") continue;
        const name = entry.trim();
        if (!name) continue;
        if (!modules.has(name)) {
          modules.set(name, undefined);
        }
      }
    }
  }

  const moduleTable = raw.modules;
  if (isRecord(moduleTable)) {
    for (const [key, value] of Object.entries(moduleTable)) {
      const name = key.trim();
      if (!name) continue;
      modules.set(name, value);
    }
  }

  return modules;
}

function determineEnabledModules(raw: Record<string, unknown>): string[] {
  const modules = gatherModuleEntries(raw);
  const enabled = new Set<string>();

  for (const [name, value] of modules.entries()) {
    if (name === "pilot") continue;

    if (value === undefined) {
      continue;
    }

    if (!isRecord(value)) {
      if (coerceBoolean(value) === true) {
        enabled.add(name);
      }
      continue;
    }

    const launch = value.launch ?? value.launchConfig;
    if (moduleLaunchEnabled(launch)) {
      enabled.add(name);
    }
  }

  return sortModules(Array.from(enabled));
}

function sortModules(modules: string[]): string[] {
  if (modules.length <= 1) return modules;
  const order = new Map<string, number>();
  MODULE_DISPLAY_ORDER.forEach((name, index) => order.set(name, index));
  return modules.sort((a, b) => {
    const aIndex = order.get(a);
    const bIndex = order.get(b);
    if (aIndex !== undefined && bIndex !== undefined) return aIndex - bIndex;
    if (aIndex !== undefined) return -1;
    if (bIndex !== undefined) return 1;
    return a.localeCompare(b);
  });
}

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

function buildNavigation(moduleLinks: NavigationLink[]): NavigationLink[] {
  return [
    ...STATIC_PREFIX_LINKS.map(cloneLink),
    ...moduleLinks.map(cloneLink),
    ...STATIC_SUFFIX_LINKS.map(cloneLink),
  ];
}

function computeNavigation(
  hostname: string,
  hostsDir: string,
): NavigationComputation {
  const path = findHostConfig(hostname, hostsDir);
  if (!path) {
    return { links: buildNavigation([]) };
  }

  try {
    const raw = loadRawHostConfig(path);
    const modules = determineEnabledModules(raw);
    const moduleLinks = modules.map(moduleLinkFor);
    const stat = Deno.statSync(path);
    const mtimeMs = stat.mtime?.getTime() ?? 0;
    return {
      links: buildNavigation(moduleLinks),
      path,
      mtimeMs,
    };
  } catch (error) {
    console.warn(`Failed to read navigation from ${path}`, error);
    return { links: buildNavigation([]) };
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
