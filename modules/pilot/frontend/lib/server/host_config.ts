import { extname, join, resolve } from "$std/path/mod.ts";
import { parse as parseJsonc } from "$std/jsonc/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { parse as parseYaml } from "$std/yaml/mod.ts";

import { hostsRoot as defaultHostsRoot } from "./paths.ts";

export interface HostConfigLocatorOptions {
  hostname?: string;
  hostsDir?: string;
}

export interface HostConfigDetails {
  path: string;
  raw: Record<string, unknown>;
  mtimeMs: number;
}

export interface EnabledModuleOptions extends HostConfigLocatorOptions {
  includePilot?: boolean;
}

const DEFAULT_HOSTS_DIR = resolve(defaultHostsRoot());

const HOST_CONFIG_EXTENSIONS = [
  ".json",
  ".jsonc",
  ".yaml",
  ".yml",
  ".toml",
] as const;

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

export function findHostConfig(
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

export function loadRawHostConfig(path: string): Record<string, unknown> {
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

export function resolveHostConfig(
  options: HostConfigLocatorOptions = {},
): HostConfigDetails | undefined {
  const hostname = options.hostname ?? Deno.hostname();
  const hostsDir = options.hostsDir
    ? resolve(options.hostsDir)
    : DEFAULT_HOSTS_DIR;
  const path = findHostConfig(hostname, hostsDir);
  if (!path) return undefined;
  const raw = loadRawHostConfig(path);
  const stat = Deno.statSync(path);
  const mtimeMs = stat.mtime?.getTime() ?? 0;
  return { path, raw, mtimeMs };
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

function sortModules(modules: string[]): string[] {
  if (modules.length <= 1) return modules;
  const order = new Map<string, number>();
  const displayOrder = [
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
  displayOrder.forEach((name, index) => order.set(name, index));
  return modules.sort((a, b) => {
    const aIndex = order.get(a);
    const bIndex = order.get(b);
    if (aIndex !== undefined && bIndex !== undefined) return aIndex - bIndex;
    if (aIndex !== undefined) return -1;
    if (bIndex !== undefined) return 1;
    return a.localeCompare(b);
  });
}

export function determineEnabledModules(
  raw: Record<string, unknown>,
  options: { includePilot?: boolean } = {},
): string[] {
  const modules = gatherModuleEntries(raw);
  const enabled = new Set<string>();

  for (const [name, value] of modules.entries()) {
    if (!options.includePilot && name === "pilot") continue;

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

export function enabledModulesForHost(
  options: EnabledModuleOptions = {},
): { modules: string[]; path?: string; mtimeMs?: number } {
  const includePilot = options.includePilot ?? false;
  const resolved = resolveHostConfig(options);
  if (!resolved) {
    return { modules: [] };
  }
  const modules = determineEnabledModules(resolved.raw, { includePilot });
  return { modules, path: resolved.path, mtimeMs: resolved.mtimeMs };
}

export const __test__ = {
  coerceBoolean,
  moduleLaunchEnabled,
  gatherModuleEntries,
  sortModules,
};
