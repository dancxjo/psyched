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

export type EnabledServiceOptions = HostConfigLocatorOptions;

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

interface ModuleEntry {
  declared: boolean;
  value?: unknown;
}

interface ServiceEntry {
  declared: boolean;
  value?: unknown;
}

function gatherModuleEntries(
  raw: Record<string, unknown>,
): Map<string, ModuleEntry> {
  const modules = new Map<string, ModuleEntry>();

  const host = raw.host;
  if (isRecord(host)) {
    const declared = host.modules;
    if (Array.isArray(declared)) {
      for (const entry of declared) {
        if (typeof entry !== "string") continue;
        const name = entry.trim();
        if (!name) continue;
        const existing = modules.get(name);
        if (existing) {
          modules.set(name, { declared: true, value: existing.value });
        } else {
          modules.set(name, { declared: true });
        }
      }
    }
  }

  const moduleTable = raw.modules;
  if (isRecord(moduleTable)) {
    for (const [key, value] of Object.entries(moduleTable)) {
      const name = key.trim();
      if (!name) continue;
      const existing = modules.get(name);
      if (existing) {
        modules.set(name, { declared: existing.declared, value });
      } else {
        modules.set(name, { declared: false, value });
      }
    }
  }

  return modules;
}

function gatherServiceEntries(
  raw: Record<string, unknown>,
): Map<string, ServiceEntry> {
  const services = new Map<string, ServiceEntry>();

  const host = raw.host;
  if (isRecord(host)) {
    const declared = host.services;
    if (Array.isArray(declared)) {
      for (const entry of declared) {
        if (typeof entry !== "string") continue;
        const name = entry.trim();
        if (!name) continue;
        const existing = services.get(name);
        if (existing) {
          services.set(name, { declared: true, value: existing.value });
        } else {
          services.set(name, { declared: true });
        }
      }
    }
  }

  const serviceTable = raw.services;
  if (isRecord(serviceTable)) {
    for (const [key, value] of Object.entries(serviceTable)) {
      const name = key.trim();
      if (!name) continue;
      const existing = services.get(name);
      if (existing) {
        services.set(name, { declared: existing.declared, value });
      } else {
        services.set(name, { declared: false, value });
      }
    }
  }

  return services;
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
  const restrictToDeclared = Array.from(modules.values()).some((entry) =>
    entry.declared
  );

  for (const [name, entry] of modules.entries()) {
    if (!options.includePilot && name === "pilot") continue;
    if (restrictToDeclared && !entry.declared) continue;

    const value = entry.value;

    if (value === undefined) {
      if (entry.declared) {
        enabled.add(name);
      }
      continue;
    }

    const direct = coerceBoolean(value);
    if (direct !== undefined) {
      if (direct) {
        enabled.add(name);
      }
      continue;
    }

    if (isRecord(value)) {
      const launch = value.launch ?? value.launchConfig;
      if (launch === undefined) {
        if (entry.declared) {
          enabled.add(name);
        }
        continue;
      }
      if (moduleLaunchEnabled(launch)) {
        enabled.add(name);
      }
      continue;
    }

    if (entry.declared) {
      enabled.add(name);
    }
  }

  return sortModules(Array.from(enabled));
}

function serviceEnabled(raw: unknown): boolean {
  const direct = coerceBoolean(raw);
  if (direct !== undefined) {
    return direct;
  }
  if (isRecord(raw)) {
    const enabled = coerceBoolean(raw.enabled);
    if (enabled !== undefined) {
      return enabled;
    }
    return true;
  }
  return false;
}

function sortServices(services: string[]): string[] {
  if (services.length <= 1) return services;
  return services.sort((a, b) => a.localeCompare(b));
}

export function determineEnabledServices(raw: Record<string, unknown>): string[] {
  const services = gatherServiceEntries(raw);
  const enabled = new Set<string>();
  const restrictToDeclared = Array.from(services.values()).some((entry) =>
    entry.declared
  );

  for (const [name, entry] of services.entries()) {
    if (restrictToDeclared && !entry.declared) continue;

    const value = entry.value;

    if (value === undefined) {
      if (entry.declared) {
        enabled.add(name);
      }
      continue;
    }

    if (serviceEnabled(value)) {
      enabled.add(name);
      continue;
    }

    if (entry.declared) {
      enabled.add(name);
    }
  }

  return sortServices(Array.from(enabled));
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

export function enabledServicesForHost(
  options: EnabledServiceOptions = {},
): { services: string[]; path?: string; mtimeMs?: number } {
  const resolved = resolveHostConfig(options);
  if (!resolved) {
    return { services: [] };
  }
  const services = determineEnabledServices(resolved.raw);
  return { services, path: resolved.path, mtimeMs: resolved.mtimeMs };
}

export const __test__ = {
  coerceBoolean,
  moduleLaunchEnabled,
  gatherModuleEntries,
  sortModules,
};
