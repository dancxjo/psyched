import { extname, join } from "$std/path/mod.ts";
import { parse as parseJsonc } from "$std/jsonc/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { parse as parseYaml } from "$std/yaml/mod.ts";
import { hostsRoot } from "./paths.ts";

/** Supported host manifest file extension. */
const HOST_CONFIG_EXTENSIONS = [
    ".toml",
    ".json",
    ".jsonc",
    ".yaml",
    ".yml",
] as const;

type HostConfigExtension = typeof HOST_CONFIG_EXTENSIONS[number];

export type HostConfigEntry = Record<string, unknown>;
export type HostConfigScopes = Record<string, Record<string, HostConfigEntry>>;

export interface ModuleLaunchConfig {
    enabled?: boolean;
    arguments?: Record<string, unknown>;
    [key: string]: unknown;
}

export interface ModuleDirective {
    name: string;
    setup?: boolean;
    launch?: boolean;
    env?: Record<string, string>;
    depends_on?: string[];
    launchConfig?: ModuleLaunchConfig;
}

export interface ServiceDirective {
    name: string;
    setup?: boolean;
    up?: boolean;
    env?: Record<string, string>;
    depends_on?: string[];
    compose?: string | string[];
    compose_files?: string[];
}

export interface HostConfig {
    host: {
        name: string;
        roles?: string[];
        installers?: string[];
        modules?: string[];
        services?: string[];
        [key: string]: unknown;
    };
    provision?: { scripts?: string[]; installers?: string[] };
    modules?: ModuleDirective[];
    services?: ServiceDirective[];
    config?: HostConfigScopes;
}

export class HostConfigFormatError extends Error {
    path: string;

    constructor(path: string, message: string) {
        super(`Host config ${path}: ${message}`);
        this.name = "HostConfigFormatError";
        this.path = path;
    }
}

export class HostConfigNotFoundError extends Error {
    hostname: string;

    constructor(hostname: string) {
        super(`No host config found for '${hostname}'.`);
        this.name = "HostConfigNotFoundError";
        this.hostname = hostname;
    }
}

type RawHostConfig = {
    host?: unknown;
    provision?: unknown;
    modules?: unknown;
    services?: unknown;
    config?: unknown;
    [key: string]: unknown;
};

interface NormalizationContext {
    path: string;
}

function describeType(value: unknown): string {
    if (Array.isArray(value)) return "array";
    if (value === null) return "null";
    return typeof value;
}

function isRecord(value: unknown): value is Record<string, unknown> {
    return !!value && typeof value === "object" && !Array.isArray(value);
}

function coerceName(value: unknown): string | undefined {
    if (typeof value !== "string") return undefined;
    const trimmed = value.trim();
    return trimmed.length ? trimmed : undefined;
}

function coerceBoolean(value: unknown): boolean | undefined {
    if (typeof value === "boolean") return value;
    if (typeof value === "string") {
        const normalized = value.trim().toLowerCase();
        if (!normalized) return undefined;
        if (["true", "yes", "on", "1"].includes(normalized)) return true;
        if (["false", "no", "off", "0"].includes(normalized)) return false;
        return undefined;
    }
    if (typeof value === "number") {
        if (Number.isNaN(value)) return undefined;
        return value !== 0;
    }
    return undefined;
}

function normalizeModuleLaunchConfig(
    raw: unknown,
    context: NormalizationContext,
    location: string,
): { launch?: boolean; launchConfig?: ModuleLaunchConfig } {
    if (raw === undefined || raw === null) return {};

    if (typeof raw === "boolean") {
        return { launch: raw };
    }

    if (isRecord(raw)) {
        const config: ModuleLaunchConfig = { ...raw } as ModuleLaunchConfig;
        const enabledRaw = config.enabled;
        let enabled = coerceBoolean(enabledRaw);
        if (enabledRaw !== undefined && enabled === undefined) {
            throw new HostConfigFormatError(
                context.path,
                `Expected ${location}.enabled to be a boolean, received ${describeType(enabledRaw)
                }`,
            );
        }

        if (config.arguments !== undefined && !isRecord(config.arguments)) {
            throw new HostConfigFormatError(
                context.path,
                `Expected ${location}.arguments to be a table, received ${describeType(config.arguments)
                }`,
            );
        }

        if (enabled === undefined && config.arguments !== undefined) {
            enabled = true;
        }

        const normalizedConfig: ModuleLaunchConfig = { ...config };
        if (enabled !== undefined) {
            normalizedConfig.enabled = enabled;
        }

        return {
            launch: enabled,
            launchConfig: normalizedConfig,
        };
    }

    const coerced = coerceBoolean(raw);
    if (coerced !== undefined) {
        return { launch: coerced };
    }

    throw new HostConfigFormatError(
        context.path,
        `Expected ${location} to be a boolean or table, received ${describeType(raw)
        }`,
    );
}

function normalizeModuleDirective(
    raw: unknown,
    context: NormalizationContext,
    location: string,
    fallbackName?: string,
    defaultLaunch?: boolean,
): ModuleDirective {
    if (raw === undefined || raw === null) {
        raw = {};
    }

    if (!isRecord(raw)) {
        throw new HostConfigFormatError(
            context.path,
            `Expected ${location} to be a table, received ${describeType(raw)}.`,
        );
    }

    const explicit = coerceName(raw.name);
    const name = explicit ?? coerceName(fallbackName ?? "");
    if (!name) {
        throw new HostConfigFormatError(
            context.path,
            `Module entry ${location} is missing a name.`,
        );
    }

    const normalized = { ...raw } as Record<string, unknown>;
    const { launch, launchConfig } = normalizeModuleLaunchConfig(
        normalized["launch"],
        context,
        `${location}.launch`,
    );

    let launchValue = launch;
    if (launchValue === undefined && defaultLaunch === true) {
        launchValue = true;
    }

    if (launchValue === undefined) {
        delete normalized["launch"];
    } else {
        normalized["launch"] = launchValue;
    }

    if (launchConfig) {
        normalized["launchConfig"] = launchConfig;
    } else {
        delete normalized["launchConfig"];
    }

    return { ...normalized, name } as ModuleDirective;
}

function normalizeModuleDirectives(
    hostModuleNames: readonly string[] | undefined,
    raw: unknown,
    context: NormalizationContext,
): ModuleDirective[] {
    const byName = new Map<string, unknown>();
    const tableOrder: string[] = [];
    if (raw !== undefined && raw !== null) {
        if (!isRecord(raw)) {
            throw new HostConfigFormatError(
                context.path,
                `Expected 'modules' to be a table, received ${describeType(raw)}.`,
            );
        }

        for (const [moduleName, value] of Object.entries(raw)) {
            byName.set(moduleName, value);
            tableOrder.push(moduleName);
        }
    }

    const result: ModuleDirective[] = [];
    const seen = new Set<string>();
    const declared = hostModuleNames ?? [];
    const restrictToDeclared = hostModuleNames !== undefined;

    for (const moduleName of declared) {
        const name = coerceName(moduleName);
        if (!name || seen.has(name)) continue;
        seen.add(name);
        const rawEntry = byName.get(name);
        if (rawEntry !== undefined) {
            byName.delete(name);
        }
        result.push(
            normalizeModuleDirective(
                rawEntry,
                context,
                `modules.${name}`,
                name,
                true,
            ),
        );
    }

    if (!restrictToDeclared) {
        for (const moduleName of tableOrder) {
            const name = coerceName(moduleName);
            if (!name || seen.has(name)) continue;
            seen.add(name);
            const rawEntry = byName.get(name);
            if (rawEntry !== undefined) {
                byName.delete(name);
            }
            result.push(
                normalizeModuleDirective(
                    rawEntry,
                    context,
                    `modules.${name}`,
                    name,
                ),
            );
        }

        for (const [moduleName, rawEntry] of byName.entries()) {
            const name = coerceName(moduleName);
            if (!name || seen.has(name)) continue;
            seen.add(name);
            result.push(
                normalizeModuleDirective(
                    rawEntry,
                    context,
                    `modules.${name}`,
                    name,
                ),
            );
        }
    }

    return result;
}

function normalizeServiceDirective(
    raw: unknown,
    context: NormalizationContext,
    location: string,
    fallbackName?: string,
    defaultUp?: boolean,
): ServiceDirective {
    if (raw === undefined || raw === null) {
        raw = {};
    }

    if (!isRecord(raw)) {
        throw new HostConfigFormatError(
            context.path,
            `Expected ${location} to be a table, received ${describeType(raw)}.`,
        );
    }

    const explicit = coerceName(raw.name);
    const name = explicit ?? coerceName(fallbackName ?? "");
    if (!name) {
        throw new HostConfigFormatError(
            context.path,
            `Service entry ${location} is missing a name.`,
        );
    }

    const normalized = { ...raw } as Record<string, unknown>;
    const upRaw = normalized["up"];
    let up = coerceBoolean(upRaw);
    if (upRaw !== undefined && up === undefined) {
        throw new HostConfigFormatError(
            context.path,
            `Expected ${location}.up to be a boolean, received ${describeType(upRaw)
            }`,
        );
    }

    if (up === undefined && defaultUp === true) {
        up = true;
    }

    if (up === undefined) {
        delete normalized["up"];
    } else {
        normalized["up"] = up;
    }

    return { ...normalized, name } as ServiceDirective;
}

function normalizeServiceDirectives(
    hostServiceNames: readonly string[] | undefined,
    raw: unknown,
    context: NormalizationContext,
): ServiceDirective[] {
    const byName = new Map<string, unknown>();
    if (raw !== undefined && raw !== null) {
        if (!isRecord(raw)) {
            throw new HostConfigFormatError(
                context.path,
                `Expected 'services' to be a table, received ${describeType(raw)}.`,
            );
        }

        for (const [serviceName, value] of Object.entries(raw)) {
            byName.set(serviceName, value);
        }
    }

    const result: ServiceDirective[] = [];
    const declared = hostServiceNames ?? [];
    for (const serviceName of declared) {
        if (!serviceName) continue;
        if (byName.has(serviceName)) {
            const rawEntry = byName.get(serviceName);
            byName.delete(serviceName);
            result.push(
                normalizeServiceDirective(
                    rawEntry,
                    context,
                    `services.${serviceName}`,
                    serviceName,
                    true,
                ),
            );
        } else {
            result.push(
                normalizeServiceDirective(
                    undefined,
                    context,
                    `services.${serviceName}`,
                    serviceName,
                    true,
                ),
            );
        }
    }

    return result;
}

function normalizeConfigScopes(
    raw: unknown,
    context: NormalizationContext,
): HostConfigScopes | undefined {
    if (raw === undefined || raw === null) return undefined;
    if (!isRecord(raw)) {
        throw new HostConfigFormatError(
            context.path,
            `Expected 'config' to be a table, received ${describeType(raw)}.`,
        );
    }

    const scopes: HostConfigScopes = {};
    for (const [scopeName, scopeValue] of Object.entries(raw)) {
        if (scopeValue === undefined || scopeValue === null) {
            scopes[scopeName] = {};
            continue;
        }
        if (!isRecord(scopeValue)) {
            throw new HostConfigFormatError(
                context.path,
                `Expected config.${scopeName} to be a table, received ${describeType(scopeValue)
                }`,
            );
        }
        const entries: Record<string, HostConfigEntry> = {};
        for (const [entryName, entryValue] of Object.entries(scopeValue)) {
            if (entryValue === undefined || entryValue === null) {
                entries[entryName] = {};
                continue;
            }
            if (!isRecord(entryValue)) {
                throw new HostConfigFormatError(
                    context.path,
                    `Expected config.${scopeName}.${entryName} to be a table, received ${describeType(entryValue)
                    }`,
                );
            }
            entries[entryName] = { ...entryValue };
        }
        scopes[scopeName] = entries;
    }

    return scopes;
}

function mergeDirectiveTables(
    legacy: unknown,
    scoped: Record<string, HostConfigEntry> | undefined,
    context: NormalizationContext,
    legacyLocation: string,
): Record<string, unknown> | undefined {
    let result: Record<string, unknown> | undefined;

    if (legacy !== undefined && legacy !== null) {
        if (!isRecord(legacy)) {
            throw new HostConfigFormatError(
                context.path,
                `Expected ${legacyLocation} to be a table, received ${describeType(legacy)
                }`,
            );
        }
        result = { ...legacy };
    }

    if (scoped && Object.keys(scoped).length) {
        if (!result) {
            result = {};
        }
        for (const [key, value] of Object.entries(scoped)) {
            result[key] = value;
        }
    }

    return result;
}

function normalizeStringArray(
    raw: unknown,
    context: NormalizationContext,
    location: string,
): string[] | undefined {
    if (raw === undefined) return undefined;
    if (raw === null) return [];
    if (!Array.isArray(raw)) {
        throw new HostConfigFormatError(
            context.path,
            `Expected ${location} to be an array, received ${describeType(raw)}.`,
        );
    }
    const results: string[] = [];
    for (const [index, value] of raw.entries()) {
        const name = coerceName(value);
        if (!name) {
            throw new HostConfigFormatError(
                context.path,
                `Expected ${location}[${index}] to be a non-empty string, received ${describeType(value)
                }`,
            );
        }
        if (!results.includes(name)) {
            results.push(name);
        }
    }
    return results;
}

function normalizeHostSection(
    hostname: string,
    raw: unknown,
    context: NormalizationContext,
): HostConfig["host"] {
    if (raw === undefined || raw === null) {
        raw = {};
    }
    if (!isRecord(raw)) {
        throw new HostConfigFormatError(
            context.path,
            `Expected 'host' to be a table, received ${describeType(raw)}.`,
        );
    }

    const name = coerceName(raw.name) ?? hostname;
    if (!name) {
        throw new HostConfigFormatError(
            context.path,
            `Host entry is missing a name and no fallback was provided.`,
        );
    }

    const roles = normalizeStringArray(raw.roles, context, "host.roles");
    const installers = normalizeStringArray(
        raw.installers,
        context,
        "host.installers",
    );
    const modules = normalizeStringArray(raw.modules, context, "host.modules");
    const services = normalizeStringArray(
        raw.services,
        context,
        "host.services",
    );

    const host: HostConfig["host"] = { ...raw, name };
    if (roles !== undefined) host.roles = roles;
    if (installers !== undefined) host.installers = installers;
    if (modules !== undefined) host.modules = modules;
    if (services !== undefined) host.services = services;

    return host;
}

function normalizeHostConfig(
    hostname: string,
    path: string,
    raw: RawHostConfig,
): HostConfig {
    const {
        host: rawHost,
        modules: rawModules,
        services: rawServices,
        config: rawConfig,
        ...rest
    } = raw;
    const context = { path };
    const host = normalizeHostSection(hostname, rawHost, context);
    const configScopes = normalizeConfigScopes(rawConfig, context);
    const moduleTable = mergeDirectiveTables(
        rawModules,
        configScopes?.mod,
        context,
        "'modules'",
    );
    const serviceTable = mergeDirectiveTables(
        rawServices,
        configScopes?.srv,
        context,
        "'services'",
    );
    const modules = normalizeModuleDirectives(host.modules, moduleTable, context);
    const services = normalizeServiceDirectives(
        host.services,
        serviceTable,
        context,
    );
    const config: HostConfig = {
        ...rest,
        host,
        modules,
        services,
    } as HostConfig;
    if (configScopes && Object.keys(configScopes).length) {
        config.config = configScopes;
    }
    return config;
}

export function pathExists(path: string): boolean {
    try {
        Deno.statSync(path);
        return true;
    } catch (error) {
        if (error instanceof Deno.errors.NotFound) return false;
        throw error;
    }
}

function findConfigPath(hostname: string): string | undefined {
    const root = hostsRoot();
    for (const extension of HOST_CONFIG_EXTENSIONS) {
        const candidate = join(root, `${hostname}${extension}`);
        if (pathExists(candidate)) {
            return candidate;
        }
    }
    return undefined;
}

function parseHostConfig(path: string): RawHostConfig {
    const text = Deno.readTextFileSync(path);
    const extension = extname(path).toLowerCase() as HostConfigExtension;
    switch (extension) {
        case ".json":
        case ".jsonc": {
            const parsed = parseJsonc(text);
            if (!isRecord(parsed)) {
                throw new HostConfigFormatError(
                    path,
                    `Expected JSON host config to produce an object but received ${describeType(parsed)
                    }`,
                );
            }
            return parsed as RawHostConfig;
        }
        case ".yaml":
        case ".yml": {
            const parsed = parseYaml(text);
            if (!isRecord(parsed)) {
                throw new HostConfigFormatError(
                    path,
                    `Expected YAML host config to produce an object but received ${describeType(parsed)
                    }`,
                );
            }
            return parsed as RawHostConfig;
        }
        case ".toml":
        default: {
            return parseToml(text) as unknown as RawHostConfig;
        }
    }
}

export function locateHostConfig(hostname: string): string {
    const path = findConfigPath(hostname);
    if (!path) {
        throw new HostConfigNotFoundError(hostname);
    }
    return path;
}

export function readHostConfig(hostname: string): HostConfig {
    return loadHostConfig(hostname).config;
}

export function loadHostConfig(
    hostname: string,
): { path: string; config: HostConfig } {
    const path = locateHostConfig(hostname);
    const raw = parseHostConfig(path);
    const config = normalizeHostConfig(hostname, path, raw);
    return { path, config };
}

export function availableHosts(): string[] {
    const names = new Set<string>();
    for (const entry of Deno.readDirSync(hostsRoot())) {
        if (!entry.isFile) continue;
        for (const extension of HOST_CONFIG_EXTENSIONS) {
            if (entry.name.endsWith(extension)) {
                names.add(entry.name.slice(0, -extension.length));
                break;
            }
        }
    }
    return Array.from(names).sort();
}

export const __internals__ = {
    HOST_CONFIG_EXTENSIONS,
    normalizeHostConfig,
    normalizeModuleDirectives,
    normalizeServiceDirectives,
    normalizeConfigScopes,
};
