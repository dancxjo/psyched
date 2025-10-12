import { extname, join, resolve } from "$std/path/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { DepGraph } from "$deps/dependency-graph";
import { colors } from "$cliffy/ansi/colors.ts";
import { $ } from "$dax";
import { hostsRoot, repoRoot } from "./paths.ts";
import { bringModuleUp, setupModule } from "./module.ts";
import { bringServiceUp, setupService } from "./service.ts";
import { getInstaller, runInstaller } from "./deps/installers.ts";
import { buildRosEnv } from "./ros_env.ts";

/** Supported host manifest file extension. */
const HOST_CONFIG_EXTENSIONS = [
  ".toml",
] as const;

type HostConfigExtension = typeof HOST_CONFIG_EXTENSIONS[number];

export type HostConfigEntry = Record<string, unknown>;
export type HostConfigScopes = Record<string, Record<string, HostConfigEntry>>;

const moduleOps = {
  setup: setupModule,
  launch: bringModuleUp,
};

const serviceOps = {
  setup: setupService,
  start: bringServiceUp,
};

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
        `Expected ${location}.enabled to be a boolean, received ${
          describeType(enabledRaw)
        }.`,
      );
    }

    if (config.arguments !== undefined && !isRecord(config.arguments)) {
      throw new HostConfigFormatError(
        context.path,
        `Expected ${location}.arguments to be a table, received ${
          describeType(config.arguments)
        }.`,
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
    `Expected ${location} to be a boolean or table, received ${
      describeType(raw)
    }.`,
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

  return { ...raw, name } as ServiceDirective;
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
        ),
      );
    } else {
      result.push(
        normalizeServiceDirective(
          undefined,
          context,
          `services.${serviceName}`,
          serviceName,
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
        `Expected config.${scopeName} to be a table, received ${
          describeType(scopeValue)
        }.`,
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
          `Expected config.${scopeName}.${entryName} to be a table, received ${
            describeType(entryValue)
          }.`,
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
        `Expected ${legacyLocation} to be a table, received ${
          describeType(legacy)
        }.`,
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
        `Expected ${location}[${index}] to be a non-empty string, received ${
          describeType(value)
        }.`,
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

function pathExists(path: string): boolean {
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
          `Expected JSON host config to produce an object but received ${
            describeType(parsed)
          }.`,
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
          `Expected YAML host config to produce an object but received ${
            describeType(parsed)
          }.`,
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

function applyEnv(overrides: Record<string, string> = {}): () => void {
  const previous = new Map<string, string | undefined>();
  for (const [key, value] of Object.entries(overrides)) {
    previous.set(key, Deno.env.get(key));
    Deno.env.set(key, value);
  }
  return () => {
    for (const [key, value] of previous.entries()) {
      if (value === undefined) {
        Deno.env.delete(key);
      } else {
        Deno.env.set(key, value);
      }
    }
  };
}

function resolveScriptPath(script: string, configPath: string): string {
  if (script.startsWith("/")) return script;
  const configDir = resolve(join(configPath, ".."));
  const candidates = [
    script,
    join(configDir, script),
    join(repoRoot(), script),
  ];
  for (const candidate of candidates) {
    const resolved = resolve(candidate);
    if (pathExists(resolved)) return resolved;
  }
  return resolve(join(configDir, script));
}

interface ProvisionRunOptions {
  verbose: boolean;
  showLogsOnSuccess: boolean;
}

export interface ProvisionTask {
  id: string;
  label: string;
  dependencies: string[];
  run: () => Promise<void>;
}

export interface TaskRegistry {
  tasks: ProvisionTask[];
  aliasToId: Map<string, string>;
}

function createTaskRegistry(): TaskRegistry {
  return {
    tasks: [],
    aliasToId: new Map<string, string>(),
  };
}

function registerTask(
  registry: TaskRegistry,
  task: ProvisionTask,
  aliases: string[] = [],
): void {
  if (!registry.aliasToId.has(task.id)) {
    registry.aliasToId.set(task.id, task.id);
  }
  for (const alias of aliases) {
    const existing = registry.aliasToId.get(alias);
    if (existing && existing !== task.id) {
      throw new Error(
        `Alias '${alias}' already registered for task '${existing}'`,
      );
    }
    registry.aliasToId.set(alias, task.id);
  }
  registry.tasks.push(task);
}

function resolveDependencyAliases(
  registry: TaskRegistry,
  task: ProvisionTask,
): void {
  const resolved: string[] = [];
  for (const dependency of task.dependencies) {
    const target = registry.aliasToId.get(dependency) ??
      registry.aliasToId.get(dependency.trim());
    if (!target) {
      throw new Error(
        `Unknown dependency '${dependency}' referenced by task '${task.id}'.`,
      );
    }
    if (target !== task.id && !resolved.includes(target)) {
      resolved.push(target);
    }
  }
  task.dependencies = resolved;
}

export const __test__ = {
  createTaskRegistry,
  registerTask,
  resolveDependencyAliases,
};

export const __internals__ = {
  moduleOps,
  serviceOps,
};

function mergeDependencies(
  defaults: string[],
  extras: string[] | undefined,
): string[] {
  const unique = new Set<string>();
  for (const value of defaults) {
    if (value) unique.add(value);
  }
  if (extras) {
    for (const value of extras) {
      if (value) unique.add(value);
    }
  }
  return Array.from(unique);
}

async function executeProvisionScript(
  configPath: string,
  script: string,
): Promise<void> {
  const resolved = resolveScriptPath(script, configPath);
  if (!pathExists(resolved)) {
    throw new Error(`script not found at ${resolved}`);
  }
  console.log(colors.cyan(`Running ${resolved} …`));
  const result = await $`bash ${resolved}`.stdout("inherit").stderr("inherit")
    .noThrow();
  if (result.code !== 0) {
    throw new Error(`exit ${result.code}`);
  }
}

/** Options that adjust host provisioning behaviour. */
export interface ProvisionHostOptions {
  /** Enable verbose logging for installers and scripts. */
  verbose?: boolean;
  /** Show installer logs even on success when true. */
  showLogsOnSuccess?: boolean;
  /** Include module setup/launch tasks when true. */
  includeModules?: boolean;
  /** Include service setup/start tasks when true. */
  includeServices?: boolean;
}

export async function provisionHost(
  hostname?: string,
  options: ProvisionHostOptions = {},
): Promise<void> {
  const detected = hostname ?? Deno.hostname();
  const { path, config } = loadHostConfig(detected);
  await provisionHostProfile({
    detectedHostname: detected,
    profileName: detected,
    configPath: path,
    config,
    options,
  });
}

export interface ProvisionHostProfileParams {
  detectedHostname: string;
  profileName: string;
  configPath: string;
  config: HostConfig;
  options?: ProvisionHostOptions;
}

export async function provisionHostProfile(
  params: ProvisionHostProfileParams,
): Promise<void> {
  const {
    detectedHostname,
    profileName,
    configPath,
    config,
    options = {},
  } = params;
  console.log(colors.bold(`Detected hostname: ${detectedHostname}`));
  if (profileName !== detectedHostname) {
    console.log(
      colors.bold(
        colors.magenta(`Applying host profile '${profileName}'.`),
      ),
    );
  }
  console.log(colors.cyan(`Loading host config: ${configPath}`));
  const cfg = config;
  const scripts = cfg.provision?.scripts ?? [];
  const hostInstallers = cfg.host.installers ?? [];
  const provisionInstallers = cfg.provision?.installers ?? [];
  const installers = Array.from(
    new Set<string>([...hostInstallers, ...provisionInstallers]),
  );
  const modules = cfg.modules ?? [];
  const services = cfg.services ?? [];
  const sharedEnv = buildRosEnv();
  const envVerbose = Deno.env.get("PSH_VERBOSE") === "1";
  const verbose = options.verbose ?? envVerbose;
  const showLogsOnSuccess = options.showLogsOnSuccess ?? verbose;
  const includeModules = options.includeModules ?? false;
  const includeServices = options.includeServices ?? false;
  const provisionOptions: ProvisionRunOptions = { verbose, showLogsOnSuccess };
  const registry = createTaskRegistry();

  const installerAliasDefaults = installers.map((id) => id);

  for (const id of installers) {
    const taskId = `installer:${id}`;
    registerTask(registry, {
      id: taskId,
      label: `Installer '${id}'`,
      dependencies: [],
      run: async () => {
        const installer = getInstaller(id);
        if (!installer) {
          throw new Error(`Unknown installer '${id}'.`);
        }
        console.log(
          colors.bold(colors.magenta(`\nInstaller: ${installer.label}`)),
        );
        await runInstaller(id, {
          verbose: provisionOptions.verbose,
          showLogsOnSuccess: provisionOptions.showLogsOnSuccess,
        });
      },
    }, [id]);
  }

  for (const script of scripts) {
    const taskId = `script:${script}`;
    registerTask(registry, {
      id: taskId,
      label: `Provision script '${script}'`,
      dependencies: mergeDependencies(installerAliasDefaults, undefined),
      run: async () => {
        await executeProvisionScript(configPath, script);
      },
    }, [script]);
  }

  if (!modules.length) {
    console.log(
      colors.yellow(`No modules configured for host '${profileName}'.`),
    );
  } else if (!includeModules) {
    console.log(
      colors.yellow(
        "Module provisioning skipped during host bootstrap. " +
          "Open a new shell session before running 'psh mod setup' to configure modules.",
      ),
    );
  } else {
    for (const directive of modules) {
      const {
        name: moduleName,
        env: moduleEnvOverrides = {},
        setup = true,
        launch = false,
      } = directive;
      const moduleDefaults = installers.includes("ros2") ? ["ros2"] : [];
      const moduleDependencies = mergeDependencies(
        moduleDefaults,
        directive.depends_on,
      );
      const moduleEnv = { ...sharedEnv, ...moduleEnvOverrides };

      let setupTaskId: string | undefined;
      const moduleAliases = [`module:${moduleName}`, moduleName];
      if (setup) {
        setupTaskId = `module:${moduleName}:setup`;
        registerTask(registry, {
          id: setupTaskId,
          label: `Module '${moduleName}' setup`,
          dependencies: [...moduleDependencies],
          run: async () => {
            const restore = applyEnv(moduleEnv);
            try {
              await moduleOps.setup(moduleName);
              console.log(colors.green(`[${moduleName}] setup complete.`));
            } finally {
              restore();
            }
          },
        }, launch ? [] : moduleAliases);
      }

      if (launch) {
        const launchTaskId = `module:${moduleName}:launch`;
        const launchDeps = setupTaskId
          ? mergeDependencies(moduleDependencies, [setupTaskId])
          : [...moduleDependencies];
        registerTask(registry, {
          id: launchTaskId,
          label: `Module '${moduleName}' launch`,
          dependencies: launchDeps,
          run: async () => {
            const restore = applyEnv(moduleEnv);
            try {
              await moduleOps.launch(moduleName);
            } finally {
              restore();
            }
          },
        }, moduleAliases);
      }
    }
  }

  if (!services.length) {
    console.log(
      colors.yellow(`No services configured for host '${profileName}'.`),
    );
  } else if (!includeServices) {
    console.log(
      colors.yellow(
        "Service provisioning skipped during host bootstrap. " +
          "Run 'psh svc setup' after refreshing your shell if services require setup.",
      ),
    );
  } else {
    for (const directive of services) {
      const {
        name: serviceName,
        env: serviceEnvOverrides = {},
        setup = true,
        up = false,
      } = directive;
      const serviceDefaults = installers.includes("docker") ? ["docker"] : [];
      const serviceDependencies = mergeDependencies(
        serviceDefaults,
        directive.depends_on,
      );
      const serviceEnv = { ...sharedEnv, ...serviceEnvOverrides };

      let setupTaskId: string | undefined;
      const serviceAliases = [`service:${serviceName}`, serviceName];
      if (setup) {
        setupTaskId = `service:${serviceName}:setup`;
        registerTask(registry, {
          id: setupTaskId,
          label: `Service '${serviceName}' setup`,
          dependencies: [...serviceDependencies],
          run: async () => {
            const restore = applyEnv(serviceEnv);
            try {
              await serviceOps.setup(serviceName);
              console.log(colors.green(`[${serviceName}] setup complete.`));
            } finally {
              restore();
            }
          },
        }, up ? [] : serviceAliases);
      }

      if (up) {
        const startTaskId = `service:${serviceName}:start`;
        const startDeps = setupTaskId
          ? mergeDependencies(serviceDependencies, [setupTaskId])
          : [...serviceDependencies];
        registerTask(registry, {
          id: startTaskId,
          label: `Service '${serviceName}' start`,
          dependencies: startDeps,
          run: async () => {
            const restore = applyEnv(serviceEnv);
            try {
              await serviceOps.start(serviceName);
            } finally {
              restore();
            }
          },
        }, serviceAliases);
      }
    }
  }

  for (const task of registry.tasks) {
    resolveDependencyAliases(registry, task);
  }

  const graph = new DepGraph<ProvisionTask>();
  const taskMap = new Map<string, ProvisionTask>();
  for (const task of registry.tasks) {
    if (!graph.hasNode(task.id)) {
      graph.addNode(task.id, task);
    }
    taskMap.set(task.id, task);
  }
  for (const task of registry.tasks) {
    for (const dependency of task.dependencies) {
      graph.addDependency(task.id, dependency);
    }
  }

  let orderedIds: string[];
  try {
    orderedIds = graph.overallOrder();
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    throw new Error(`Task dependency cycle detected: ${message}`);
  }

  const failedTasks = new Set<string>();
  const issues: string[] = [];

  for (const taskId of orderedIds) {
    const task = taskMap.get(taskId);
    if (!task) continue;
    const blockingDeps = task.dependencies.filter((dep) =>
      failedTasks.has(dep)
    );
    if (blockingDeps.length) {
      const blockedLabels = blockingDeps.map((dep) =>
        taskMap.get(dep)?.label ?? dep
      );
      console.warn(
        colors.yellow(
          `${task.label} skipped due to failed dependency (${
            blockedLabels.join(", ")
          }).`,
        ),
      );
      issues.push(
        `${task.label} (skipped: ${blockedLabels.join(", ")})`,
      );
      failedTasks.add(taskId);
      continue;
    }

    console.log(colors.bold(`\n→ ${task.label}`));
    try {
      await task.run();
      console.log(colors.green(`✓ ${task.label}`));
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      console.error(colors.red(`✗ ${task.label}: ${message}`));
      issues.push(`${task.label} (${message})`);
      failedTasks.add(taskId);
    }
  }

  if (issues.length) {
    console.warn(colors.yellow(`Host provisioning finished with issues:`));
    for (const issue of issues) {
      console.warn(colors.yellow(`  - ${issue}`));
    }
  } else {
    console.log(
      colors.green(`Host provisioning complete for '${profileName}'.`),
    );
  }
}
