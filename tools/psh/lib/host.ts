import { join, resolve } from "$std/path/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import { $ } from "$dax";
import { hostsRoot, repoRoot } from "./paths.ts";
import { bringModuleUp, setupModule } from "./module.ts";
import { bringServiceUp, setupService } from "./service.ts";
import { getInstaller, runInstaller } from "./deps/installers.ts";

export interface HostConfig {
  host: { name: string };
  provision?: { scripts?: string[]; installers?: string[] };
  modules?: ModuleDirective[];
  services?: ServiceDirective[];
}

export interface ModuleDirective {
  name: string;
  setup?: boolean;
  launch?: boolean;
  env?: Record<string, string>;
}

export interface ServiceDirective {
  name: string;
  setup?: boolean;
  up?: boolean;
  env?: Record<string, string>;
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

export function locateHostConfig(hostname: string): string {
  const candidate = join(hostsRoot(), `${hostname}.toml`);
  if (!pathExists(candidate)) {
    throw new Error(`no host config found for ${hostname}`);
  }
  return candidate;
}

export function readHostConfig(hostname: string): HostConfig {
  const path = locateHostConfig(hostname);
  return parseToml(Deno.readTextFileSync(path)) as unknown as HostConfig;
}

export function availableHosts(): string[] {
  const names: string[] = [];
  for (const entry of Deno.readDirSync(hostsRoot())) {
    if (entry.isFile && entry.name.endsWith(".toml")) {
      names.push(entry.name.replace(/\.toml$/, ""));
    }
  }
  names.sort();
  return names;
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

async function runProvisionScripts(
  configPath: string,
  scripts: string[] = [],
): Promise<string[]> {
  const failures: string[] = [];
  for (const script of scripts) {
    const resolved = resolveScriptPath(script, configPath);
    if (!pathExists(resolved)) {
      failures.push(`${script} (missing)`);
      console.error(colors.red(`Script not found: ${resolved}`));
      continue;
    }
    console.log(colors.cyan(`Running ${resolved} …`));
    const result = await $`bash ${resolved}`.stdout("inherit").stderr("inherit")
      .noThrow();
    if (result.code !== 0) {
      failures.push(`${script} (exit ${result.code})`);
    }
  }
  return failures;
}

async function runProvisionInstallers(
  installers: string[] = [],
  options: ProvisionRunOptions,
): Promise<string[]> {
  if (!installers.length) return [];
  const failures: string[] = [];
  for (const id of installers) {
    const installer = getInstaller(id);
    if (!installer) {
      failures.push(`${id} (unknown installer)`);
      console.error(colors.red(`Unknown installer '${id}'.`));
      continue;
    }
    console.log(colors.bold(colors.magenta(`\nInstaller: ${installer.label}`)));
    try {
      await runInstaller(id, {
        verbose: options.verbose,
        showLogsOnSuccess: options.showLogsOnSuccess,
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      console.error(colors.red(`Installer '${id}' failed: ${message}`));
      failures.push(`${id} (installer failed)`);
    }
  }
  return failures;
}

async function processModules(
  host: string,
  directives: ModuleDirective[] = [],
): Promise<string[]> {
  if (!directives.length) {
    console.log(colors.yellow(`No modules configured for host '${host}'.`));
    return [];
  }
  const failures: string[] = [];
  for (const directive of directives) {
    const { name, env = {}, setup = true, launch = false } = directive;
    console.log(colors.bold(`Processing module '${name}'`));
    const restore = applyEnv(env);
    let setupOk = true;
    try {
      if (setup) {
        const result = await $`true`.noThrow(); // placeholder for BDD (ensures async context)
        void result;
        try {
          await setupModule(name);
          console.log(colors.green(`[${name}] setup complete.`));
        } catch (error) {
          console.error(colors.red(`[${name}] setup failed: ${error}`));
          failures.push(`${name} (setup: ${error})`);
          setupOk = false;
        }
      }
      if (launch && (setupOk || !setup)) {
        try {
          await bringModuleUp(name);
        } catch (error) {
          console.error(colors.red(`[${name}] launch failed: ${error}`));
          failures.push(`${name} (launch: ${error})`);
        }
      } else if (launch && !setupOk) {
        console.warn(
          colors.yellow(`[${name}] skipping launch because setup failed.`),
        );
      }
    } finally {
      restore();
    }
  }
  return failures;
}

async function processServices(
  host: string,
  directives: ServiceDirective[] = [],
): Promise<string[]> {
  if (!directives.length) {
    console.log(colors.yellow(`No services configured for host '${host}'.`));
    return [];
  }
  const failures: string[] = [];
  for (const directive of directives) {
    const { name, env = {}, setup = true, up = false } = directive;
    console.log(colors.bold(`Processing service '${name}'`));
    const restore = applyEnv(env);
    let setupOk = true;
    try {
      if (setup) {
        try {
          await setupService(name);
          console.log(colors.green(`[${name}] setup complete.`));
        } catch (error) {
          console.error(colors.red(`[${name}] setup failed: ${error}`));
          failures.push(`${name} (setup: ${error})`);
          setupOk = false;
        }
      }
      if (up && (setupOk || !setup)) {
        try {
          await bringServiceUp(name);
        } catch (error) {
          console.error(colors.red(`[${name}] start failed: ${error}`));
          failures.push(`${name} (up: ${error})`);
        }
      } else if (up && !setupOk) {
        console.warn(
          colors.yellow(`[${name}] skipping start because setup failed.`),
        );
      }
    } finally {
      restore();
    }
  }
  return failures;
}

export interface ProvisionHostOptions {
  verbose?: boolean;
  showLogsOnSuccess?: boolean;
}

export async function provisionHost(
  hostname?: string,
  options: ProvisionHostOptions = {},
): Promise<void> {
  const name = hostname ?? Deno.hostname();
  console.log(colors.bold(`Detected hostname: ${name}`));
  const configPath = locateHostConfig(name);
  console.log(colors.cyan(`Loading host config: ${configPath}`));
  const cfg = parseToml(
    Deno.readTextFileSync(configPath),
  ) as unknown as HostConfig;
  const scripts = cfg.provision?.scripts ?? [];
  const installers = cfg.provision?.installers ?? [];
  const envVerbose = Deno.env.get("PSH_VERBOSE") === "1";
  const verbose = options.verbose ?? envVerbose;
  const showLogsOnSuccess = options.showLogsOnSuccess ?? verbose;
  const provisionOptions: ProvisionRunOptions = { verbose, showLogsOnSuccess };
  const failures: string[] = [];
  failures.push(...await runProvisionInstallers(installers, provisionOptions));
  failures.push(...await runProvisionScripts(configPath, scripts));
  failures.push(...await processModules(name, cfg.modules));
  failures.push(...await processServices(name, cfg.services));
  if (failures.length) {
    console.warn(colors.yellow(`Host provisioning finished with issues:`));
    for (const failure of failures) {
      console.warn(colors.yellow(`  - ${failure}`));
    }
  } else {
    console.log(colors.green(`Host provisioning complete for '${name}'.`));
  }
}
