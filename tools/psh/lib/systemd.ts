import { join } from "$std/path/mod.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import { $ } from "$dax";
import {
  locateModuleDir,
  moduleConfig,
  resolveModuleScript,
} from "./module.ts";
import { repoRoot } from "./paths.ts";
import { buildRosEnv } from "./ros_env.ts";
import {
  HostConfig,
  HostConfigNotFoundError,
  loadHostConfig,
  ModuleDirective,
  ServiceDirective,
} from "./host.ts";
import { resolveServiceContext } from "./service.ts";

function homeDir(): string {
  const home = Deno.env.get("HOME");
  if (!home) throw new Error("HOME environment variable is not set");
  return home;
}

function systemdUserDir(): string {
  const base = Deno.env.get("XDG_CONFIG_HOME") ?? join(homeDir(), ".config");
  const dir = join(base, "systemd", "user");
  Deno.mkdirSync(dir, { recursive: true });
  return dir;
}

function moduleUnitName(module: string): string {
  return `psh-module-${module}.service`;
}

function serviceUnitName(service: string): string {
  return `psh-service-${service}.service`;
}

type UnitKind = "module" | "service";

function unitName(kind: UnitKind, name: string): string {
  return kind === "module" ? moduleUnitName(name) : serviceUnitName(name);
}

function unitPath(kind: UnitKind, name: string): string {
  return join(systemdUserDir(), unitName(kind, name));
}

function environmentLines(
  env?: Record<string, string>,
): string[] {
  if (!env) return [];
  const entries = Object.entries(env);
  if (!entries.length) return [];
  entries.sort(([a], [b]) => a.localeCompare(b));
  return entries.map(([key, value]) => `Environment=${key}=${value}`);
}

function mergeEnv(
  ...sources: Array<Record<string, string> | undefined>
): Record<string, string> {
  const result: Record<string, string> = {};
  for (const source of sources) {
    if (!source) continue;
    for (const [key, value] of Object.entries(source)) {
      if (!key) continue;
      result[key] = value;
    }
  }
  return result;
}

function sanitizeEnvRecord(
  input?: Record<string, unknown>,
): Record<string, string> {
  if (!input) return {};
  const result: Record<string, string> = {};
  for (const [key, value] of Object.entries(input)) {
    if (value === undefined || value === null) continue;
    if (!key) continue;
    result[key] = typeof value === "string" ? value : String(value);
  }
  return result;
}

let cachedHostConfig: HostConfig | null | undefined;

function hostCandidates(): string[] {
  const candidates: string[] = [];
  const override = Deno.env.get("PSH_HOST")?.trim();
  if (override) candidates.push(override);
  try {
    const hostname = Deno.hostname();
    if (hostname?.trim()) {
      candidates.push(hostname.trim());
    }
  } catch (_error) {
    // ignore hostname detection failures
  }
  const seen = new Set<string>();
  const ordered: string[] = [];
  for (const candidate of candidates) {
    if (!candidate) continue;
    if (seen.has(candidate)) continue;
    seen.add(candidate);
    ordered.push(candidate);
  }
  return ordered;
}

function detectHostConfig(): HostConfig | null {
  if (cachedHostConfig !== undefined) {
    return cachedHostConfig;
  }

  for (const candidate of hostCandidates()) {
    try {
      const { config } = loadHostConfig(candidate);
      cachedHostConfig = config;
      return config;
    } catch (error) {
      if (error instanceof HostConfigNotFoundError) continue;
      throw error;
    }
  }

  cachedHostConfig = null;
  return null;
}

function moduleDirective(module: string): ModuleDirective | undefined {
  const host = detectHostConfig();
  return host?.modules?.find((entry) => entry.name === module);
}

function serviceDirective(service: string): ServiceDirective | undefined {
  const host = detectHostConfig();
  return host?.services?.find((entry) => entry.name === service);
}

function moduleEnv(module: string): Record<string, string> {
  const directiveEnv = sanitizeEnvRecord(moduleDirective(module)?.env);
  return mergeEnv(
    { PSYCHED_REPO_ROOT: repoRoot() },
    buildRosEnv(),
    directiveEnv,
  );
}

function serviceEnv(
  service: string,
  baseEnv: Record<string, string>,
): Record<string, string> {
  const directiveEnv = sanitizeEnvRecord(serviceDirective(service)?.env);
  return mergeEnv(
    { PSYCHED_REPO_ROOT: repoRoot() },
    baseEnv,
    directiveEnv,
  );
}

async function isLingerEnabled(): Promise<boolean> {
  try {
    const result = await $`loginctl show-user ${Deno.env.get("USER") || ""}`
      .stdout("piped")
      .stderr("piped")
      .noThrow();
    
    if (result.code !== 0) {
      return false;
    }
    
    const output = result.stdout;
    for (const line of output.split("\n")) {
      if (line.startsWith("Linger=")) {
        return line.split("=")[1]?.trim() === "yes";
      }
    }
    return false;
  } catch (_error) {
    return false;
  }
}

async function enableLinger(): Promise<void> {
  const user = Deno.env.get("USER");
  if (!user) {
    console.warn(
      colors.yellow(
        "⚠️  USER environment variable not set, skipping linger setup",
      ),
    );
    return;
  }

  const isEnabled = await isLingerEnabled();
  if (isEnabled) {
    return;
  }

  console.log(
    colors.cyan(
      `Enabling user lingering for ${user} (required for services to start at boot)...`,
    ),
  );
  
  try {
    await $`loginctl enable-linger ${user}`.stdout("inherit").stderr("inherit");
    console.log(colors.green(`✓ Lingering enabled for ${user}`));
  } catch (error) {
    console.warn(
      colors.yellow(
        `⚠️  Failed to enable lingering: ${error}. Services may not start at boot.`,
      ),
    );
    console.warn(
      colors.yellow(
        `   You may need to run: sudo loginctl enable-linger ${user}`,
      ),
    );
  }
}

async function reloadSystemd(): Promise<void> {
  await enableLinger();
  await $`systemctl --user daemon-reload`.stdout("inherit").stderr("inherit");
}

export async function setupSystemd(module: string): Promise<void> {
  const config = moduleConfig(module);
  if (!config.launch) {
    throw new Error(`module '${module}' does not declare a launch script`);
  }
  const moduleDir = locateModuleDir(module);
  const launch = await resolveModuleScript(config.launch, moduleDir);
  const shutdown = config.shutdown
    ? await resolveModuleScript(config.shutdown, moduleDir)
    : undefined;
  const env = moduleEnv(module);
  const lines: string[] = [
    "# Autogenerated by psh systemd setup",
    "[Unit]",
    `Description=Psyched module ${module}`,
    "After=network.target",
    "",
    "[Service]",
    "Type=simple",
    `WorkingDirectory=${moduleDir}`,
    `ExecStart=/usr/bin/env bash ${launch}`,
  ];
  if (shutdown) {
    lines.push(`ExecStop=/usr/bin/env bash ${shutdown}`);
  }
  lines.push(...environmentLines(env));
  lines.push("Restart=on-failure");
  lines.push("");
  lines.push("[Install]");
  lines.push("WantedBy=default.target");
  lines.push("");
  const path = unitPath("module", module);
  Deno.writeTextFileSync(path, lines.join("\n"));
  console.log(colors.green(`Wrote ${path}`));
  await reloadSystemd();
}

export async function setupServiceSystemd(service: string): Promise<void> {
  const context = resolveServiceContext(service);
  const env = serviceEnv(service, context.env);
  const composeArgs = `-f ${context.composePath} -p ${context.project}`;
  const lines: string[] = [
    "# Autogenerated by psh systemd setup",
    "[Unit]",
    `Description=Psyched service ${service}`,
    "After=network-online.target",
    "Wants=network-online.target",
    "",
    "[Service]",
    "Type=oneshot",
    "RemainAfterExit=yes",
    `WorkingDirectory=${context.dir}`,
    `ExecStart=/usr/bin/env docker compose ${composeArgs} up -d`,
    `ExecStop=/usr/bin/env docker compose ${composeArgs} down`,
  ];
  lines.push(...environmentLines(env));
  lines.push("");
  lines.push("[Install]");
  lines.push("WantedBy=default.target");
  lines.push("");
  const path = unitPath("service", service);
  Deno.writeTextFileSync(path, lines.join("\n"));
  console.log(colors.green(`Wrote ${path}`));
  await reloadSystemd();
}

export async function teardownSystemd(module: string): Promise<void> {
  const path = unitPath("module", module);
  try {
    Deno.removeSync(path);
    console.log(colors.yellow(`Removed ${path}`));
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      console.warn(colors.yellow(`Unit file ${path} not found`));
    } else {
      throw error;
    }
  }
  await reloadSystemd();
}

export async function teardownServiceSystemd(service: string): Promise<void> {
  const path = unitPath("service", service);
  try {
    Deno.removeSync(path);
    console.log(colors.yellow(`Removed ${path}`));
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      console.warn(colors.yellow(`Unit file ${path} not found`));
    } else {
      throw error;
    }
  }
  await reloadSystemd();
}

async function systemctl(
  action: string,
  kind: UnitKind,
  name: string,
): Promise<void> {
  const unit = unitName(kind, name);
  await $`systemctl --user ${action} ${unit}`.stdout("inherit").stderr(
    "inherit",
  );
}

export async function enableSystemd(module: string): Promise<void> {
  await systemctl("enable", "module", module);
}

export async function disableSystemd(module: string): Promise<void> {
  await systemctl("disable", "module", module);
}

export async function startSystemd(module: string): Promise<void> {
  await systemctl("start", "module", module);
}

export async function stopSystemd(module: string): Promise<void> {
  await systemctl("stop", "module", module);
}

export async function enableServiceSystemd(service: string): Promise<void> {
  await systemctl("enable", "service", service);
}

export async function disableServiceSystemd(service: string): Promise<void> {
  await systemctl("disable", "service", service);
}

export async function startServiceSystemd(service: string): Promise<void> {
  await systemctl("start", "service", service);
}

export async function stopServiceSystemd(service: string): Promise<void> {
  await systemctl("stop", "service", service);
}

export const __test__ = {
  environmentLines,
  mergeEnv,
  sanitizeEnvRecord,
  moduleUnitName,
  serviceUnitName,
  isLingerEnabled,
  enableLinger,
};
