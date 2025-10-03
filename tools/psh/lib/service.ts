import { join, resolve } from "$std/path/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import { $ } from "$dax";
import { repoRoot, servicesRoot } from "./paths.ts";

export interface ServiceConfig {
  compose: string;
  project?: string;
  setup_scripts?: string[];
  teardown_scripts?: string[];
  env?: Record<string, string>;
  description?: string;
}

interface ServiceManifest {
  service: Record<string, ServiceConfig>;
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

function locateServiceDir(service: string): string {
  const dir = join(servicesRoot(), service);
  if (!pathExists(dir)) {
    throw new Error(`service '${service}' not found under services/`);
  }
  return dir;
}

function loadServiceConfig(serviceDir: string, service: string): ServiceConfig {
  const manifestPath = join(serviceDir, "service.toml");
  if (!pathExists(manifestPath)) {
    throw new Error(
      `service '${service}' missing service.toml at ${manifestPath}`,
    );
  }
  const manifest = parseToml(
    Deno.readTextFileSync(manifestPath),
  ) as unknown as ServiceManifest;
  const config = manifest.service?.[service];
  if (!config) {
    throw new Error(
      `service '${service}' missing service.${service} entry in service.toml`,
    );
  }
  return config;
}

function resolveScriptPath(script: string, serviceDir: string): string {
  const absolute = script.startsWith("/");
  const candidates = absolute
    ? [script]
    : [script, join(serviceDir, script), join(repoRoot(), script)];
  for (const candidate of candidates) {
    const resolved = absolute ? candidate : resolve(candidate);
    if (pathExists(resolved)) {
      return resolved;
    }
  }
  return absolute ? script : resolve(join(serviceDir, script));
}

async function runScripts(
  service: string,
  serviceDir: string,
  scripts: string[] | undefined,
  phase: "setup" | "teardown",
): Promise<void> {
  if (!scripts?.length) return;
  for (const script of scripts) {
    const path = resolveScriptPath(script, serviceDir);
    if (!pathExists(path)) {
      const message = `[${service}] ${phase} script missing: ${path}`;
      if (phase === "setup") throw new Error(message);
      console.warn(colors.yellow(message));
      continue;
    }
    console.log(colors.cyan(`[${service}] running ${phase} script ${path}`));
    await $`bash ${path}`.cwd(serviceDir).stdout("inherit").stderr("inherit");
  }
}

function composeFilePath(serviceDir: string, config: ServiceConfig): string {
  const path = config.compose.startsWith("/")
    ? config.compose
    : resolve(join(serviceDir, config.compose));
  if (!pathExists(path)) {
    throw new Error(`compose file not found for ${serviceDir}: ${path}`);
  }
  return path;
}

function composeProject(service: string, config: ServiceConfig): string {
  return config.project ?? `psyched-${service}`;
}

export async function setupService(service: string): Promise<void> {
  const serviceDir = locateServiceDir(service);
  const config = loadServiceConfig(serviceDir, service);
  await runScripts(service, serviceDir, config.setup_scripts, "setup");
}

export async function setupServices(services: string[]): Promise<void> {
  for (const service of services) {
    await setupService(service);
  }
}

export async function teardownService(service: string): Promise<void> {
  const serviceDir = locateServiceDir(service);
  const config = loadServiceConfig(serviceDir, service);
  await bringServiceDown(service);
  await runScripts(service, serviceDir, config.teardown_scripts, "teardown");
}

export async function bringServiceUp(service: string): Promise<void> {
  const serviceDir = locateServiceDir(service);
  const config = loadServiceConfig(serviceDir, service);
  const composePath = composeFilePath(serviceDir, config);
  const project = composeProject(service, config);
  console.log(
    colors.green(`==> Starting service '${service}' using ${composePath}`),
  );
  let cmd = $`docker compose -f ${composePath} -p ${project} up -d`.cwd(
    serviceDir,
  );
  if (config.env) {
    cmd = cmd.env(config.env);
  }
  await cmd.stdout("inherit").stderr("inherit");
}

export async function bringServiceDown(service: string): Promise<void> {
  const serviceDir = locateServiceDir(service);
  const config = loadServiceConfig(serviceDir, service);
  const composePath = composeFilePath(serviceDir, config);
  const project = composeProject(service, config);
  console.log(
    colors.yellow(`==> Stopping service '${service}' using ${composePath}`),
  );
  let cmd = $`docker compose -f ${composePath} -p ${project} down`.cwd(
    serviceDir,
  );
  if (config.env) {
    cmd = cmd.env(config.env);
  }
  await cmd.stdout("inherit").stderr("inherit");
}

export async function teardownServices(services: string[]): Promise<void> {
  for (const service of services) {
    await teardownService(service);
  }
}

export interface ServiceStatus {
  name: string;
  status: "running" | "stopped" | "error";
  description?: string;
}

export function listServices(): string[] {
  const names: string[] = [];
  for (const entry of Deno.readDirSync(servicesRoot())) {
    if (entry.isDirectory) names.push(entry.name);
  }
  names.sort();
  return names;
}

export async function serviceStatuses(): Promise<ServiceStatus[]> {
  const statuses: ServiceStatus[] = [];
  for (const name of listServices()) {
    try {
      statuses.push(await serviceStatus(name));
    } catch (error) {
      statuses.push({ name, status: "error", description: String(error) });
    }
  }
  return statuses;
}

export async function serviceStatus(name: string): Promise<ServiceStatus> {
  const serviceDir = locateServiceDir(name);
  const config = loadServiceConfig(serviceDir, name);
  const composePath = composeFilePath(serviceDir, config);
  const project = composeProject(name, config);
  let cmd = $`docker compose -f ${composePath} -p ${project} ps`.cwd(
    serviceDir,
  );
  if (config.env) cmd = cmd.env(config.env);
  const output = await cmd.stderr("piped").text();
  const running = output.split("\n").slice(1).some((line) =>
    line.includes("Up")
  );
  return {
    name,
    status: running ? "running" : "stopped",
    description: config.description,
  };
}
