import { join, resolve } from "$std/path/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import { $, CommandBuilder } from "$dax";
import { repoRoot, servicesRoot } from "./paths.ts";
import { buildRosEnv } from "./ros_env.ts";

export interface ServiceConfig {
  compose: string;
  project?: string;
  setup_scripts?: string[];
  teardown_scripts?: string[];
  env?: Record<string, string>;
  description?: string;
  /** Compose service name to target when spawning shells (defaults to the service id). */
  shell_service?: string;
  /** Default command executed by `psh svc shell` when no override is provided. */
  shell_command?: string[];
  /** Default user passed to `docker compose exec -u`. */
  shell_user?: string;
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

/** Options for `psh svc shell` and programmatic shell execution. */
export interface ServiceShellOptions {
  command?: string[];
  service?: string;
  user?: string;
  interactive?: boolean;
  tty?: boolean;
}

/**
 * Construct the `docker compose exec` argument vector for a service shell.
 * Exposed via `__test__` for deterministic unit coverage.
 */
function buildShellArgs(
  service: string,
  config: ServiceConfig,
  composePath: string,
  project: string,
  options: ServiceShellOptions = {},
): string[] {
  const target = options.service ?? config.shell_service ?? service;
  const interactive = options.interactive ?? true;
  const tty = options.tty ?? true;
  const command = options.command ?? config.shell_command ?? ["bash"];
  const user = options.user ?? config.shell_user;

  const args = [
    "docker",
    "compose",
    "-f",
    composePath,
    "-p",
    project,
    "exec",
  ];
  if (interactive) args.push("-i");
  if (tty) args.push("-t");
  if (user) args.push("-u", user);
  args.push(target, ...command);
  return args;
}

export interface ResolvedServiceContext {
  name: string;
  dir: string;
  config: ServiceConfig;
  composePath: string;
  project: string;
  env: Record<string, string>;
}

export function resolveServiceContext(service: string): ResolvedServiceContext {
  const serviceDir = locateServiceDir(service);
  const config = loadServiceConfig(serviceDir, service);
  const composePath = composeFilePath(serviceDir, config);
  const project = composeProject(service, config);
  const env = { ...buildRosEnv(), ...(config.env ?? {}) };
  return {
    name: service,
    dir: serviceDir,
    config,
    composePath,
    project,
    env,
  };
}

export interface SetupServiceOptions {
  /** When true, invoke `docker compose build` after setup scripts finish. */
  build?: boolean;
}

async function buildComposeImages(
  context: ResolvedServiceContext,
): Promise<void> {
  console.log(
    colors.cyan(
      `==> Building Docker images for service '${context.name}'`,
    ),
  );
  let cmd =
    $`docker compose -f ${context.composePath} -p ${context.project} build`
      .cwd(context.dir);
  cmd = cmd.env(context.env);
  await cmd.stdout("inherit").stderr("inherit");
}

export async function setupService(
  service: string,
  options: SetupServiceOptions = {},
): Promise<void> {
  const context = resolveServiceContext(service);
  await runScripts(
    service,
    context.dir,
    context.config.setup_scripts,
    "setup",
  );
  if (options.build) {
    await buildComposeImages(context);
  }
}

export async function setupServices(
  services: string[],
  options: SetupServiceOptions = {},
): Promise<void> {
  for (const service of services) {
    await setupService(service, options);
  }
}

export async function teardownService(service: string): Promise<void> {
  const serviceDir = locateServiceDir(service);
  const config = loadServiceConfig(serviceDir, service);
  await bringServiceDown(service);
  await runScripts(service, serviceDir, config.teardown_scripts, "teardown");
}

export async function bringServiceUp(service: string): Promise<void> {
  const context = resolveServiceContext(service);
  console.log(
    colors.green(
      `==> Starting service '${service}' using ${context.composePath}`,
    ),
  );
  let cmd =
    $`docker compose -f ${context.composePath} -p ${context.project} up -d`
      .cwd(context.dir);
  cmd = cmd.env(context.env);
  await cmd.stdout("inherit").stderr("inherit");
}

export async function bringServiceDown(service: string): Promise<void> {
  const context = resolveServiceContext(service);
  console.log(
    colors.yellow(
      `==> Stopping service '${service}' using ${context.composePath}`,
    ),
  );
  let cmd =
    $`docker compose -f ${context.composePath} -p ${context.project} down`
      .cwd(context.dir);
  cmd = cmd.env(context.env);
  await cmd.stdout("inherit").stderr("inherit");
}

export async function teardownServices(services: string[]): Promise<void> {
  for (const service of services) {
    await teardownService(service);
  }
}

export async function openServiceShell(
  service: string,
  options: ServiceShellOptions = {},
): Promise<void> {
  const context = resolveServiceContext(service);
  const args = buildShellArgs(
    service,
    context.config,
    context.composePath,
    context.project,
    options,
  );

  let builder = new CommandBuilder().command(args)
    .cwd(context.dir)
    .stdin("inherit")
    .stdout("inherit")
    .stderr("inherit")
    .noThrow();
  builder = builder.env(context.env);

  const result = await builder.spawn();
  if (result.code !== 0) {
    throw new Error(
      `docker compose exec exited with code ${result.code}`,
    );
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
  const context = resolveServiceContext(name);
  let cmd = $`docker compose -f ${context.composePath} -p ${context.project} ps`
    .cwd(context.dir);
  cmd = cmd.env(context.env);
  const output = await cmd.stderr("piped").text();
  const running = output.split("\n").slice(1).some((line) =>
    line.includes("Up")
  );
  return {
    name,
    status: running ? "running" : "stopped",
    description: context.config.description,
  };
}

export const __test__ = {
  buildShellArgs,
};
