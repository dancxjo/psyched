import { join, resolve } from "$std/path/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { runPsh } from "./psh_cli.ts";
import { servicesRoot } from "./paths.ts";

export interface ServiceConfig {
  compose: string;
  project?: string;
  setup_scripts?: string[];
  teardown_scripts?: string[];
  env?: Record<string, string>;
  description?: string;
  shell_service?: string;
  shell_command?: string[];
  shell_user?: string;
}

interface ServiceManifest {
  service: Record<string, ServiceConfig>;
}

export interface ServiceStatus {
  name: string;
  status: "running" | "stopped" | "error";
  description?: string;
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
    throw new Error(`Service '${service}' not found under services/`);
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

function composeFilePath(serviceDir: string, config: ServiceConfig): string {
  const candidate = config.compose.startsWith("/")
    ? config.compose
    : resolve(join(serviceDir, config.compose));
  if (!pathExists(candidate)) {
    throw new Error(`compose file not found for ${serviceDir}: ${candidate}`);
  }
  return candidate;
}

function composeProject(service: string, config: ServiceConfig): string {
  return config.project ?? `psyched-${service}`;
}

export function listServices(): string[] {
  const names: string[] = [];
  for (const entry of Deno.readDirSync(servicesRoot())) {
    if (entry.isDirectory) names.push(entry.name);
  }
  names.sort();
  return names;
}

function decode(data: Uint8Array): string {
  return new TextDecoder().decode(data);
}

export async function serviceStatus(name: string): Promise<ServiceStatus> {
  const serviceDir = locateServiceDir(name);
  const config = loadServiceConfig(serviceDir, name);
  const composePath = composeFilePath(serviceDir, config);
  const project = composeProject(name, config);
  const command = new Deno.Command("docker", {
    args: ["compose", "-f", composePath, "-p", project, "ps"],
    cwd: serviceDir,
    env: config.env,
    stdout: "piped",
    stderr: "piped",
  });
  const { success, stdout, stderr } = await command.output();
  if (!success) {
    const message = decode(stderr) || `docker compose ps failed for ${name}`;
    throw new Error(message.trim());
  }
  const output = decode(stdout);
  const running = output.split("\n").slice(1).some((line) =>
    line.includes("Up")
  );
  return {
    name,
    status: running ? "running" : "stopped",
    description: config.description,
  };
}

export async function serviceStatuses(): Promise<ServiceStatus[]> {
  const statuses: ServiceStatus[] = [];
  for (const name of listServices()) {
    try {
      statuses.push(await serviceStatus(name));
    } catch (error) {
      statuses.push({
        name,
        status: "error",
        description: String(error),
      });
    }
  }
  return statuses;
}

export async function setupServices(services: string[]): Promise<void> {
  if (!services.length) return;
  await runPsh(["srv", "setup", ...services]);
}

export async function teardownServices(services: string[]): Promise<void> {
  if (!services.length) return;
  await runPsh(["srv", "teardown", ...services]);
}

export async function bringServiceUp(service: string): Promise<void> {
  await runPsh(["srv", "up", service]);
}

export async function bringServiceDown(service: string): Promise<void> {
  await runPsh(["srv", "down", service]);
}

export async function bringServicesUp(services: string[]): Promise<void> {
  if (!services.length) return;
  await runPsh(["srv", "up", ...services]);
}

export async function bringServicesDown(services: string[]): Promise<void> {
  if (!services.length) return;
  await runPsh(["srv", "down", ...services]);
}
