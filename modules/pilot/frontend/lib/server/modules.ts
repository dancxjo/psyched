import { join, resolve } from "$std/path/mod.ts";
import { enabledModulesForHost } from "./host_config.ts";
import { modulesRoot, workspaceRoot } from "./paths.ts";
import { runPsh } from "./psh_cli.ts";
import { ModuleStatus, moduleStatuses } from "./module_status.ts";

export interface ListModulesOptions {
  hostname?: string;
  hostsDir?: string;
  modulesDir?: string;
  includePilot?: boolean;
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

function pidPath(module: string): string {
  return join(workspaceRoot(), ".psh", `${module}.pid`);
}

function readModuleDirectories(dir: string): string[] {
  const names: string[] = [];
  try {
    for (const entry of Deno.readDirSync(dir)) {
      if (entry.isDirectory) names.push(entry.name);
    }
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return [];
    }
    throw error;
  }
  names.sort();
  return names;
}

export function listModules(options: ListModulesOptions = {}): string[] {
  const modulesDir = options.modulesDir
    ? resolve(options.modulesDir)
    : modulesRoot();
  const includePilot = options.includePilot ?? false;
  const directories = readModuleDirectories(modulesDir);
  const directorySet = new Set(directories);

  try {
    const { modules } = enabledModulesForHost({
      hostname: options.hostname,
      hostsDir: options.hostsDir,
      includePilot,
    });
    if (modules.length > 0) {
      return modules.filter((name) => directorySet.has(name));
    }
  } catch (error) {
    console.warn("Failed to resolve host modules", error);
  }

  return includePilot
    ? directories
    : directories.filter((name) => name !== "pilot");
}

export { ModuleStatus, moduleStatuses };

export function isModuleRunning(module: string): boolean {
  return pathExists(pidPath(module));
}

async function execModuleCommand(
  command: "setup" | "teardown" | "up" | "down",
  modules: string[],
  options: { verbose?: boolean } = {},
): Promise<void> {
  if (!modules.length) return;
  const args = ["mod", command, ...modules];
  if (options.verbose && command === "up") {
    args.splice(3, 0, "--verbose");
  }
  await runPsh(args);
}

export async function setupModules(modules: string[]): Promise<void> {
  await execModuleCommand("setup", modules);
}

export async function teardownModules(modules: string[]): Promise<void> {
  await execModuleCommand("teardown", modules);
}

export async function bringModulesUp(
  modules: string[],
  options: { verbose?: boolean } = {},
): Promise<void> {
  await execModuleCommand("up", modules, options);
}

export async function bringModulesDown(modules: string[]): Promise<void> {
  await execModuleCommand("down", modules);
}
