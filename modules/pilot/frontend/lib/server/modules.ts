import { join } from "$std/path/mod.ts";
import { modulesRoot, workspaceRoot } from "./paths.ts";
import { runPsh } from "./psh_cli.ts";
import { ModuleStatus, moduleStatuses } from "./module_status.ts";

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

export function listModules(): string[] {
  const names: string[] = [];
  for (const entry of Deno.readDirSync(modulesRoot())) {
    if (entry.isDirectory) names.push(entry.name);
  }
  names.sort();
  return names;
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
