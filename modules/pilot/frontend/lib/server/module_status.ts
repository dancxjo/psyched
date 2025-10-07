import { fromFileUrl, join } from "$std/path/mod.ts";

/**
 * Options for customizing how module status information is discovered.
 *
 * These hooks primarily exist to make the helpers testable: production code
 * relies on the defaults that resolve paths relative to this source file.
 */
export interface ModuleStatusOptions {
  /**
   * Absolute path to the repository root. Defaults to the root detected from
   * this file's location (../../../../../).
   */
  repoRoot?: string;
  /**
   * Absolute path to the workspace directory. Defaults to `<repo>/work`.
   */
  workspaceRoot?: string;
  /**
   * Absolute path to the modules directory. Defaults to `<repo>/modules`.
   */
  modulesRoot?: string;
}

/**
 * Status reported for each module controlled by `psh`.
 */
export interface ModuleStatus {
  name: string;
  status: "running" | "stopped";
  pid?: number;
}

const DEFAULT_REPO_ROOT = fromFileUrl(
  new URL("../../../../../", import.meta.url),
);
const DEFAULT_WORKSPACE_ROOT = join(DEFAULT_REPO_ROOT, "work");
const DEFAULT_MODULES_ROOT = join(DEFAULT_REPO_ROOT, "modules");

function determineRepoRoot(options: ModuleStatusOptions): string {
  return options.repoRoot ?? DEFAULT_REPO_ROOT;
}

function determineWorkspaceRoot(
  repoRoot: string,
  options: ModuleStatusOptions,
): string {
  if (options.workspaceRoot) return options.workspaceRoot;
  if (repoRoot === DEFAULT_REPO_ROOT) return DEFAULT_WORKSPACE_ROOT;
  return join(repoRoot, "work");
}

function determineModulesRoot(
  repoRoot: string,
  options: ModuleStatusOptions,
): string {
  if (options.modulesRoot) return options.modulesRoot;
  if (repoRoot === DEFAULT_REPO_ROOT) return DEFAULT_MODULES_ROOT;
  return join(repoRoot, "modules");
}

function pidDirectory(workspaceRoot: string): string {
  return join(workspaceRoot, ".psh");
}

function modulePidPath(workspaceRoot: string, module: string): string {
  return join(pidDirectory(workspaceRoot), `${module}.pid`);
}

function readPid(workspaceRoot: string, module: string): number | null {
  try {
    const contents = Deno.readTextFileSync(modulePidPath(workspaceRoot, module))
      .trim();
    if (!contents) return null;
    const parsed = Number(contents);
    return Number.isNaN(parsed) ? null : parsed;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return null;
    }
    throw error;
  }
}

function clearPid(workspaceRoot: string, module: string): void {
  try {
    Deno.removeSync(modulePidPath(workspaceRoot, module));
  } catch (error) {
    if (!(error instanceof Deno.errors.NotFound)) {
      throw error;
    }
  }
}

function isPidRunning(pid: number): boolean {
  try {
    Deno.statSync(`/proc/${pid}`);
    return true;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return false;
    }
    throw error;
  }
}

function listModules(modulesRoot: string): string[] {
  try {
    const names: string[] = [];
    for (const entry of Deno.readDirSync(modulesRoot)) {
      if (entry.isDirectory) names.push(entry.name);
    }
    names.sort();
    return names;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return [];
    }
    throw error;
  }
}

/**
 * Inspect the repository workspace to determine the launch status of modules.
 *
 * The detection logic mirrors the subset of `tools/psh/lib/module.ts` required
 * by the frontend without pulling the entire CLI dependency graph into the
 * browser bundle. Each status reflects the presence of a PID file under
 * `<workspace>/.psh` and whether the process is still running.
 *
 * When a stale PID file is encountered the helper clears it to ensure future
 * reads do not report the module as running.
 */
export function moduleStatuses(
  options: ModuleStatusOptions = {},
): ModuleStatus[] {
  const repoRoot = determineRepoRoot(options);
  const workspaceRoot = determineWorkspaceRoot(repoRoot, options);
  const modulesRoot = determineModulesRoot(repoRoot, options);
  const results: ModuleStatus[] = [];

  for (const name of listModules(modulesRoot)) {
    const pid = readPid(workspaceRoot, name);
    if (pid && isPidRunning(pid)) {
      results.push({ name, status: "running", pid });
      continue;
    }
    if (pid) {
      clearPid(workspaceRoot, name);
    }
    results.push({ name, status: "stopped" });
  }

  return results;
}
