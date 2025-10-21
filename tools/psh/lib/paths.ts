import { dirname, fromFileUrl, join, resolve } from "$std/path/mod.ts";

function pathExistsSync(path: string): boolean {
  try {
    Deno.statSync(path);
    return true;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return false;
    }
    throw error;
  }
}

function isDirectory(path: string): boolean {
  try {
    return Deno.statSync(path).isDirectory;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return false;
    }
    throw error;
  }
}

function expandTilde(path: string): string {
  if (path.startsWith("~/")) {
    const home = Deno.env.get("HOME");
    if (!home) {
      throw new Error(
        "HOME environment variable is not set; cannot expand '~/'",
      );
    }
    return join(home, path.slice(2));
  }
  return path;
}

function resolveWithinWorkspace(envVar: string, fallback: string): string {
  const override = Deno.env.get(envVar);
  if (override) {
    const expanded = expandTilde(override);
    if (expanded.startsWith("/")) {
      return resolve(expanded);
    }
    return resolve(join(workspaceRoot(), expanded));
  }
  return resolve(join(workspaceRoot(), fallback));
}

const repoRootCache = new Map<string, string>();

function candidateIsRepoRoot(candidate: string): boolean {
  return (
    isDirectory(join(candidate, "modules")) &&
    pathExistsSync(join(candidate, "tools", "psh", "deno.json"))
  );
}

export function repoRoot(): string {
  const override = Deno.env.get("PSYCHED_REPO_ROOT");
  const cacheKey = override ? `repo:${override}` : "repo";
  const cached = repoRootCache.get(cacheKey);
  if (cached) {
    return cached;
  }

  if (override) {
    const expanded = expandTilde(override);
    if (candidateIsRepoRoot(expanded)) {
      const resolved = resolve(expanded);
      repoRootCache.set(cacheKey, resolved);
      return resolved;
    }
  }

  const scriptDir = dirname(fromFileUrl(import.meta.url));
  const searchRoots = new Set<string>([
    Deno.cwd(),
    resolve(join(scriptDir, "../../..")),
  ]);

  for (const root of Array.from(searchRoots)) {
    let current = resolve(root);
    while (true) {
      if (candidateIsRepoRoot(current)) {
        repoRootCache.set("repo", current);
        return current;
      }
      const parent = dirname(current);
      if (parent === current) {
        break;
      }
      current = parent;
    }
  }

  throw new Error(
    "Unable to determine psyched repository root. Run inside the repository or set PSYCHED_REPO_ROOT.",
  );
}

export function workspaceRoot(): string {
  const override = Deno.env.get("PSYCHED_WORKSPACE_DIR");
  if (override) {
    const expanded = expandTilde(override);
    if (expanded.startsWith("/")) {
      return resolve(expanded);
    }
    return resolve(join(repoRoot(), expanded));
  }
  return resolve(join(repoRoot(), "work"));
}

export function workspaceSrc(): string {
  return resolveWithinWorkspace("PSYCHED_WORKSPACE_SRC", "src");
}

export function workspaceInstall(): string {
  return resolveWithinWorkspace("PSYCHED_WORKSPACE_INSTALL", "install");
}

export function modulesRoot(): string {
  return resolve(join(repoRoot(), "modules"));
}

export function servicesRoot(): string {
  return resolve(join(repoRoot(), "services"));
}

export function hostsRoot(): string {
  return resolve(join(repoRoot(), "hosts"));
}

export function scriptsRoot(): string {
  return resolve(join(repoRoot(), "tools"));
}
