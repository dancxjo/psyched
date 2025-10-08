import { hostsRoot } from "./paths.ts";
import { runPsh } from "./psh_cli.ts";

function pathExists(path: string): boolean {
  try {
    Deno.statSync(path);
    return true;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) return false;
    throw error;
  }
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

export async function provisionHosts(hosts?: string[]): Promise<void> {
  if (hosts && hosts.length === 0) return;
  if (!hosts || hosts.length === 0) {
    await runPsh(["host", "setup"]);
    return;
  }
  await runPsh(["host", "setup", ...hosts]);
}

export function locateHostConfig(hostname: string): string {
  const candidate = `${hostsRoot()}/${hostname}.toml`;
  if (!pathExists(candidate)) {
    throw new Error(`Host profile '${hostname}' not found.`);
  }
  return candidate;
}
