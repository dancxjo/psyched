import { fromFileUrl, join } from "$std/path/mod.ts";

const REPO_ROOT = fromFileUrl(new URL("../../../../../", import.meta.url));
const MODULES_ROOT = join(REPO_ROOT, "modules");
const SERVICES_ROOT = join(REPO_ROOT, "services");
const HOSTS_ROOT = join(REPO_ROOT, "hosts");
const WORKSPACE_ROOT = join(REPO_ROOT, "work");
const TOOLS_ROOT = join(REPO_ROOT, "tools");

export function repoRoot(): string {
  return REPO_ROOT;
}

export function modulesRoot(): string {
  return MODULES_ROOT;
}

export function servicesRoot(): string {
  return SERVICES_ROOT;
}

export function hostsRoot(): string {
  return HOSTS_ROOT;
}

export function workspaceRoot(): string {
  return WORKSPACE_ROOT;
}

export function pshRoot(): string {
  return join(TOOLS_ROOT, "psh");
}
