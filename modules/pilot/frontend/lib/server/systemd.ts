import { runPsh } from "./psh_cli.ts";

export async function setupSystemd(module: string): Promise<void> {
  await runPsh(["sys", "setup", module]);
}

export async function teardownSystemd(module: string): Promise<void> {
  await runPsh(["sys", "teardown", module]);
}

export async function enableSystemd(module: string): Promise<void> {
  await runPsh(["sys", "enable", module]);
}

export async function disableSystemd(module: string): Promise<void> {
  await runPsh(["sys", "disable", module]);
}

export async function startSystemd(module: string): Promise<void> {
  await runPsh(["sys", "up", module]);
}

export async function stopSystemd(module: string): Promise<void> {
  await runPsh(["sys", "down", module]);
}
