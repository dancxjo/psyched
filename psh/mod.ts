import { join } from "@std/path";
import {
  loadModuleSpec,
  repoDirFromModules,
  prepareModuleContext,
  applyModuleActions,
  cleanupModuleContext,
} from "./modules.ts";
import { $, type DaxCommandBuilder, type DaxTemplateTag } from "./util.ts";

let commandRunner: DaxTemplateTag = $;

/** Allow tests to intercept shell execution. */
export function setModCommandRunner(runner: DaxTemplateTag): void {
  commandRunner = runner;
}

/** Restore the default Dax command runner. */
export function resetModCommandRunner(): void {
  commandRunner = $;
}

async function fileExists(path: string): Promise<boolean> {
  try {
    const stat = await Deno.stat(path);
    return stat.isFile;
  } catch (err) {
    if (err instanceof Deno.errors.NotFound) return false;
    throw err;
  }
}

async function runProcess(
  builder: DaxCommandBuilder,
  label: string,
): Promise<void> {
  const result = await builder.stdout("inherit").stderr("inherit").noThrow();
  if (result.code !== 0) {
    console.error(`[mod] ${label} failed with code ${result.code ?? 1}`);
    Deno.exit(result.code ?? 1);
  }
}

export async function runModuleScript(module: string, action?: string) {
  const repoDir = repoDirFromModules();
  const moduleDir = join(repoDir, "modules", module);
  const entrypoint = join(repoDir, "tools", "systemd_entrypoint.sh");
  const requested = action?.toLowerCase();
  const actionOrder = [
    requested,
    ...(requested ? [] : ["launch", "start", "up"]),
  ].filter((value): value is string => Boolean(value));

  const specInfo = await loadModuleSpec(module).catch(() => null);
  const systemd = specInfo?.spec.systemd;

  // Special-case: run the action for all modules when module is '*' or 'all'.
  if (module === "*" || module.toLowerCase?.() === "all") {
    try {
      for await (const entry of Deno.readDir(join(repoDir, "modules"))) {
        if (!entry.isDirectory) continue;
        // skip hidden entries
        if (entry.name.startsWith(".")) continue;
        console.log(`[mod] Invoking ${action ?? "(default)"} for module: ${entry.name}`);
        // await each module sequentially to avoid noisy parallel shell output
        // and to preserve ordering when linking packages into src/
        // eslint-disable-next-line no-await-in-loop
        await runModuleScript(entry.name, action);
      }
    } catch (err) {
      console.error(`[mod] Failed iterating modules: ${err}`);
      Deno.exit(2);
    }
    return;
  }

  // Special-case: list available modules
  if (requested === "list") {
    try {
      console.log("Available modules:");
      for await (const entry of Deno.readDir(join(repoDir, "modules"))) {
        if (!entry.isDirectory) continue;
        if (entry.name.startsWith(".")) continue;
        console.log(` - ${entry.name}`);
      }
      return;
    } catch (err) {
      console.error(`[mod] Failed listing modules: ${err}`);
      Deno.exit(2);
    }
  }

  // Special-case: module setup (link packages, install deps, etc.)
  if (requested === "setup") {
    const spec = specInfo?.spec;
    const ctx = await prepareModuleContext(module, null);
    try {
      await applyModuleActions(spec?.actions, ctx);
    } finally {
      await cleanupModuleContext(ctx);
    }
    return;
  }

  for (const act of actionOrder) {
    const scriptPath = join(moduleDir, `${act}.sh`);
    if (await fileExists(scriptPath)) {
      console.log(`[mod] Running: ${scriptPath}`);
      await runProcess(commandRunner`bash ${[scriptPath]}`, scriptPath);
      return;
    }

    if (!systemd) {
      continue;
    }

    const isLaunch = ["launch", "start", "up"].includes(act);
    const isShutdown = ["shutdown", "stop", "down"].includes(act);

    if (isLaunch) {
      const launchCommand = systemd.launch_command?.trim();
      if (launchCommand) {
        console.log(`[mod] Running launch command for ${module}`);
        await runProcess(
          commandRunner`${entrypoint} ${["bash", "-lc", launchCommand]}`,
          `${module} launch command`,
        );
        return;
      }
      const launchScript = systemd.launch;
      if (launchScript) {
        const absPath = join(repoDir, launchScript);
        if (await fileExists(absPath)) {
          console.log(`[mod] Running launch script for ${module}: ${absPath}`);
          await runProcess(
            commandRunner`${entrypoint} ${[absPath]}`,
            `${module} launch script`,
          );
          return;
        }
      }
    }

    if (isShutdown) {
      const shutdownCommand = systemd.shutdown_command?.trim();
      if (shutdownCommand) {
        console.log(`[mod] Running shutdown command for ${module}`);
        await runProcess(
          commandRunner`${entrypoint} ${["bash", "-lc", shutdownCommand]}`,
          `${module} shutdown command`,
        );
        return;
      }
      const shutdownScript = systemd.shutdown;
      if (shutdownScript) {
        const absPath = join(repoDir, shutdownScript);
        if (await fileExists(absPath)) {
          console.log(
            `[mod] Running shutdown script for ${module}: ${absPath}`,
          );
          await runProcess(
            commandRunner`${entrypoint} ${["bash", "-lc", absPath]}`,
            `${module} shutdown script`,
          );
          return;
        }
      }
    }
  }

  const label = action ?? "(default)";
  console.error(
    `[mod] No runnable command found for module '${module}' action '${label}'`,
  );
  Deno.exit(2);
}
