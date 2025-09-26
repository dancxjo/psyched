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

export function runModuleScript(module: string, action?: string): Promise<void>;
export function runModuleScript(modules: string[], action?: string): Promise<void>;
export async function runModuleScript(modulesOrModule: string | string[], action?: string) {
  // Allow calling with an array of modules (execute sequentially) or a single
  // module name. This mirrors the systemd helpers which accept an optional
  // string[] of units.
  if (Array.isArray(modulesOrModule)) {
    const modules = modulesOrModule;
    if (modules.length === 0) return;
    for (const m of modules) {
      if (!m) continue;
      // Ensure sequential invocation to preserve ordering and avoid noisy
      // parallel output.
      // eslint-disable-next-line no-await-in-loop
      await runModuleScript(m, action);
    }
    return;
  }

  const module = modulesOrModule;
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
  // Special-case: list available modules (print once)
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
        // If the launch_command is a path to a script, run it directly
        if (launchCommand.startsWith("/")) {
          console.log(`[mod] Running launch script for ${module}: ${launchCommand}`);
          await runProcess(commandRunner`bash ${[launchCommand]}`, `${module} launch script`);
          return;
        } else if (launchCommand.startsWith("${MODULE_DIR}")) {
          const absPath = join(moduleDir, launchCommand.replace("${MODULE_DIR}/", ""));
          console.log(`[mod] Running launch script for ${module}: ${absPath}`);
          await runProcess(commandRunner`bash ${[absPath]}`, `${module} launch script`);
          return;
        } else {
          // Fallback: treat as shell command
          console.log(`[mod] Running launch command for ${module}`);
          await runProcess(commandRunner`bash -lc ${launchCommand}`, `${module} launch command`);
          return;
        }
      }
    }

    if (isShutdown) {
      const shutdownCommand = systemd.shutdown_command?.trim();
      if (shutdownCommand) {
        // If the shutdown_command is a path to a script, run it directly
        if (shutdownCommand.startsWith("/")) {
          console.log(`[mod] Running shutdown script for ${module}: ${shutdownCommand}`);
          await runProcess(commandRunner`bash ${[shutdownCommand]}`, `${module} shutdown script`);
          return;
        } else if (shutdownCommand.startsWith("${MODULE_DIR}")) {
          const absPath = join(moduleDir, shutdownCommand.replace("${MODULE_DIR}/", ""));
          console.log(`[mod] Running shutdown script for ${module}: ${absPath}`);
          await runProcess(commandRunner`bash ${[absPath]}`, `${module} shutdown script`);
          return;
        } else {
          // Fallback: treat as shell command
          console.log(`[mod] Running shutdown command for ${module}`);
          await runProcess(commandRunner`bash -lc ${shutdownCommand}`, `${module} shutdown command`);
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
