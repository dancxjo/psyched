import { join } from "@std/path";
import { loadModuleSpec, repoDirFromModules } from "./modules.ts";

async function fileExists(path: string): Promise<boolean> {
  try {
    const stat = await Deno.stat(path);
    return stat.isFile;
  } catch (err) {
    if (err instanceof Deno.errors.NotFound) return false;
    throw err;
  }
}

async function runProcess(cmd: Deno.Command, label: string): Promise<void> {
  const child = cmd.spawn();
  const status = await child.status;
  if (!status.success) {
    console.error(`[mod] ${label} failed with code ${status.code ?? 1}`);
    Deno.exit(status.code ?? 1);
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

  for (const act of actionOrder) {
    const scriptPath = join(moduleDir, `${act}.sh`);
    if (await fileExists(scriptPath)) {
      console.log(`[mod] Running: ${scriptPath}`);
      const cmd = new Deno.Command("bash", {
        args: [scriptPath],
        stdout: "inherit",
        stderr: "inherit",
      });
      await runProcess(cmd, scriptPath);
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
        const cmd = new Deno.Command(entrypoint, {
          args: ["bash", "-lc", launchCommand],
          stdout: "inherit",
          stderr: "inherit",
        });
        await runProcess(cmd, `${module} launch command`);
        return;
      }
      const launchScript = systemd.launch;
      if (launchScript) {
        const absPath = join(repoDir, launchScript);
        if (await fileExists(absPath)) {
          console.log(`[mod] Running launch script for ${module}: ${absPath}`);
          const cmd = new Deno.Command(entrypoint, {
            args: [absPath],
            stdout: "inherit",
            stderr: "inherit",
          });
          await runProcess(cmd, `${module} launch script`);
          return;
        }
      }
    }

    if (isShutdown) {
      const shutdownCommand = systemd.shutdown_command?.trim();
      if (shutdownCommand) {
        console.log(`[mod] Running shutdown command for ${module}`);
        const cmd = new Deno.Command(entrypoint, {
          args: ["bash", "-lc", shutdownCommand],
          stdout: "inherit",
          stderr: "inherit",
        });
        await runProcess(cmd, `${module} shutdown command`);
        return;
      }
      const shutdownScript = systemd.shutdown;
      if (shutdownScript) {
        const absPath = join(repoDir, shutdownScript);
        if (await fileExists(absPath)) {
          console.log(
            `[mod] Running shutdown script for ${module}: ${absPath}`,
          );
          const cmd = new Deno.Command(entrypoint, {
            args: ["bash", "-lc", absPath],
            stdout: "inherit",
            stderr: "inherit",
          });
          await runProcess(cmd, `${module} shutdown script`);
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
