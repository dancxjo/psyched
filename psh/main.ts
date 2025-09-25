#!/usr/bin/env -S deno run -A

import { Command } from "@cliffy/command";
import { setup } from "./setup.ts";
import { repoPath, $ } from "./util.ts";
import { runInstallRos2, runInstallDocker } from "./install.ts";
import { systemdGenerate, systemdInstall } from "./systemd.ts";
import { uninstallPsh } from "./uninstall.ts";
import { colors } from "@cliffy/ansi/colors";


async function systemdUninstall() {
  // Determine host shortname
  const hn = await $`hostname -s`;
  const host = (hn.stdout || hn.stderr || "").toString().trim() || Deno.env.get('HOST') || "$(hostname)";
  const unitsDir = `${Deno.cwd()}/hosts/${host}/systemd`;

  try {
    const stat = await Deno.stat(unitsDir);
    if (!stat.isDirectory) throw new Error('not dir');
  } catch (_err) {
    console.log(`No generated unit directory at ${unitsDir}; skipping systemd cleanup.`);
    return;
  }

  console.log(`Removing installed systemd unit files listed in ${unitsDir} from /etc/systemd/system`);
  for await (const ent of Deno.readDir(unitsDir)) {
    if (!ent.name.endsWith('.service')) continue;
    const name = ent.name;
    // Try to stop & disable the service first
    const disable = await $`sudo systemctl disable --now ${name}`;
    if (disable.code !== 0) {
      console.log(`Warning: failed to disable ${name}:`, disable.stderr || disable.stdout);
      // Continue to attempt removal anyway
    } else {
      console.log(`Disabled ${name}`);
    }

    const rm = await $`sudo rm -f /etc/systemd/system/${name}`;
    if (rm.code !== 0) {
      console.log(`Warning: failed to remove /etc/systemd/system/${name}:`, rm.stderr || rm.stdout);
    } else {
      console.log(`Removed /etc/systemd/system/${name}`);
    }
  }

  const reload = await $`sudo systemctl daemon-reload`;
  if (reload.code !== 0) {
    console.log('Warning: failed to reload systemd daemon:', reload.stderr || reload.stdout);
  } else {
    console.log('systemd daemon reloaded.');
  }
}

if (import.meta.main) {
  // Import ansi and table from cliffy
  const { Table } = await import("@cliffy/table");
  // Use statically imported colors from @cliffy/ansi

  await new Command()
    .name("psh")
    .description("Psyched CLI")
    .version("v1.0.0")
    .globalOption("-d, --debug", "Enable debug output.")
    .action(async () => {
      // Print a colored welcome banner
      console.log(colors.bold.underline.rgb24("Welcome to Psyched Robotics", 0x00bfff));
      console.log(colors.gray("─────────────────────────────────────────────"));

      // Print current setup summary in a beautiful table (suppress all info-gathering output)
      let ros2Installed = false;
      try {
        const ros2Dir = "/opt/ros/";
        for await (const ent of Deno.readDir(ros2Dir)) {
          if (ent.isDirectory) {
            ros2Installed = true;
            break;
          }
        }
      } catch {
        ros2Installed = false;
      }

      let dockerInstalled = false;
      try {
        const r = await $`docker --version`.stdout("null").stderr("null");
        dockerInstalled = r.code === 0;
      } catch {
        dockerInstalled = false;
      }

      const hn = await $`hostname -s`.stdout("piped").stderr("piped");
      const host = (hn.stdout || hn.stderr || "").toString().trim() || Deno.env.get("HOST") || "$(hostname)";
      const tomlPath = `${Deno.cwd()}/hosts/${host}.toml`;
      let modules: string[] = [];
      try {
        const tomlText = await Deno.readTextFile(tomlPath);
        const { parse: parseToml } = await import("@std/toml");
        const spec = parseToml(tomlText) as Record<string, unknown>;
        if (Array.isArray(spec.modules)) modules = spec.modules as string[];
      } catch {
        try {
          const modsDir = `${Deno.cwd()}/hosts/${host}/modules`;
          for await (const ent of Deno.readDir(modsDir)) {
            modules.push(ent.name);
          }
        } catch { /* ignore missing modules dir */ }
      }

      // Gather running status for each module
      const running: string[] = [];
      const moduleStatus: Array<[string, boolean, boolean]> = [];
      for (const mod of modules) {
        let isRunning = false;
        try {
          const r = await $`systemctl is-active psyched-${mod}.service`.stdout("null").stderr("null");
          isRunning = (r.stdout || "").toString().trim() === "active";
        } catch { /* ignore systemctl errors */ }
        moduleStatus.push([mod, true, isRunning]);
        if (isRunning) running.push(mod);
      }

      // Gather systemd units and their status
      const unitsDir = `${Deno.cwd()}/hosts/${host}/systemd`;
      const units: Array<{ name: string, active: boolean }> = [];
      try {
        for await (const ent of Deno.readDir(unitsDir)) {
          if (ent.name.endsWith('.service')) {
            let active = false;
            try {
              const r = await $`systemctl is-active ${ent.name}`.stdout("null").stderr("null");
              active = (r.stdout || "").toString().trim() === "active";
            } catch { }
            units.push({ name: ent.name, active });
          }
        }
      } catch { /* ignore missing systemd dir */ }

      // Build summary table
      const table = new Table()
        .header([
          colors.bold("Info"),
          colors.bold("Status")
        ])
        .body([
          [colors.cyan("ROS2 Installed"), ros2Installed ? colors.green("Yes") : colors.red("No")],
          [colors.cyan("Docker Installed"), dockerInstalled ? colors.green("Yes") : colors.red("No")],
          [colors.cyan("Enabled Modules"), modules.length ? colors.yellow(modules.join(", ")) : colors.gray("none")],
          [colors.cyan("Running Modules"), running.length ? colors.green(running.join(", ")) : colors.gray("none")],
          [colors.cyan("Systemd Units"), units.length ? colors.magenta(units.map(u => u.name).join(", ")) : colors.gray("none")],
        ])
        .border(true)
        .render();

      // Build module status table
      if (modules.length) {
        // Flip: columns are module names, rows are statuses
        const header = [colors.bold("Status"), ...modules.map(mod => colors.bold(colors.cyan(mod)))];
        const enabledRow = [colors.bold("Enabled"), ...moduleStatus.map(([_, enabled, _r]) => enabled ? colors.green("Yes") : colors.red("No"))];
        const runningRow = [colors.bold("Running"), ...moduleStatus.map(([_, _e, running]) => running ? colors.green("Yes") : colors.red("No"))];
        // Table is rendered, variable is unused
        // deno-lint-ignore no-unused-vars
        const modTable = new Table()
          .header(header)
          .body([
            enabledRow,
            runningRow
          ])
          .border(true)
          .render();
      }

      // Build flipped systemd units table (columns: unit names, row: Active status)
      if (units.length) {
        const header = units.map(u => colors.bold(colors.magenta(u.name)));
        const statusRow = units.map(u => u.active ? colors.green("Yes") : colors.red("No"));
        const _unitTable = new Table()
          .header(header)
          .body([statusRow])
          .border(true)
          .render();
      }

      // Friendly tip
      console.log(colors.gray("Tip: Use 'psh setup' or 'psh bringup' to configure modules."));
    })
    .command("install", "Install psh to /usr/bin/psh")
    .action(async () => await uninstallPsh())
    .command("setup", "Setup sub-command.")
    .arguments("[target:string] [...rest:string]")
    .action(async (options: Record<string, unknown>, ...args: (string | undefined)[]) => {
      // Support multiple setup targets, e.g. `psh setup ros2 docker`.
      // If no known targets are provided, fall back to the existing setup() behaviour.
      if (!args || args.length === 0) {
        await setup(options, args as unknown[]);
        return;
      }

      const requested = args.filter(Boolean).map((a) => String(a));

      // Run matching installers in the order they were provided.
      let ranAny = false;
      for (const t of requested) {
        if (t === 'ros2') {
          await runInstallRos2();
          ranAny = true;
          continue;
        }
        if (t === 'docker') {
          await runInstallDocker();
          ranAny = true;
          continue;
        }
      }

      // If we didn't run any known installers, pass through to setup() for
      // backward compatibility.
      if (!ranAny) {
        await setup(options, args as unknown[]);
      }
    })
    .command("systemd", "Manage systemd units for this host")
    .alias("sys")
    .arguments("[action:string]")
    .action(async (_options: Record<string, unknown>, ...args: (string | undefined)[]) => {
      const action = args[0] || 'generate';
      if (action === '*' || action === 'generate' || action === 'gen') {
        await systemdGenerate();
        return;
      }
      if (action === 'install') {
        await systemdInstall();
        return;
      }
      console.error(`Unknown systemd action: ${action}`);
      Deno.exit(2);
    })
    // (removed unused 'bar' command)
    .command("env", "Print the path to the environment setup script (for sourcing)")
    .action(() => {
      // Print the absolute path to tools/setup_env.sh so callers can do:
      //   source $(psh env)
      console.log(repoPath('../tools/setup_env.sh'));
    })
    .command("clean", "Undo work made by psh and remove generated garbage")
    .arguments("[target:string]")
    .action(async (_options: Record<string, unknown>, ...args: (string | undefined)[]) => {
      const target = args && args[0] ? String(args[0]) : 'all';
      if (target === 'install') {
        await uninstallPsh(); // installPsh is now imported from uninstall.ts
        return;
      }
      if (target === 'systemd') {
        await systemdUninstall();
        return;
      }
      if (target === '*' || target === 'all') {
        await uninstallPsh();
        await systemdUninstall();
        return;
      }
      console.error(`Unknown clean target: ${target}`);
      Deno.exit(2);
    })
    .parse(Deno.args);
}
