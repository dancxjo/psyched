#!/usr/bin/env -S deno run -A

import { Command } from "@cliffy/command";
import { setup } from "./setup.ts";
// Use the project's shell runner for readable command execution when we need
// to run external scripts from TypeScript.
// deno-lint-ignore-file no-import-prefix no-unversioned-import
// @ts-ignore runtime import via jsr
import $ from "jsr:@david/dax";

function repoPath(relative: string) {
  // Resolve a path relative to this file (psh/...). Example: '../tools/install_ros2.sh'
  return decodeURIComponent(new URL(relative, import.meta.url).pathname);
}

async function installPsh() {
  const scriptPath = decodeURIComponent(new URL(import.meta.url).pathname);
  const wrapper = `#!/usr/bin/env bash\nexec deno run -A '${scriptPath}' "${"$@"}"\n`;

  console.log("Installing psh to /usr/bin/psh (requires sudo)...");

  // If /usr/bin/psh already exists and its contents exactly match the
  // desired wrapper, do nothing. This makes the install idempotent and
  // avoids unnecessary writes that could change permissions/mtime.
  const exists = await $`sudo test -f /usr/bin/psh`;
  if (exists.code === 0) {
    // Dump the root-owned file to a temp file using sudo, then read it with
    // Deno. This avoids attempting to access command stdout via the shell
    // runner which may not be configured to pipe stdout.
    const tmp = `/tmp/psh_existing.${Deno.pid}`;
    const dump = await $`sudo bash -c 'cat /usr/bin/psh > ${$.path(tmp)}'`;
    if (dump.code !== 0) {
      console.log("Unable to read existing /usr/bin/psh; will overwrite.");
    } else {
      try {
        const existing = await Deno.readTextFile(tmp);
        if (existing.trim() === wrapper.trim()) {
          console.log("psh is already installed and up-to-date; skipping write.");
          // cleanup
          await $`sudo rm -f ${$.path(tmp)}`;
          return;
        }
        console.log("Existing /usr/bin/psh differs from desired wrapper; updating...");
      } catch (_err) {
        console.log("Failed to read temp file; will overwrite /usr/bin/psh.");
      }
      // cleanup
      await $`sudo rm -f ${$.path(tmp)}`;
    }
  }

  // Stream the wrapper content into sudo tee to avoid nested quoting/heredoc
  // issues and to ensure we don't call tee with no stdin (which blocks).
  const write = await $`printf '%s\n' ${wrapper} | sudo tee /usr/bin/psh >/dev/null`;
  if (write.code !== 0) {
    console.error("Failed to write /usr/bin/psh:", write.stderr || write.stdout);
    Deno.exit(write.code);
  }
  const chmod = await $`sudo chmod a+x /usr/bin/psh`;
  if (chmod.code !== 0) {
    console.error("Failed to set executable on /usr/bin/psh:", chmod.stderr || chmod.stdout);
    Deno.exit(chmod.code);
  }
  console.log("psh installed to /usr/bin/psh");
}

async function runInstallRos2() {
  const script = repoPath('../tools/install_ros2.sh');
  console.log(`Running ROS2 install script: ${script}`);
  const r = await $`bash ${script}`;
  if (r.code !== 0) {
    console.error(r.stderr || r.stdout);
    Deno.exit(r.code);
  }
}

async function runInstallDocker() {
  const script = repoPath('../tools/install_docker.sh');
  console.log(`Running Docker install script: ${script}`);
  const r = await $`bash ${script}`;
  if (r.code !== 0) {
    console.error(r.stderr || r.stdout);
    Deno.exit(r.code);
  }
}

async function systemdGenerate() {
  const script = repoPath('../tools/systemd_generate');
  console.log(`Running systemd_generate -> ${script}`);
  const r = await $`bash ${script}`;
  if (r.code !== 0) {
    console.error(r.stderr || r.stdout);
    Deno.exit(r.code);
  }
}

async function systemdInstall() {
  // Generate first
  await systemdGenerate();

  // Determine host shortname
  const hn = await $`hostname -s`;
  const host = (hn.stdout || hn.stderr || "").toString().trim() || Deno.env.get('HOST') || "$(hostname)";
  const unitsDir = `${Deno.cwd()}/hosts/${host}/systemd`;

  try {
    const stat = await Deno.stat(unitsDir);
    if (!stat.isDirectory) throw new Error('not dir');
  } catch (_err) {
    console.error(`No generated unit directory at ${unitsDir}`);
    Deno.exit(1);
  }

  console.log(`Installing unit files from ${unitsDir} -> /etc/systemd/system`);
  for await (const ent of Deno.readDir(unitsDir)) {
    if (!ent.name.endsWith('.service')) continue;
    const src = `${unitsDir}/${ent.name}`;
    const cp = await $`sudo cp ${src} /etc/systemd/system/${ent.name}`;
    if (cp.code !== 0) {
      console.error(`Failed to copy ${ent.name}:`, cp.stderr || cp.stdout);
      Deno.exit(cp.code);
    }
    const enable = await $`sudo systemctl enable --now ${ent.name}`;
    if (enable.code !== 0) {
      console.error(`Failed to enable ${ent.name}:`, enable.stderr || enable.stdout);
      // Continue; services can be enabled individually later
    }
    console.log(`Installed and enabled ${ent.name}`);
  }

  const reload = await $`sudo systemctl daemon-reload`;
  if (reload.code !== 0) {
    console.error(reload.stderr || reload.stdout);
    Deno.exit(reload.code);
  }
  console.log('Systemd units installed and daemon reloaded.');
}

async function uninstallPsh() {
  console.log('Uninstalling /usr/bin/psh (requires sudo)...');
  const exists = await $`sudo test -f /usr/bin/psh`;
  if (exists.code !== 0) {
    console.log('/usr/bin/psh does not exist; nothing to do.');
    return;
  }

  const rm = await $`sudo rm -f /usr/bin/psh`;
  if (rm.code !== 0) {
    console.error('Failed to remove /usr/bin/psh:', rm.stderr || rm.stdout);
    Deno.exit(rm.code);
  }
  console.log('Removed /usr/bin/psh');
}

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
  await new Command()
    .name("psh")
    .description("Psyched CLI")
    .version("v1.0.0")
    .globalOption("-d, --debug", "Enable debug output.")
    .action((_options: Record<string, unknown>, ..._args: string[]) => { })
    .command("install", "Install psh to /usr/bin/psh")
    .action(async () => await installPsh())
    .command("setup", "Setup sub-command.")
    .arguments("[target:string] [...rest:string]")
    .action(async (options: Record<string, unknown>, ...args: string[]) => {
      // Support multiple setup targets, e.g. `psh setup ros2 docker`.
      // If no known targets are provided, fall back to the existing setup() behaviour.
      if (!args || args.length === 0) {
        await setup(options, args as unknown[]);
        return;
      }

      const requested = args.map((a) => a.toString());

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
    .action(async (_options: Record<string, unknown>, ...args: string[]) => {
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
    .action(async (_options: Record<string, unknown>, ...args: string[]) => {
      const target = args && args[0] ? args[0].toString() : 'all';
      if (target === 'install') {
        await uninstallPsh();
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
