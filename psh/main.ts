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
  const wrapper = `#!/usr/bin/env bash\nexec deno run -A '${scriptPath}' "$@"\n`;

  console.log("Installing psh to /usr/bin/psh (requires sudo)...");
  // Write wrapper and make executable
  const tee = await $`sudo tee /usr/bin/psh >/dev/null`;
  // Use a here-doc to send the wrapper content to sudo tee
  const write = await $`sudo bash -c "cat > /usr/bin/psh <<'PSH'\n${wrapper}\nPSH"`;
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
  } catch (err) {
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

if (import.meta.main) {
  await new Command()
    .name("psh")
    .description("Psyched CLI")
    .version("v1.0.0")
    .globalOption("-d, --debug", "Enable debug output.")
    .action((_options, ..._args) => console.log("Main command called."))
    .command("install", "Install psh to /usr/bin/psh")
    .action(async () => await installPsh())
    .command("setup", "Setup sub-command.")
    .option("-f, --foo", "Foo option.")
    .arguments("[target:string] [...rest:string]")
    .action(async (options, ...args) => {
      const target = args[0];
      if (target === 'ros2') {
        await runInstallRos2();
        return;
      }
      await setup(options, args as unknown[]);
    })
    .command("systemd", "Manage systemd units for this host")
    .alias("sys")
    .arguments("[action:string]")
    .action(async (_options, ...args) => {
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
    // Child command 2.
    .command("bar", "Bar sub-command.")
    .option("-b, --bar", "Bar option.")
    .arguments("<input:string> [output:string]")
    .action((_options, ..._args) => console.log("Bar command called."))
    .parse(Deno.args);
}
