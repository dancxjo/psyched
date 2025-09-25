// Systemd management functions
import { repoPath, $ } from "./util.ts";

export async function systemdGenerate(): Promise<void> {
    const script = repoPath('../tools/systemd_generate');
    console.log(`Running systemd_generate -> ${script}`);
    const r = await $`bash ${script}`;
    if (r.code !== 0) {
        console.error(r.stderr || r.stdout);
        throw new Error(`systemd_generate failed (${r.code})`);
    }
}

export async function systemdInstall(): Promise<void> {
    await systemdGenerate();
    const hn = await $`hostname -s`;
    const host = (hn.stdout || hn.stderr || "").toString().trim() || Deno.env.get('HOST') || "$(hostname)";
    const unitsDir = `${Deno.cwd()}/hosts/${host}/systemd`;
    try {
        const stat = await Deno.stat(unitsDir);
        if (!stat.isDirectory) throw new Error('not dir');
    } catch (_err) {
        throw new Error(`No generated unit directory at ${unitsDir}`);
    }
    console.log(`Installing unit files from ${unitsDir} -> /etc/systemd/system`);
    for await (const ent of Deno.readDir(unitsDir)) {
        if (!ent.name.endsWith('.service')) continue;
        const src = `${unitsDir}/${ent.name}`;
        const cp = await $`sudo cp ${src} /etc/systemd/system/${ent.name}`;
        if (cp.code !== 0) {
            throw new Error(`Failed to copy ${ent.name}: ${cp.stderr || cp.stdout}`);
        }
        const enable = await $`sudo systemctl enable --now ${ent.name}`;
        if (enable.code !== 0) {
            console.error(`Failed to enable ${ent.name}:`, enable.stderr || enable.stdout);
        }
        console.log(`Installed and enabled ${ent.name}`);
    }
    const reload = await $`sudo systemctl daemon-reload`;
    if (reload.code !== 0) {
        throw new Error(`Failed to reload systemd daemon: ${reload.stderr || reload.stdout}`);
    }
    console.log('Systemd units installed and daemon reloaded.');
}

export async function systemdUninstall(): Promise<void> {
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
        const disable = await $`sudo systemctl disable --now ${name}`;
        if (disable.code !== 0) {
            console.log(`Warning: failed to disable ${name}:`, disable.stderr || disable.stdout);
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
