import { $ } from "./util.ts";
import { colors } from "@cliffy/ansi/colors";

export async function printSummaryTable() {
    const { Table } = await import("@cliffy/table");
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
                } catch { /* ignore systemctl errors */ }
                units.push({ name: ent.name, active });
            }
        }
    } catch { /* ignore missing systemd dir */ }

    // Build summary table
    new Table()
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
        const header = [colors.bold("Status"), ...modules.map(mod => colors.bold(colors.cyan(mod)))];
        const enabledRow = [colors.bold("Enabled"), ...moduleStatus.map(([_, enabled, _r]) => enabled ? colors.green("Yes") : colors.red("No"))];
        const runningRow = [colors.bold("Running"), ...moduleStatus.map(([_, _e, running]) => running ? colors.green("Yes") : colors.red("No"))];
        new Table()
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
        new Table()
            .header(header)
            .body([statusRow])
            .border(true)
            .render();
    }

    // Friendly tip
    console.log(colors.gray("Tip: Use 'psh setup' to provision hosts and 'psh mod <module> launch' to start services."));
}
