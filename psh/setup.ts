
// --- Utility Imports & Helpers ---
import { $, repoPath } from "./util.ts";
import { parse as parseToml } from "@std/toml";

// --- Installers ---
async function runInstallRos2(): Promise<void> {
    const script = repoPath("../tools/install_ros2.sh");
    console.log(`Running ROS2 install script: ${script}`);
    const r = await $`bash ${script}`;
    if (r.code !== 0) {
        console.error(r.stderr || r.stdout);
        throw new Error(`ROS2 installer failed (${r.code})`);
    }
}

async function runInstallDocker(): Promise<void> {
    const script = repoPath("../tools/install_docker.sh");
    console.log(`Running Docker install script: ${script}`);
    const r = await $`bash ${script}`;
    if (r.code !== 0) {
        console.error(r.stderr || r.stdout);
        throw new Error(`Docker installer failed (${r.code})`);
    }
}

// --- Module Setup ---
async function runModuleSetup(moduleName: string, config: Record<string, unknown> | null): Promise<void> {
    const script = `${Deno.cwd()}/modules/${moduleName}/setup.sh`;
    try {
        const stat = await Deno.stat(script);
        if (!stat.isFile) throw new Error("not a file");
    } catch {
        console.warn(`No setup script for module '${moduleName}' at ${script}; skipping.`);
        return;
    }

    // Write config to temp file if present
    let configPath: string | null = null;
    if (config && Object.keys(config).length > 0) {
        configPath = `/tmp/psh_${moduleName}_config.${Deno.pid}.json`;
        await Deno.writeTextFile(configPath, JSON.stringify(config));
    }

    console.log(`Running module setup: ${moduleName}` + (configPath ? ` (config: ${configPath})` : ""));
    const r = configPath ? await $`bash ${script} ${configPath}` : await $`bash ${script}`;
    if (r.code !== 0) {
        console.error(`Module ${moduleName} setup failed:`, r.stderr || r.stdout);
        throw new Error(`Module ${moduleName} setup failed (${r.code})`);
    }

    if (configPath) {
        try { await Deno.remove(configPath); } catch { /* ignore */ }
    }
}

// --- Main Setup Flow ---
export async function setup(_options: Record<string, unknown>, args: unknown[]): Promise<void> {
    // Determine host name
    const hostArg = args?.[0]?.toString();
    let host = hostArg;
    if (!host) {
        const hn = await $`hostname -s`.stdout("piped").stderr("piped");
        host = (hn.stdout || hn.stderr || "").toString().trim() || Deno.env.get("HOST") || undefined;
    }
    if (!host) {
        console.error("Unable to determine host name. Provide it as `psh setup <host>` or set HOST env var.");
        Deno.exit(2);
    }

    // Read host TOML
    const tomlPath = `${Deno.cwd()}/hosts/${host}.toml`;
    let tomlText: string;
    try {
        tomlText = await Deno.readTextFile(tomlPath);
    } catch (err) {
        console.error(`Host TOML not found at ${tomlPath}:`, String(err));
        Deno.exit(2);
    }

    // Parse TOML
    let spec: Record<string, unknown>;
    try {
        spec = parseToml(tomlText) as Record<string, unknown>;
    } catch (err) {
        console.error(`Failed to parse TOML for host ${host}:`, String(err));
        Deno.exit(2);
    }

    console.log(`Applying host '${host}'`);

    // Optional installers
    try {
        if (spec?.setup_ros2) {
            console.log("Host requests ROS2 installation.");
            await runInstallRos2();
        }
        if (spec?.setup_docker) {
            console.log("Host requests Docker installation.");
            await runInstallDocker();
        }
    } catch (err) {
        console.error("Installer failed:", String(err));
        Deno.exit(3);
    }

    // Modules activation
    const modulesList = Array.isArray(spec.modules) ? spec.modules as string[] : [];
    if (!modulesList.length) {
        console.log("No modules listed for this host in TOML. Nothing more to do.");
        return;
    }

    // Extract module-specific config tables
    const modulesConfigTable = (spec.modules && typeof spec.modules === "object") ? spec.modules as Record<string, unknown> : {};

    for (const moduleName of modulesList) {
        const moduleConfig = modulesConfigTable && typeof modulesConfigTable[moduleName] === "object"
            ? modulesConfigTable[moduleName] as Record<string, unknown>
            : null;
        try {
            await runModuleSetup(moduleName, moduleConfig);
        } catch (err) {
            console.error(`Failed to setup module ${moduleName}:`, String(err));
            // Continue with other modules
        }
    }

    console.log("Host setup finished.");
}
