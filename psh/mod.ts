// mod.ts - Module script runner for psh

/**
 * Run a module script (setup, launch, etc) for a given module.
 * If action is omitted, tries launch/start/up in order.
 */
export async function runModuleScript(module: string, action?: string) {
    const actions = [
        action,
        ...(action ? [] : ["launch", "start", "up"])
    ].filter(Boolean);
    for (const act of actions) {
        const scriptPath = `${Deno.cwd()}/modules/${module}/${act}.sh`;
        try {
            const stat = await Deno.stat(scriptPath);
            if (stat.isFile) {
                console.log(`[mod] Running: ${scriptPath}`);
                const cmd = new Deno.Command("bash", {
                    args: [scriptPath],
                    stdout: "inherit",
                    stderr: "inherit"
                });
                const status = await cmd.spawn().status;
                if (!status.success) {
                    console.error(`[mod] Script failed: ${scriptPath}`);
                    Deno.exit(status.code);
                }
                return;
            }
        } catch { /* not found */ }
    }
    console.error(`[mod] No script found for module '${module}' action '${action || "(default)"}'`);
    Deno.exit(2);
}
