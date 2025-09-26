import { assertEquals } from "@std/assert";
import { getHostModules } from "./setup.ts";
import { createCli } from "./cli.ts";
import { repoDirFromModules } from "./modules.ts";
import { join } from "@std/path";

Deno.test("getHostModules returns modules declared in host toml", async () => {
    const repo = repoDirFromModules();
    const hostsDir = join(repo, "hosts");
    await Deno.mkdir(hostsDir, { recursive: true });
    const tomlPath = join(hostsDir, "unittesthost.toml");
    const toml = `modules = ["alpha", "beta"]\n`;
    await Deno.writeTextFile(tomlPath, toml);
    const mods = await getHostModules("unittesthost");
    assertEquals(mods, ["alpha", "beta"]);
    await Deno.remove(tomlPath);
});

Deno.test("`psh mod setup` uses host modules when none provided", async () => {
    // Create host toml
    const repo = repoDirFromModules();
    const tomlPath = join(repo, "hosts", "cli-test-host.toml");
    await Deno.mkdir(join(repo, "hosts"), { recursive: true });
    await Deno.writeTextFile(tomlPath, `modules = ["x", "y"]\n`);

    // Capture runModuleScript calls by overriding deps when creating CLI
    let capturedModules: string[] | string | null = null;
    const cli = createCli({
        runModuleScript: (modules: string[] | string, _action?: string) => {
            capturedModules = modules;
        },
    });

    // Invoke the module command with no modules (defaults should be read)
    const prevHost = Deno.env.get("HOST");
    try {
        Deno.env.set("HOST", "cli-test-host");
        await cli.parse(["module", "setup"]);
    } finally {
        if (prevHost === undefined) Deno.env.delete("HOST"); else Deno.env.set("HOST", prevHost);
    }

    // Expect capturedModules to be the host-declared modules
    assertEquals(capturedModules, ["x", "y"]);

    await Deno.remove(tomlPath).catch(() => undefined);
});
