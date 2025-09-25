import { repoPath } from "./util.ts";

export async function colconBuild(): Promise<void> {
    const absSrc = repoPath("../src");
    const cmd = new Deno.Command("colcon", {
        args: ["build", "--symlink-install", "--base-paths", absSrc],
        stdout: "inherit",
        stderr: "inherit",
    });
    const child = cmd.spawn();
    const status = await child.status;
    if (!status.success) {
        console.error(`[psh] colcon build failed with code ${status.code ?? 1}`);
        Deno.exit(status.code ?? 1);
    }
    console.log("Colcon build complete.");
}

export async function colconInstall(): Promise<void> {
    const absSrc = repoPath("../src");
    const cmd = new Deno.Command("colcon", {
        args: ["install", "--base-paths", absSrc],
        stdout: "inherit",
        stderr: "inherit",
    });
    const child = cmd.spawn();
    const status = await child.status;
    if (!status.success) {
        console.error(`[psh] colcon install failed with code ${status.code ?? 1}`);
        Deno.exit(status.code ?? 1);
    }
    console.log("Colcon install complete.");
}
