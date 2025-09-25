// --- Utility Imports & Helpers ---
import { repoPath } from "./util.ts";

// Clean workspace directories: src, install, build
export async function clean(): Promise<void> {
    const dirs = ["src", "install", "build"];
    for (const dir of dirs) {
        const absPath = repoPath(`../${dir}`);
        try {
            await Deno.remove(absPath, { recursive: true });
            console.log(`Removed: ${absPath}`);
        } catch (err) {
            if (err instanceof Deno.errors.NotFound) {
                console.log(`Not found: ${absPath}`);
            } else {
                console.error(`Failed to remove ${absPath}:`, String(err));
            }
        }
    }
    console.log("Workspace clean complete.");
}
