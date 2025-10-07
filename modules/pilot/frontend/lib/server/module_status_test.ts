import { assertEquals } from "$std/assert/mod.ts";
import { join } from "$std/path/mod.ts";
import { moduleStatuses } from "./module_status.ts";

Deno.test("returns running and stopped module statuses", () => {
  const tempDir = Deno.makeTempDirSync();
  try {
    const modulesDir = join(tempDir, "modules");
    const workspaceDir = join(tempDir, "work");
    Deno.mkdirSync(modulesDir, { recursive: true });
    Deno.mkdirSync(workspaceDir, { recursive: true });
    Deno.mkdirSync(join(workspaceDir, ".psh"), { recursive: true });

    // Create two modules to exercise both branches.
    Deno.mkdirSync(join(modulesDir, "alpha"));
    Deno.mkdirSync(join(modulesDir, "bravo"));

    // alpha is running: record the current process PID.
    const runningPid = Deno.pid;
    Deno.writeTextFileSync(
      join(workspaceDir, ".psh", "alpha.pid"),
      `${runningPid}\n`,
    );

    // bravo has a stale PID that should be cleared.
    Deno.writeTextFileSync(join(workspaceDir, ".psh", "bravo.pid"), "999999\n");

    const statuses = moduleStatuses({
      repoRoot: tempDir,
      modulesRoot: modulesDir,
      workspaceRoot: workspaceDir,
    });

    assertEquals(statuses, [
      { name: "alpha", status: "running", pid: runningPid },
      { name: "bravo", status: "stopped" },
    ]);

    // The stale PID should be removed to avoid future false positives.
    let removed = false;
    try {
      Deno.statSync(join(workspaceDir, ".psh", "bravo.pid"));
    } catch (error) {
      if (error instanceof Deno.errors.NotFound) {
        removed = true;
      } else {
        throw error;
      }
    }
    assertEquals(removed, true);
  } finally {
    Deno.removeSync(tempDir, { recursive: true });
  }
});
