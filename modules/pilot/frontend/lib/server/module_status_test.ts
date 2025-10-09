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

Deno.test("moduleStatuses filters modules using host manifest when provided", () => {
  const tempDir = Deno.makeTempDirSync();
  try {
    const modulesDir = join(tempDir, "modules");
    const workspaceDir = join(tempDir, "work");
    const hostsDir = join(tempDir, "hosts");
    Deno.mkdirSync(modulesDir, { recursive: true });
    Deno.mkdirSync(workspaceDir, { recursive: true });
    Deno.mkdirSync(hostsDir, { recursive: true });
    Deno.mkdirSync(join(workspaceDir, ".psh"), { recursive: true });

    for (const name of ["alpha", "bravo", "charlie"]) {
      Deno.mkdirSync(join(modulesDir, name));
    }

    const config = {
      host: { modules: ["alpha"] },
      modules: {
        alpha: { launch: true },
        bravo: { launch: false },
        charlie: { launch: { enabled: true } },
      },
    };
    Deno.writeTextFileSync(
      join(hostsDir, "test-host.json"),
      JSON.stringify(config, null, 2),
    );

    const statuses = moduleStatuses({
      repoRoot: tempDir,
      modulesRoot: modulesDir,
      workspaceRoot: workspaceDir,
      hostsDir,
      hostname: "test-host",
    });

    assertEquals(statuses, [
      { name: "alpha", status: "stopped" },
      { name: "charlie", status: "stopped" },
    ]);
  } finally {
    Deno.removeSync(tempDir, { recursive: true });
  }
});
