import { assertEquals } from "$std/assert/assert_equals.ts";
import { join } from "$std/path/mod.ts";

import { listModules } from "./modules.ts";

Deno.test("listModules filters by host manifest", () => {
  const tempDir = Deno.makeTempDirSync();
  try {
    const modulesDir = join(tempDir, "modules");
    const hostsDir = join(tempDir, "hosts");
    Deno.mkdirSync(modulesDir, { recursive: true });
    Deno.mkdirSync(hostsDir, { recursive: true });

    for (const name of ["alpha", "bravo", "charlie", "pilot"]) {
      Deno.mkdirSync(join(modulesDir, name));
    }

    const hostConfig = {
      host: { modules: ["alpha", "pilot"] },
      modules: {
        alpha: { launch: { enabled: true } },
        bravo: { launch: { enabled: false } },
        charlie: { launch: { arguments: { enabled: true } } },
      },
    };
    Deno.writeTextFileSync(
      join(hostsDir, "test-host.json"),
      JSON.stringify(hostConfig, null, 2),
    );

    const modules = listModules({
      hostname: "test-host",
      hostsDir,
      modulesDir,
    });

    assertEquals(modules, ["alpha", "charlie"]);
  } finally {
    Deno.removeSync(tempDir, { recursive: true });
  }
});

Deno.test("listModules falls back to scanning module directories", () => {
  const tempDir = Deno.makeTempDirSync();
  try {
    const modulesDir = join(tempDir, "modules");
    const hostsDir = join(tempDir, "hosts");
    Deno.mkdirSync(modulesDir, { recursive: true });
    Deno.mkdirSync(hostsDir, { recursive: true });

    for (const name of ["alpha", "bravo", "pilot"]) {
      Deno.mkdirSync(join(modulesDir, name));
    }

    const modules = listModules({
      hostname: "ghost", // no manifest written
      hostsDir,
      modulesDir,
    });

    assertEquals(modules, ["alpha", "bravo"]);
  } finally {
    Deno.removeSync(tempDir, { recursive: true });
  }
});
