import { assertEquals } from "$std/assert/assert_equals.ts";
import { assertGreater } from "$std/assert/assert_greater.ts";
import { assert } from "$std/assert/assert.ts";
import { join } from "$std/path/mod.ts";

import {
  dashboardTiles,
  moduleTiles,
  moduleTilesForHost,
  serviceTiles,
  serviceTilesForHost,
} from "./tiles.ts";

Deno.test("module tiles have unique names", () => {
  const names = moduleTiles.map((tile) => tile.name);
  const unique = new Set(names);
  assertEquals(unique.size, names.length);
});

Deno.test("service tiles have unique names", () => {
  const names = serviceTiles.map((tile) => tile.name);
  const unique = new Set(names);
  assertEquals(unique.size, names.length);
});

Deno.test("dashboard tiles expose overlays", () => {
  assertGreater(moduleTiles.length, 0);
  assertGreater(serviceTiles.length, 0);
  for (const tile of dashboardTiles) {
    assert(typeof tile.title === "string" && tile.title.length > 0);
    assert(typeof tile.href === "string" && tile.href.startsWith("/"));
    assertEquals(typeof tile.overlay, "function");
  }
});

Deno.test("moduleTilesForHost returns tiles for enabled modules", () => {
  const tempDir = Deno.makeTempDirSync();
  try {
    const modulesDir = join(tempDir, "modules");
    const hostsDir = join(tempDir, "hosts");
    Deno.mkdirSync(modulesDir, { recursive: true });
    Deno.mkdirSync(hostsDir, { recursive: true });

    for (const name of ["chat", "imu", "memory"]) {
      Deno.mkdirSync(join(modulesDir, name));
    }

    const manifest = {
      host: { modules: ["imu", "chat"], services: ["asr"] },
      modules: {
        chat: { launch: true },
        imu: { launch: { enabled: true } },
      },
      services: {
        asr: {},
      },
    };
    Deno.writeTextFileSync(
      join(hostsDir, "dash.json"),
      JSON.stringify(manifest, null, 2),
    );

    const tiles = moduleTilesForHost({
      hostname: "dash",
      hostsDir,
      modulesDir,
    });

    assertEquals(tiles.map((tile) => tile.name).sort(), ["chat", "imu"]);

    const serviceTilesForHostResult = serviceTilesForHost({
      hostname: "dash",
      hostsDir,
    });
    assertEquals(serviceTilesForHostResult.map((tile) => tile.name), ["asr"]);
  } finally {
    Deno.removeSync(tempDir, { recursive: true });
  }
});
