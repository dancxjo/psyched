import { assertEquals } from "$std/assert/assert_equals.ts";
import { assertGreater } from "$std/assert/assert_greater.ts";
import { assert } from "$std/assert/assert.ts";

import { dashboardTiles, moduleTiles, serviceTiles } from "./tiles.ts";

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
