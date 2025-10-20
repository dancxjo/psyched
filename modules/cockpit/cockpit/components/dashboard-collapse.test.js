import { assertEquals, assert } from "https://deno.land/std@0.224.0/assert/mod.ts";

import {
  createCollapsedCardManager,
  loadCollapsedCardSet,
  normaliseCardId,
  persistCollapsedCardSet,
} from "./dashboard-collapse.js";

function createStorageStub(initial = new Map()) {
  const store = new Map(initial);
  return {
    getItem(key) {
      return store.has(key) ? store.get(key) : null;
    },
    setItem(key, value) {
      store.set(key, value);
    },
    dump() {
      return new Map(store);
    },
  };
}

Deno.test("loadCollapsedCardSet returns stored identifiers for the requested scope", () => {
  const storage = createStorageStub([
    ["psyched::cockpit::dashboard::collapsed::module-pilot", "[\"pilot-status\", \"pilot-scripts\"]"],
  ]);
  const collapsed = loadCollapsedCardSet("module-pilot", storage);
  assert(collapsed instanceof Set, "expected a Set to be returned");
  assertEquals([...collapsed].sort(), ["pilot-scripts", "pilot-status"]);
});

Deno.test("loadCollapsedCardSet ignores corrupt or non-array payloads", () => {
  const storage = createStorageStub([
    ["psyched::cockpit::dashboard::collapsed::module-pilot", "not json"],
  ]);
  const collapsed = loadCollapsedCardSet("module-pilot", storage);
  assertEquals(collapsed.size, 0);
});

Deno.test("persistCollapsedCardSet trims identifiers and removes duplicates", () => {
  const storage = createStorageStub();
  const saved = persistCollapsedCardSet("module-pilot", [" pilot-status ", "pilot-status", "pilot-scripts", ""], storage);
  assert(saved, "persist should return true when storage is available");
  const stored = storage.dump().get("psyched::cockpit::dashboard::collapsed::module-pilot");
  assertEquals(stored, "[\"pilot-status\",\"pilot-scripts\"]");
});

Deno.test("createCollapsedCardManager toggles and persists state", () => {
  const storage = createStorageStub();
  const manager = createCollapsedCardManager("module-pilot", storage);
  assertEquals(manager.isCollapsed("pilot-status"), false);
  const collapsedAfterSet = manager.setCollapsed("pilot-status", true);
  assertEquals([...collapsedAfterSet], ["pilot-status"]);
  assertEquals(manager.isCollapsed("pilot-status"), true);
  manager.toggle("pilot-status");
  assertEquals(manager.isCollapsed("pilot-status"), false);
  const stored = storage.dump().get("psyched::cockpit::dashboard::collapsed::module-pilot");
  assertEquals(stored, "[]");
});

Deno.test("normaliseCardId generates predictable slugs", () => {
  assertEquals(normaliseCardId(" Pilot Status "), "pilot-status");
  assertEquals(normaliseCardId("⚡️ Boost Mode"), "boost-mode");
  assertEquals(normaliseCardId(""), "");
});
