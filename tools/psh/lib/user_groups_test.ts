import { assertEquals } from "$std/testing/asserts.ts";
import {
  computeMissingGroups,
  detectProvisionUser,
  essentialGroups,
  parseGroupList,
} from "./user_groups.ts";

Deno.test("essentialGroups merges defaults and extras without duplicates", () => {
  const result = essentialGroups(["dialout", "spi", "SPI"]);
  assertEquals(result.sort(), [
    "audio",
    "dialout",
    "gpio",
    "i2c",
    "plugdev",
    "render",
    "spi",
    "video",
  ].sort());
});

Deno.test("parseGroupList normalizes whitespace", () => {
  const result = parseGroupList("audio video\nplugdev");
  assertEquals(result.sort(), ["audio", "video", "plugdev"].sort());
});

Deno.test("computeMissingGroups identifies absent memberships", () => {
  const missing = computeMissingGroups(["audio", "video"], ["audio", "i2c", "video"]);
  assertEquals(missing, ["i2c"]);
});

Deno.test("detectProvisionUser prefers non-root environment candidates", () => {
  const user = detectProvisionUser({
    env: {
      SUDO_USER: "robot",
      USER: "root",
      LOGNAME: "root",
    },
  });
  assertEquals(user, "robot");
});

Deno.test("detectProvisionUser falls back to any defined candidate", () => {
  const user = detectProvisionUser({
    env: {
      USER: "root",
      LOGNAME: "maintainer",
    },
  });
  assertEquals(user, "maintainer");
});
