import { assertEquals } from "$std/testing/asserts.ts";
import { buildHostPowerRule, HOST_POWER_ACTIONS } from "./host_power.ts";

Deno.test("buildHostPowerRule generates a stable polkit rule", () => {
  const expected = [
    "/* Psyched host power privileges */",
    "polkit.addRule(function(action, subject) {",
    "  const allowed = [",
    ...HOST_POWER_ACTIONS.map((id) => `    "${id}",`),
    "  ];",
    '  if (allowed.indexOf(action.id) !== -1 && subject.isInGroup("psyched-power")) {',
    "    return polkit.Result.YES;",
    "  }",
    "});",
    "",
  ].join("\n");
  assertEquals(buildHostPowerRule("psyched-power"), expected);
});
