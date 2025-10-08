import {
  MODULE_ACTIONS,
  moduleActionEndpoint,
  moduleActionLabel,
} from "./module_actions.ts";

Deno.test("module action endpoints map to psh mod APIs", () => {
  for (const action of MODULE_ACTIONS) {
    const endpoint = moduleActionEndpoint(action);
    if (!endpoint.startsWith("/api/psh/mod/")) {
      throw new Error(`Unexpected endpoint prefix for ${action}: ${endpoint}`);
    }
    const suffix = endpoint.replace("/api/psh/mod/", "");
    if (suffix !== action) {
      throw new Error(`Endpoint suffix mismatch for ${action}: ${suffix}`);
    }
  }
});

Deno.test("module action labels are human friendly", () => {
  const expected = new Map(
    [
      ["setup", "Setup"],
      ["up", "Start"],
      ["down", "Stop"],
      ["teardown", "Teardown"],
    ] as const,
  );
  for (const action of MODULE_ACTIONS) {
    const label = moduleActionLabel(action);
    if (label !== expected.get(action)) {
      throw new Error(`Unexpected label for ${action}: ${label}`);
    }
  }
});
