import {
  OVERLAY_ORDER_STORAGE_KEY,
  parseOverlayOrderJson,
  reconcileOverlayOrder,
  serialiseOverlayOrder,
} from "./persisted_order.ts";

Deno.test("overlay storage key exposes a stable identifier", () => {
  if (OVERLAY_ORDER_STORAGE_KEY.length === 0) {
    throw new Error("storage key should not be empty");
  }
});

Deno.test("parseOverlayOrderJson returns an empty list for invalid payloads", () => {
  const invalidInputs = [null, "{}", "\"foo\"", "[1,2,3]", "[\"\", null]"];
  for (const input of invalidInputs) {
    const result = parseOverlayOrderJson(input as string | null);
    if (result.length !== 0) {
      throw new Error(`Expected empty array for input ${input}, received ${result}`);
    }
  }
});

Deno.test("parseOverlayOrderJson recovers valid overlay keys", () => {
  const order = parseOverlayOrderJson('["alpha","beta","gamma"]');
  if (order.join(",") !== "alpha,beta,gamma") {
    throw new Error(`Unexpected overlay order: ${order}`);
  }
});

Deno.test("reconcileOverlayOrder drops missing overlays and preserves stored order", () => {
  const result = reconcileOverlayOrder(["alpha", "beta", "gamma"], ["gamma", "zeta", "alpha"]);
  if (result.join(",") !== "gamma,alpha,beta") {
    throw new Error(`Unexpected reconciliation result: ${result}`);
  }
});

Deno.test("reconcileOverlayOrder appends new overlays at the end", () => {
  const result = reconcileOverlayOrder(["alpha", "beta", "gamma"], []);
  if (result.join(",") !== "alpha,beta,gamma") {
    throw new Error(`Unexpected reconciliation result: ${result}`);
  }
});

Deno.test("serialiseOverlayOrder produces JSON arrays", () => {
  const result = serialiseOverlayOrder(["alpha", "beta"]);
  if (result !== '["alpha","beta"]') {
    throw new Error(`Unexpected serialised payload: ${result}`);
  }
});
