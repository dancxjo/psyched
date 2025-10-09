import { assert, assertEquals } from "$std/testing/asserts.ts";
import {
  clearCockpitConnectionError,
  cockpitConnectionError,
  cockpitConnectionStatus,
  cockpitHasError,
  cockpitIsConnected,
  cockpitIsConnecting,
  recordCockpitConnectionError,
  resetCockpitSignals,
  updateCockpitConnectionStatus,
} from "./cockpit_signals.ts";

Deno.test("updates propagate to dependent computed signals", () => {
  resetCockpitSignals();
  const observed: Array<{ status: string; connected: boolean }> = [];
  const unsubscribe = cockpitConnectionStatus.subscribe(() => {
    observed.push({
      status: cockpitConnectionStatus.value,
      connected: cockpitIsConnected.value,
    });
  });
  updateCockpitConnectionStatus("connecting");
  updateCockpitConnectionStatus("open");
  unsubscribe();
  assertEquals(observed, [
    { status: "idle", connected: false },
    { status: "connecting", connected: false },
    { status: "open", connected: true },
  ]);
});

Deno.test("recording an error flags the computed helpers", () => {
  resetCockpitSignals();
  updateCockpitConnectionStatus("connecting");
  recordCockpitConnectionError("boom");
  assertEquals(cockpitConnectionStatus.value, "error");
  assert(cockpitHasError.value);
  assertEquals(cockpitConnectionError.value, "boom");
  clearCockpitConnectionError();
  assertEquals(cockpitConnectionError.value, null);
  assert(cockpitHasError.value);
});

Deno.test("reset restores idle state", () => {
  updateCockpitConnectionStatus("open");
  recordCockpitConnectionError("oops");
  resetCockpitSignals();
  assertEquals(cockpitConnectionStatus.value, "idle");
  assertEquals(cockpitConnectionError.value, null);
  assert(!cockpitIsConnected.value);
  assert(!cockpitIsConnecting.value);
  assert(!cockpitHasError.value);
});
