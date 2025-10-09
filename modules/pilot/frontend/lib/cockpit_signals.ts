import { computed, signal, type Signal } from "@preact/signals";
import type { ConnectionStatus } from "./cockpit.ts";

const statusSignal = signal<ConnectionStatus>("idle");
const errorSignal = signal<string | null>(null);

/**
 * Reactive view of the cockpit websocket connection.
 *
 * Consumers can subscribe to {@link cockpitConnectionStatus} or
 * {@link cockpitConnectionError} to react to changes without prop drilling.
 *
 * @example
 * ```ts
 * import { effect } from "@preact/signals";
 * import {
 *   cockpitConnectionStatus,
 *   cockpitIsConnected,
 * } from "@pilot/lib/cockpit_signals.ts";
 *
 * effect(() => {
 *   console.log("status", cockpitConnectionStatus.value);
 *   if (cockpitIsConnected.value) {
 *     console.log("connected!");
 *   }
 * });
 * ```
 */
export const cockpitConnectionStatus: Signal<ConnectionStatus> = statusSignal;
export const cockpitConnectionError: Signal<string | null> = errorSignal;

export const cockpitIsConnected = computed(() =>
  statusSignal.value === "open"
);
export const cockpitIsConnecting = computed(() =>
  statusSignal.value === "connecting"
);
export const cockpitHasError = computed(() =>
  statusSignal.value === "error" || errorSignal.value !== null
);

export function updateCockpitConnectionStatus(status: ConnectionStatus): void {
  statusSignal.value = status;
  if (status === "open") {
    errorSignal.value = null;
  }
}

export function recordCockpitConnectionError(message: string): void {
  errorSignal.value = message;
  statusSignal.value = "error";
}

export function clearCockpitConnectionError(): void {
  errorSignal.value = null;
}

export function resetCockpitSignals(): void {
  statusSignal.value = "idle";
  errorSignal.value = null;
}
