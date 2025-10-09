import type { ConnectionStatus } from "./cockpit.ts";

type Subscriber<T> = (value: T) => void;

export interface ReadonlySignal<T> {
  readonly value: T;
  peek(): T;
  subscribe(listener: Subscriber<T>): () => void;
  valueOf(): T;
  toString(): string;
  toJSON(): T;
}

export interface Signal<T> extends ReadonlySignal<T> {
  value: T;
}

class SimpleSignal<T> implements Signal<T> {
  #value: T;
  #listeners = new Set<Subscriber<T>>();

  constructor(initial: T) {
    this.#value = initial;
  }

  get value(): T {
    return this.#value;
  }

  set value(next: T) {
    if (Object.is(this.#value, next)) return;
    this.#value = next;
    this.#broadcast();
  }

  peek(): T {
    return this.#value;
  }

  subscribe(listener: Subscriber<T>): () => void {
    this.#listeners.add(listener);
    listener(this.#value);
    return () => {
      this.#listeners.delete(listener);
    };
  }

  valueOf(): T {
    return this.#value;
  }

  toString(): string {
    return String(this.#value);
  }

  toJSON(): T {
    return this.#value;
  }

  #broadcast(): void {
    for (const listener of this.#listeners) {
      try {
        listener(this.#value);
      } catch (error) {
        console.error("cockpit signal listener threw", error);
      }
    }
  }
}

class ComputedSignal<T> implements ReadonlySignal<T> {
  #compute: () => T;
  #value: T;
  #listeners = new Set<Subscriber<T>>();

  constructor(
    compute: () => T,
    dependencies: readonly ReadonlySignal<unknown>[],
  ) {
    this.#compute = compute;
    this.#value = compute();

    const refresh = () => {
      const next = this.#compute();
      if (Object.is(this.#value, next)) return;
      this.#value = next;
      this.#broadcast();
    };

    for (const dependency of dependencies) {
      dependency.subscribe(() => refresh());
    }
  }

  get value(): T {
    const next = this.#compute();
    if (!Object.is(this.#value, next)) {
      this.#value = next;
    }
    return this.#value;
  }

  peek(): T {
    return this.#value;
  }

  subscribe(listener: Subscriber<T>): () => void {
    this.#listeners.add(listener);
    listener(this.value);
    return () => {
      this.#listeners.delete(listener);
    };
  }

  valueOf(): T {
    return this.value;
  }

  toString(): string {
    return String(this.value);
  }

  toJSON(): T {
    return this.value;
  }

  #broadcast(): void {
    for (const listener of this.#listeners) {
      try {
        listener(this.#value);
      } catch (error) {
        console.error("cockpit computed signal listener threw", error);
      }
    }
  }
}

function createSignal<T>(initial: T): Signal<T> {
  return new SimpleSignal(initial);
}

function createComputed<T>(
  compute: () => T,
  dependencies: readonly ReadonlySignal<unknown>[],
): ReadonlySignal<T> {
  return new ComputedSignal(compute, dependencies);
}

const statusSignal = createSignal<ConnectionStatus>("idle");
const errorSignal = createSignal<string | null>(null);

/**
 * Reactive view of the cockpit websocket connection.
 *
 * Consumers can subscribe to {@link cockpitConnectionStatus} or
 * {@link cockpitConnectionError} to react to changes without prop drilling.
 *
 * @example
 * ```ts
 * import {
 *   cockpitConnectionStatus,
 *   cockpitIsConnected,
 * } from "@pilot/lib/cockpit_signals.ts";
 *
 * const unsubscribe = cockpitConnectionStatus.subscribe(() => {
 *   console.log("status", cockpitConnectionStatus.value);
 *   if (cockpitIsConnected.value) {
 *     console.log("connected!");
 *   }
 * });
 *
 * // ...later
 * unsubscribe();
 * ```
 */
export const cockpitConnectionStatus = statusSignal;
export const cockpitConnectionError = errorSignal;

export const cockpitIsConnected = createComputed(
  () => statusSignal.value === "open",
  [statusSignal],
);
export const cockpitIsConnecting = createComputed(
  () => statusSignal.value === "connecting",
  [statusSignal],
);
export const cockpitHasError = createComputed(
  () => statusSignal.value === "error" || errorSignal.value !== null,
  [statusSignal, errorSignal],
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
