/**
 * Lifecycle actions available for ROS modules managed through `psh mod`.
 */
export const MODULE_ACTIONS = [
  "setup",
  "up",
  "down",
  "teardown",
] as const;

export type ModuleAction = (typeof MODULE_ACTIONS)[number];

const MODULE_ACTION_LABELS: Readonly<Record<ModuleAction, string>> = {
  setup: "Setup",
  up: "Start",
  down: "Stop",
  teardown: "Teardown",
} as const;

/**
 * Resolve the cockpit API endpoint responsible for executing a module action.
 *
 * @example
 * ```ts
 * moduleActionEndpoint("up"); // "/api/psh/mod/up"
 * ```
 */
export function moduleActionEndpoint(action: ModuleAction): string {
  return `/api/psh/mod/${action}`;
}

/** Human friendly label for a module lifecycle action. */
export function moduleActionLabel(action: ModuleAction): string {
  return MODULE_ACTION_LABELS[action] ?? action;
}
