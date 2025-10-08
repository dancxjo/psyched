import type {
  Handlers,
  MiddlewareHandler,
  RouteConfig,
} from "$fresh/server.ts";
import type { NavigationLink } from "./lib/navigation_types.ts";

/**
 * Connection details for the cockpit websocket bridge that powers the pilot UI.
 *
 * Any of the fields can be omitted when the value is unknown. The helpers in
 * this module automatically propagate the shape through Fresh so components and
 * route handlers can consume the data without bespoke type assertions.
 *
 * @example
 * ```ts
 * const cockpit: CockpitConfig = { host: "localhost", port: "8088" };
 * ```
 */
export interface CockpitConfig {
  host?: string;
  port?: string;
  protocol?: string;
  url?: string;
}

/**
 * Metadata describing the build that produced the current frontend bundle.
 *
 * The information is injected by the bootstrap middleware defined in
 * {@link ../main.ts | main.ts}.
 */
export interface BuildInfo {
  version?: string;
  commit?: string;
  timestamp?: string;
}

/**
 * Shared Fresh state that flows between middleware, handlers, and pages.
 */
export interface State {
  buildInfo?: BuildInfo;
  cockpit?: CockpitConfig;
  navigation?: NavigationLink[];
}

/**
 * Helper surface that mirrors Fresh's `define*` utilities while constraining
 * the state type used across the cockpit.
 */
export type DefineHelpers = {
  app: <T extends MiddlewareHandler<State>>(middleware: T) => T;
  layout: <T extends MiddlewareHandler<State>>(layout: T) => T;
  route: <T>(handler: T) => T;
  page: <T>(component: T) => T;
  handlers: <Data>(handlers: Handlers<Data, State>) => Handlers<Data, State>;
  config: <T extends RouteConfig>(config: T) => T;
};

const identity = <T>(value: T): T => value;

/**
 * Typed identity wrappers that make it easy to declare routes, layouts, and
 * API handlers without repeatedly importing Fresh generics.
 */
export const define: DefineHelpers = {
  app: identity,
  layout: identity,
  route: identity,
  page: identity,
  handlers: identity,
  config: identity,
};
