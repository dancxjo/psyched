import {
  defineApp,
  defineConfig,
  defineLayout,
  defineRoute,
} from "$fresh/server.ts";

export const define = {
  app: defineApp,
  layout: defineLayout,
  route: defineRoute,
  page: defineRoute,
  config: defineConfig,
};

export type DefineHelpers = typeof define;
