import "$std/dotenv/load.ts";

import { App, staticFiles } from "fresh";
import type { CockpitConfig, State } from "./utils.ts";

function readCockpitConfigFromEnv(): CockpitConfig | undefined {
  const sanitize = (value: string | undefined): string | undefined => {
    const trimmed = value?.trim();
    return trimmed ? trimmed : undefined;
  };

  const config: CockpitConfig = {
    url: sanitize(Deno.env.get("PILOT_COCKPIT_URL")),
    host: sanitize(Deno.env.get("PILOT_COCKPIT_HOST")),
    port: sanitize(Deno.env.get("PILOT_COCKPIT_PORT")),
    protocol: sanitize(Deno.env.get("PILOT_COCKPIT_PROTOCOL")),
  };

  const entries = Object.entries(config).filter(([, value]) =>
    value !== undefined
  );
  return entries.length
    ? Object.fromEntries(entries) as CockpitConfig
    : undefined;
}

export const app = new App<State>();

app.use(staticFiles());

app.use(async (ctx) => {
  if (!ctx.state.buildInfo) {
    ctx.state.buildInfo = {
      version: Deno.env.get("PILOT_VERSION") ??
        Deno.env.get("PSYCHED_BUILD_VERSION") ?? "dev",
    };
  }
  if (!ctx.state.cockpit) {
    const config = readCockpitConfigFromEnv();
    if (config) {
      ctx.state.cockpit = config;
    }
  }
  return await ctx.next();
});

app.fsRoutes();
