import "$std/dotenv/load.ts";

import { App, staticFiles } from "fresh";
import type { State } from "./utils.ts";

export const app = new App<State>();

app.use(staticFiles());

app.use(async (ctx) => {
  if (!ctx.state.buildInfo) {
    ctx.state.buildInfo = {
      version: Deno.env.get("PILOT_VERSION") ??
        Deno.env.get("PSYCHED_BUILD_VERSION") ?? "dev",
    };
  }
  return await ctx.next();
});

app.fsRoutes();
