import { defineConfig } from "vite";
import { fresh } from "@fresh/plugin-vite";
import preact from "@preact/preset-vite";

export default defineConfig({
  plugins: [preact(), fresh()],
  server: {
    host: "0.0.0.0",
    allowedHosts: true,
  },
  optimizeDeps: {
    /**
     * The Preact signals packages rely on npm-style resolution. When Fresh runs
     * through Vite in a Deno workspace we need to explicitly prebundle them so
     * the dev server serves `signals-core.module.js` instead of 404ing the
     * request. This mirrors the guidance from the Vite team for npm packages
     * with deep ESM entry points.
     */
    include: ["@preact/signals", "@preact/signals-core"],
  },
  ssr: {
    /**
     * Keep the runtime aligned between server and browser renders to avoid
     * double-instantiating the signals runtime when Fresh hydrates islands.
     */
    noExternal: ["@preact/signals", "@preact/signals-core"],
  },
});
