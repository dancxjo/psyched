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
     * Ensure the dev server prebundles the Preact signals runtime so requests
     * for `signals-core.module.js` resolve correctly in Deno workspaces.
     */
    include: ["@preact/signals", "@preact/signals-core"],
  },
  ssr: {
    /**
     * Keep the runtime aligned between server and browser renders so Vite
     * doesn't attempt to resolve the npm bundle twice during hydration.
     */
    noExternal: ["@preact/signals", "@preact/signals-core"],
  },
});
