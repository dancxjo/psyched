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
     * Fresh runs through Vite inside a Deno workspace, so npm packages that
     * expose deep ESM entry points (like the Preact signals runtime) need to be
     * explicitly prebundled. Otherwise the dev server serves bare
     * `node_modules/.deno/...` paths that 404 in the browser.
     */
    include: ["@preact/signals", "@preact/signals-core"],
  },
  ssr: {
    /**
     * Keep the runtime aligned between server and browser renders so Fresh
     * doesn't instantiate two copies of the signals runtime during hydration.
     */
    noExternal: ["@preact/signals", "@preact/signals-core"],
  },
});
