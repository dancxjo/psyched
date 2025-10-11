import { defineConfig } from "vite";
import { fresh } from "@fresh/plugin-vite";
import preact from "@preact/preset-vite";

export default defineConfig({
  plugins: [
    preact({
      /**
       * Disable prefresh (Preact Fast Refresh) under Deno. The prefresh runtime
       * dynamically appends Vite cache-busting query parameters to `.deno`
       * virtual files which Deno's Node compat layer can't resolve, leading to
       * `os error 2` while serving the dev client. Disabling the runtime keeps
       * the dev server stable at the cost of losing automatic stateful hot
       * reloads; edits now trigger a full component reload instead of
       * preserving state.
       */
      prefreshEnabled: false,
    }),
    fresh(),
  ],
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
     *
     * Some packages (notably `@prefresh/utils`) ship files that are imported
     * with Vite's versioned "?v=" querystring suffix, which Deno's Node
     * compat layer can't resolve directly from the `.deno` cache. Prebundling
     * them forces Vite to serve the optimized copy from `.vite/deps`, avoiding
     * filesystem cache-busting suffixes that trip up Deno.
     */
    include: [
      "@preact/signals",
      "@preact/signals-core",
      "@prefresh/core",
      "@prefresh/utils",
    ],
  },
  ssr: {
    /**
     * Keep the runtime aligned between server and browser renders so Fresh
     * doesn't instantiate two copies of the signals runtime during hydration.
     */
    noExternal: [
      "@preact/signals",
      "@preact/signals-core",
      "@prefresh/core",
      "@prefresh/utils",
    ],
  },
});
