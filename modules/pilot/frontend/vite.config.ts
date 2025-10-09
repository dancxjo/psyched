import { defineConfig } from "vite";
import { fresh } from "@fresh/plugin-vite";
import preact from "@preact/preset-vite";

export default defineConfig({
  plugins: [preact(), fresh()],
  server: {
    host: "0.0.0.0",
    allowedHosts: true,
  },
});
