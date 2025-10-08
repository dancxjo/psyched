import { defineConfig } from "vite";
import { fresh } from "@fresh/plugin-vite";

export default defineConfig({
  plugins: [fresh()],
  server: {
    host: "0.0.0.0",
    allowedHosts: true,
  },
});
