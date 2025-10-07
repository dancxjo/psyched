import { defineConfig } from "vite";
import { fresh } from "@fresh/plugin-vite";
import { Server } from "http";

export default defineConfig({
  plugins: [fresh()],
  server: {
    allowedHosts: true
  }
});

