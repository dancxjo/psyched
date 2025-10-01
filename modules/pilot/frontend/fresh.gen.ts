import config from "./fresh.config.ts";
import * as $0 from "./routes/index.tsx";
import { type Manifest } from "$fresh/server.ts";

const manifest: Manifest = {
    routes: {
        "./routes/index.tsx": $0,
    },
    islands: {},
    baseUrl: import.meta.url,
};

export default manifest;
export { config };
