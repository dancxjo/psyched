import config from "./fresh.config.ts";
import * as $0 from "./routes/_app.tsx";
import * as $1 from "./routes/index.tsx";
import * as $2 from "./routes/modules/imu.tsx";
import { type Manifest } from "$fresh/server.ts";

const manifest: Manifest = {
    routes: {
        "./routes/_app.tsx": $0,
        "./routes/index.tsx": $1,
        "./routes/modules/imu.tsx": $2,
    },
    islands: {
    },
    baseUrl: import.meta.url,
};

export default manifest;
export { config };
