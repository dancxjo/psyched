import config from "./fresh.config.ts";
import * as $0 from "./routes/_app.tsx";
import * as $1 from "./routes/index.tsx";
import * as $2 from "./routes/modules/foot.tsx";
import * as $3 from "./routes/modules/imu.tsx";
import * as $4 from "./routes/modules/pilot.tsx";
import * as $I0 from "./islands/FootControlPanelIsland.tsx";
import * as $I1 from "./islands/ImuTelemetryIsland.tsx";
import * as $I2 from "./islands/PilotOverviewIsland.tsx";
import { type Manifest } from "$fresh/server.ts";

const manifest: Manifest = {
    routes: {
        "./routes/_app.tsx": $0,
        "./routes/index.tsx": $1,
        "./routes/modules/foot.tsx": $2,
        "./routes/modules/imu.tsx": $3,
        "./routes/modules/pilot.tsx": $4,
    },
    islands: {
        "./islands/FootControlPanelIsland.tsx": $I0,
        "./islands/ImuTelemetryIsland.tsx": $I1,
        "./islands/PilotOverviewIsland.tsx": $I2,
    },
    baseUrl: import.meta.url,
};

export default manifest;
export { config };
