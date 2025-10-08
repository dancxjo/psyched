import PilotOverview from "../../islands/PilotOverviewIsland.tsx";
import { define } from "../../utils.ts";
import { RouteContext } from "fresh/compat";

export default define.route((_req: Request, ctx: RouteContext) => {
  const buildInfo =
    (ctx.state as { buildInfo?: { version?: string } } | undefined)?.buildInfo;
  const version =
    typeof buildInfo?.version === "string" && buildInfo.version.length > 0
      ? buildInfo.version
      : "dev";
  const description =
    "Home for the cockpit websocket bridge and Fresh frontend. Use this page to monitor connectivity and run quick health checks.";

  return (
    <section class="content">
      <PilotOverview version={version} description={description} />
    </section>
  );
});
