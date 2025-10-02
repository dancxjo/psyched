import { type Handlers, type PageProps } from "$fresh/server.ts";
import PilotOverview from "../../components/PilotOverview.tsx";

interface PilotPageData {
  version: string;
  description: string;
}

export const handler: Handlers<PilotPageData> = {
  GET(_req, ctx) {
    const buildInfo =
      (ctx.state as { buildInfo?: { version?: string } } | undefined)
        ?.buildInfo;
    const version =
      typeof buildInfo?.version === "string" && buildInfo.version.length > 0
        ? buildInfo.version
        : "dev";

    return ctx.render({
      version,
      description:
        "Home for the cockpit websocket bridge and Fresh frontend. Use this page to monitor connectivity and run quick health checks.",
    });
  },
};

export default function PilotPage({ data }: PageProps<PilotPageData>) {
  return (
    <section class="content">
      <PilotOverview version={data.version} description={data.description} />
    </section>
  );
}
