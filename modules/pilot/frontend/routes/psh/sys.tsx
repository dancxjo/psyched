import { Panel } from "@pilot/components/dashboard.tsx";
import { page } from "fresh";
import PshSystemdManager from "../../islands/PshSystemdManager.tsx";
import { define } from "../../utils.ts";
import { listModules } from "../../lib/server/modules.ts";

interface Data {
  modules: string[];
}

export const handler = define.handlers<Data>({
  GET(_ctx) {
    const modules = listModules();
    return page<Data>({ modules });
  },
});

export default define.page<typeof handler>(({ data }) => {
  const { modules } = data;
  return (
    <section class="content">
      <Panel
        title="Systemd integration"
        subtitle="Bridge module launch scripts to persistent user units"
        accent="cyan"
      >
        <PshSystemdManager modules={modules} />
      </Panel>
    </section>
  );
});
