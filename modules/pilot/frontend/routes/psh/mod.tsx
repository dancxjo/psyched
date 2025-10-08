import { Panel } from "@pilot/components/dashboard.tsx";
import PshModuleManager from "../../islands/PshModuleManager.tsx";
import { define } from "../../utils.ts";

export default define.route(() => {
  return (
    <section class="content">
      <Panel
        title="Module orchestration"
        subtitle="Execute psh mod lifecycle commands from the cockpit"
        accent="amber"
      >
        <PshModuleManager />
      </Panel>
    </section>
  );
});
