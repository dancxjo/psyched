import { Panel } from "@pilot/components/dashboard.tsx";
import PshServiceManager from "../../islands/PshServiceManager.tsx";
import { define } from "../../utils.ts";

export default define.route(() => {
  return (
    <section class="content">
      <Panel
        title="Service orchestration"
        subtitle="Manage Docker stacks with the familiar psh srv commands"
        accent="magenta"
      >
        <PshServiceManager />
      </Panel>
    </section>
  );
});
