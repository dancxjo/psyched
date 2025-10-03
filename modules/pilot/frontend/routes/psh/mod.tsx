import PshModuleManager from "../../islands/PshModuleManager.tsx";
import { define } from "../../utils.ts";

export default define.route(() => {
  return (
    <section class="content">
      <h1>Module Orchestration</h1>
      <p>
        Run <code>psh mod</code>{" "}
        operations directly from the cockpit. Actions execute on the server
        using the shared Deno modules.
      </p>
      <PshModuleManager />
    </section>
  );
});
