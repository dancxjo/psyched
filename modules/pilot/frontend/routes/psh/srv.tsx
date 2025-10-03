import PshServiceManager from "../../islands/PshServiceManager.tsx";
import { define } from "../../utils.ts";

export default define.route(() => {
  return (
    <section class="content">
      <h1>Service Orchestration</h1>
      <p>
        These controls invoke <code>psh srv</code>{" "}
        commands to manage Docker-based services. Status updates are pulled live
        from the same backend helpers.
      </p>
      <PshServiceManager />
    </section>
  );
});
