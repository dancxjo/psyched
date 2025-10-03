import PshHostProvisioner from "../../islands/PshHostProvisioner.tsx";
import { define } from "../../utils.ts";
import { availableHosts } from "../../../../../tools/psh/lib/host.ts";

export const handler = {
  GET(_req: Request, ctx: any) {
    const hosts = availableHosts();
    return ctx.render({ hosts });
  },
};

export default define.route((_, ctx) => {
  const { hosts } = ctx.data as { hosts: string[] };
  return (
    <section class="content">
      <h1>Host Setup Wizard</h1>
      <p>
        Choose a host profile and trigger{" "}
        <code>psh host setup</code>. The server reuses the same provisioning
        routines as the CLI wizard.
      </p>
      <PshHostProvisioner hosts={hosts} />
    </section>
  );
});
