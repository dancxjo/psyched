import type { Handlers } from "$fresh/server.ts";
import PshHostProvisioner from "../../islands/PshHostProvisioner.tsx";
import { define } from "../../utils.ts";
import { availableHosts } from "../../../../../tools/psh/lib/host.ts";

interface Data {
  hosts: string[];
}

export const handler: Handlers<Data> = {
  GET(_req, ctx) {
    const hosts = availableHosts();
    return ctx.render({ hosts });
  },
};

export default define.route<Data>((_, ctx) => {
  const { hosts } = ctx.data;
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
