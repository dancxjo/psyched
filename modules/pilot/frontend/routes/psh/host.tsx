import { page } from "fresh";
import PshHostProvisioner from "../../islands/PshHostProvisioner.tsx";
import { define } from "../../utils.ts";
import { availableHosts } from "../../lib/server/hosts.ts";

interface Data {
  hosts: string[];
}

export const handler = define.handlers<Data>({
  GET(_ctx) {
    const hosts = availableHosts();
    return page<Data>({ hosts });
  },
});

export default define.page<typeof handler>(({ data }) => {
  const { hosts } = data;
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
