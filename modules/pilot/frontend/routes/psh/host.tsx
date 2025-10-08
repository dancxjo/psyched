import { page } from "fresh";
import { Panel } from "@pilot/components/dashboard.tsx";
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
      <Panel
        title="Host setup"
        subtitle="Run psh host setup without leaving the cockpit"
        accent="teal"
      >
        <PshHostProvisioner hosts={hosts} />
      </Panel>
    </section>
  );
});
