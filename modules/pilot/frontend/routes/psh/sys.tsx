import { page } from "fresh";
import PshSystemdManager from "../../islands/PshSystemdManager.tsx";
import { define } from "../../utils.ts";
import { listModules } from "../../../../../tools/psh/lib/module.ts";

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
      <h1>Systemd Integration</h1>
      <p>
        Generate and manage user-level systemd services that wrap module launch
        scripts. These commands map directly to <code>psh sys</code>.
      </p>
      <PshSystemdManager modules={modules} />
    </section>
  );
});
