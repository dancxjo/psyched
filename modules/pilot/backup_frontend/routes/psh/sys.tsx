import type { Handlers } from "$fresh/server.ts";
import PshSystemdManager from "../../islands/PshSystemdManager.tsx";
import { define } from "../../utils.ts";
import { listModules } from "../../../../../tools/psh/lib/module.ts";

interface Data {
  modules: string[];
}

export const handler: Handlers<Data> = {
  GET(_req, ctx) {
    const modules = listModules();
    return ctx.render({ modules });
  },
};

export default define.route<Data>((_, ctx) => {
  const { modules } = ctx.data;
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
