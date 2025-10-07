import { define } from "../../../../utils.ts";
import {
  bringModulesUp,
  listModules,
} from "../../../../../../../tools/psh/lib/module.ts";

interface RequestBody {
  modules?: string[];
}

export const handler = define.handlers({
  async POST(ctx) {
    const body = await ctx.req.json().catch(() =>
      ({}) as RequestBody
    ) as RequestBody;
    const modules = body.modules?.length ? body.modules : listModules();

    try {
      await bringModulesUp(modules);
      return new Response(JSON.stringify({ ok: true, modules }), {
        status: 200,
        headers: { "content-type": "application/json" },
      });
    } catch (error) {
      return new Response(JSON.stringify({ ok: false, error: String(error) }), {
        status: 500,
        headers: { "content-type": "application/json" },
      });
    }
  },
});
