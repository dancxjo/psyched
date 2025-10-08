import { define } from "../../../../utils.ts";
import { teardownSystemd } from "../../../../lib/server/systemd.ts";

interface RequestBody {
  module: string;
}

export const handler = define.handlers({
  async POST(ctx) {
    const body = await ctx.req.json().catch(() => null) as RequestBody | null;
    if (!body?.module) {
      return new Response(
        JSON.stringify({ ok: false, error: "module required" }),
        {
          status: 400,
          headers: { "content-type": "application/json" },
        },
      );
    }
    try {
      teardownSystemd(body.module);
      return new Response(JSON.stringify({ ok: true }), {
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
