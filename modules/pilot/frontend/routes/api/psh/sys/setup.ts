import { define } from "../../../../utils.ts";
import { setupSystemd } from "../../../../lib/server/systemd.ts";

interface RequestBody {
  module: string;
}

export const handler = define.handlers({
  async POST(ctx) {
    const body = await ctx.req.json().catch(
      () => ({} as RequestBody),
    ) as RequestBody;
    if (!body.module) {
      return new Response(
        JSON.stringify({ ok: false, error: "module required" }),
        {
          status: 400,
          headers: { "content-type": "application/json" },
        },
      );
    }

    try {
      await setupSystemd(body.module);
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
