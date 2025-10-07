import { define } from "../../../../utils.ts";
import {
  listServices,
  teardownServices,
} from "../../../../../../../tools/psh/lib/service.ts";

interface RequestBody {
  services?: string[];
}

export const handler = define.handlers({
  async POST(ctx) {
    const body = await ctx.req.json().catch(() =>
      ({}) as RequestBody
    ) as RequestBody;
    const services = body.services?.length ? body.services : listServices();

    try {
      await teardownServices(services);
      return new Response(JSON.stringify({ ok: true, services }), {
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
