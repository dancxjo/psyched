import { define } from "../../../../utils.ts";
import { serviceStatuses } from "../../../../../../../tools/psh/lib/service.ts";

export const handler = define.handlers({
  async GET() {
    const statuses = await serviceStatuses();
    return new Response(JSON.stringify({ ok: true, statuses }), {
      status: 200,
      headers: { "content-type": "application/json" },
    });
  },
});
