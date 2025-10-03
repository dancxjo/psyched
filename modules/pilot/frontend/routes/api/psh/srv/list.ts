import { Handler } from "$fresh/server.ts";
import { serviceStatuses } from "../../../../../../../psh/lib/service.ts";

export const handler: Handler = async () => {
  const statuses = await serviceStatuses();
  return new Response(JSON.stringify({ ok: true, statuses }), {
    status: 200,
    headers: { "content-type": "application/json" },
  });
};
