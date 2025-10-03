import { Handler } from "$fresh/server.ts";
import { moduleStatuses } from "../../../../../../../tools/psh/lib/module.ts";

export const handler: Handler = () => {
  const statuses = moduleStatuses();
  return new Response(JSON.stringify({ ok: true, statuses }), {
    status: 200,
    headers: { "content-type": "application/json" },
  });
};
