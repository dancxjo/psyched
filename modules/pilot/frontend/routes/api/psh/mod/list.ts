import { define } from "../../../../utils.ts";
import { moduleStatuses } from "../../../../lib/server/modules.ts";

export const handler = define.handlers({
  GET() {
    try {
      const statuses = moduleStatuses();
      return new Response(JSON.stringify({ ok: true, statuses }), {
        status: 200,
        headers: { "content-type": "application/json" },
      });
    } catch (error) {
      console.error("Failed to load module statuses", error);
      return new Response(
        JSON.stringify({ ok: false, error: "Failed to load module statuses" }),
        {
          status: 500,
          headers: { "content-type": "application/json" },
        },
      );
    }
  },
});
