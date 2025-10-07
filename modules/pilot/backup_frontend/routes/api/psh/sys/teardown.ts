import { Handler } from "$fresh/server.ts";
import { teardownSystemd } from "../../../../../../../tools/psh/lib/systemd.ts";

interface RequestBody {
  module: string;
}

export const handler: Handler = (req) => {
  if (req.method !== "POST") {
    return new Response("Method Not Allowed", { status: 405 });
  }

  return req.json().then((body: RequestBody) => {
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
  }).catch(() =>
    new Response(JSON.stringify({ ok: false, error: "invalid body" }), {
      status: 400,
      headers: { "content-type": "application/json" },
    })
  );
};
