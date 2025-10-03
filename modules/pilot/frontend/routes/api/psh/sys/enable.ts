import { Handler } from "$fresh/server.ts";
import { enableSystemd } from "../../../../../../../psh/lib/systemd.ts";

interface RequestBody {
  module: string;
}

export const handler: Handler = async (req) => {
  if (req.method !== "POST") {
    return new Response("Method Not Allowed", { status: 405 });
  }

  const body = await req.json().catch(() => ({} as RequestBody)) as RequestBody;
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
    await enableSystemd(body.module);
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
};
