import { Handler } from "$fresh/server.ts";
import { provisionHost } from "../../../../../../../tools/psh/lib/host.ts";

interface RequestBody {
  hosts?: string[];
}

export const handler: Handler = async (req) => {
  if (req.method !== "POST") {
    return new Response("Method Not Allowed", { status: 405 });
  }

  const body = await req.json().catch(() => ({}) as RequestBody) as RequestBody;
  const hosts = body.hosts?.length ? body.hosts : undefined;

  try {
    if (!hosts) {
      await provisionHost();
    } else {
      for (const host of hosts) {
        await provisionHost(host);
      }
    }
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
