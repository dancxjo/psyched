import { Handler } from "$fresh/server.ts";
import {
  listServices,
  setupServices,
} from "../../../../../../../psh/lib/service.ts";

interface RequestBody {
  services?: string[];
}

export const handler: Handler = async (req) => {
  if (req.method !== "POST") {
    return new Response("Method Not Allowed", { status: 405 });
  }

  const body = await req.json().catch(() => ({}) as RequestBody) as RequestBody;
  const services = body.services?.length ? body.services : listServices();

  try {
    await setupServices(services);
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
};
