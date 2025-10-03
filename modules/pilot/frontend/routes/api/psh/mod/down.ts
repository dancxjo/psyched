import { Handler } from "$fresh/server.ts";
import {
  bringModulesDown,
  listModules,
} from "../../../../../../../tools/psh/lib/module.ts";

interface RequestBody {
  modules?: string[];
}

export const handler: Handler = async (req) => {
  if (req.method !== "POST") {
    return new Response("Method Not Allowed", { status: 405 });
  }

  const body = await req.json().catch(() => ({}) as RequestBody) as RequestBody;
  const modules = body.modules?.length ? body.modules : listModules();

  try {
    await bringModulesDown(modules);
    return new Response(JSON.stringify({ ok: true, modules }), {
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
