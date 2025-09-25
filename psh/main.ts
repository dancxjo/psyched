#!/usr/bin/env -S deno run -A

import { createCli } from "./cli.ts";

if (import.meta.main) {
  await createCli().parse(Deno.args);
}
