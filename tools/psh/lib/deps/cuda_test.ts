import { assert, assertEquals } from "$std/assert/mod.ts";
import { selectCudaRepo } from "./cuda.ts";

Deno.test("selectCudaRepo returns exact match for Ubuntu 24.04", () => {
  const osRelease = [
    'NAME="Ubuntu"',
    'PRETTY_NAME="Ubuntu 24.04 LTS"',
    'VERSION_ID="24.04"',
  ].join("\n");

  const result = selectCudaRepo(osRelease);

  assertEquals(result.repo, "ubuntu2404");
  assertEquals(result.requestedVersion, "24.04");
  assertEquals(result.fallback, false);
  assertEquals(result.message, undefined);
});

Deno.test("selectCudaRepo falls back when repository is not published", () => {
  const osRelease = [
    'NAME="Ubuntu"',
    'PRETTY_NAME="Ubuntu 25.04"',
    'VERSION_ID="25.04"',
  ].join("\n");

  const result = selectCudaRepo(osRelease);

  assertEquals(result.repo, "ubuntu2404");
  assertEquals(result.requestedVersion, "25.04");
  assert(result.fallback);
  assert(result.message?.includes("25.04"));
});

Deno.test("selectCudaRepo handles missing VERSION_ID", () => {
  const osRelease = [
    'NAME="Ubuntu"',
    'PRETTY_NAME="Ubuntu (rolling)"',
  ].join("\n");

  const result = selectCudaRepo(osRelease);

  assertEquals(result.repo, "ubuntu2404");
  assertEquals(result.requestedVersion, "(unknown)");
  assert(result.fallback);
});
