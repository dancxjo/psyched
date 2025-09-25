import { assertEquals } from "@std/assert";
import { colconBuild, colconInstall } from "./colcon.ts";
import { createDaxStub, type StubInvocation } from "./test_utils.ts";

function assertColconInvocation(invocation: StubInvocation) {
  assertEquals(invocation.parts[0], "colcon ");
  assertEquals(invocation.parts[1], "");
  assertEquals(invocation.values.length, 1);
  assertEquals(invocation.options.stdout, "inherit");
  assertEquals(invocation.options.stderr, "inherit");
}

function expectInvocation(
  invocation: StubInvocation | null,
  message: string,
): StubInvocation {
  if (!invocation) {
    throw new Error(message);
  }
  return invocation;
}

Deno.test("colcon build shells out via dax", async () => {
  let captured: StubInvocation | null = null;
  const stub = createDaxStub((invocation) => {
    captured = invocation;
    return { code: 0 };
  });
  await colconBuild(stub);
  const invocation = expectInvocation(
    captured,
    "Expected colcon build to invoke dax",
  );
  const args = invocation.values[0] as string[];
  assertEquals(Array.isArray(args), true);
  assertEquals(args.slice(0, 2), ["build", "--symlink-install"]);
  assertEquals(args[2], "--base-paths");
  assertEquals(typeof args[3], "string");
  assertColconInvocation(invocation);
});

Deno.test("colcon install shells out via dax", async () => {
  let captured: StubInvocation | null = null;
  const stub = createDaxStub((invocation) => {
    captured = invocation;
    return { code: 0 };
  });
  await colconInstall(stub);
  const invocation = expectInvocation(
    captured,
    "Expected colcon install to invoke dax",
  );
  const args = invocation.values[0] as string[];
  assertEquals(Array.isArray(args), true);
  assertEquals(args.slice(0, 1), ["install"]);
  assertEquals(args[1], "--base-paths");
  assertEquals(typeof args[2], "string");
  assertColconInvocation(invocation);
});
