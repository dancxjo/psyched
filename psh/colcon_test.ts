import { assertEquals } from "@std/assert";
import { colconBuild, colconInstall } from "./colcon.ts";
import { createDaxStub, type StubInvocation } from "./test_utils.ts";
import { repoPath } from "./util.ts";

const withRosEnv = repoPath("../tools/with_ros_env.sh");

function assertColconInvocation(invocation: StubInvocation) {
  assertEquals(invocation.values[0], withRosEnv);
  assertEquals(invocation.parts.length, 3);
  assertEquals(invocation.values.length, 2);
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
  const args = invocation.values[1] as string[];
  assertEquals(Array.isArray(args), true);
  assertEquals(args.slice(0, 3), ["colcon", "build", "--symlink-install"]);
  assertEquals(args[3], "--base-paths");
  assertEquals(typeof args[4], "string");
  assertColconInvocation(invocation);
});

Deno.test("colcon install explains skip", () => {
  const messages: string[] = [];
  const originalLog = console.log;
  try {
    console.log = (...args: unknown[]) => {
      messages.push(args.map((arg) => String(arg)).join(" "));
    };
    colconInstall();
  } finally {
    console.log = originalLog;
  }
  assertEquals(messages.length > 0, true);
  assertEquals(
    messages[0],
    "[psh] Skipping 'colcon install' (not supported); ensure 'colcon build' was run instead.",
  );
});
