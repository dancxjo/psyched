import {
  assertEquals,
  assertInstanceOf,
  assertRejects,
} from "$std/testing/asserts.ts";
import { cleanEnvironment, __test__ } from "./clean.ts";

Deno.test("cleanEnvironment tears down modules, services, and workspace", async () => {
  const calls: string[] = [];
  __test__.replaceModuleOps(
    () => ["pilot", "nav"],
    async (modules) => {
      calls.push(`modules:${modules.join(",")}`);
    },
  );
  __test__.replaceServiceOps(
    () => ["tts"],
    async (services) => {
      calls.push(`services:${services.join(",")}`);
    },
  );
  __test__.replaceWorkspaceReset(async () => {
    calls.push("workspace");
  });

  try {
    await cleanEnvironment();
  } finally {
    __test__.reset();
  }

  assertEquals(calls, [
    "modules:pilot,nav",
    "services:tts",
    "workspace",
  ]);
});

Deno.test("cleanEnvironment respects skip options", async () => {
  const calls: string[] = [];
  __test__.replaceModuleOps(
    () => ["pilot"],
    async (modules) => {
      calls.push(`modules:${modules.join(",")}`);
    },
  );
  __test__.replaceServiceOps(
    () => ["tts"],
    async (services) => {
      calls.push(`services:${services.join(",")}`);
    },
  );
  __test__.replaceWorkspaceReset(async () => {
    calls.push("workspace");
  });

  try {
    await cleanEnvironment({
      skipModules: true,
      skipServices: true,
      skipWorkspace: true,
    });
  } finally {
    __test__.reset();
  }

  assertEquals(calls, []);
});

Deno.test("cleanEnvironment aggregates teardown failures", async () => {
  __test__.replaceModuleOps(
    () => ["pilot"],
    async () => {
      throw new Error("module failure");
    },
  );
  __test__.replaceServiceOps(
    () => ["tts"],
    async () => {
      throw new Error("service failure");
    },
  );
  __test__.replaceWorkspaceReset(async () => {
    throw new Error("workspace failure");
  });

  try {
    await assertRejects(
      () => cleanEnvironment(),
      AggregateError,
      "Failed to complete clean: modules, services, workspace",
    );
  } finally {
    __test__.reset();
  }
});

Deno.test("cleanEnvironment propagates AggregateError causes", async () => {
  const moduleError = new Error("module failure");
  __test__.replaceModuleOps(
    () => ["pilot"],
    async () => {
      throw moduleError;
    },
  );
  __test__.replaceServiceOps(() => []);
  __test__.replaceWorkspaceReset(async () => {});

  try {
    const error = await assertRejects(
      async () => {
        await cleanEnvironment();
      },
      AggregateError,
    );
    assertInstanceOf(error, AggregateError);
    assertEquals(error.errors, [moduleError]);
  } finally {
    __test__.reset();
  }
});
