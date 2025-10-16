import {
  assertEquals,
  assertInstanceOf,
  assertRejects,
} from "$std/testing/asserts.ts";
import { cleanEnvironment, __test__ } from "./clean.ts";

Deno.test("cleanEnvironment tears down modules, services, and workspace", async () => {
  const calls: string[] = [];
  __test__.replaceModuleOps(
    () => ["cockpit", "wifi", "nav"],
    async (modules) => {
      calls.push(`modules:${modules.join(",")}`);
    },
  );
  __test__.replaceServiceOps(
    () => ["ssh", "tts", "mdns"],
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
    "modules:cockpit,nav",
    "services:tts",
    "workspace",
  ]);
});

Deno.test(
  "cleanEnvironment preserves protected modules and services",
  async () => {
    const calls: string[] = [];
    __test__.replaceModuleOps(
      () => ["wifi", "mdns"],
      async () => {
        calls.push("modules:called");
      },
    );
    __test__.replaceServiceOps(
      () => ["ssh", "mdns"],
      async () => {
        calls.push("services:called");
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

    assertEquals(calls, ["workspace"]);
  },
);

Deno.test("cleanEnvironment respects skip options", async () => {
  const calls: string[] = [];
  __test__.replaceModuleOps(
    () => ["cockpit"],
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
    () => ["cockpit"],
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
    () => ["cockpit"],
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
