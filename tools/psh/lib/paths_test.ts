import { assertEquals, assertStringIncludes } from "$std/testing/asserts.ts";
import { dirname, fromFileUrl, join, resolve } from "$std/path/mod.ts";
import {
  modulesRoot,
  repoRoot,
  workspaceInstall,
  workspaceRoot,
  workspaceSrc,
} from "./paths.ts";

Deno.test("repoRoot matches project root", () => {
  const actual = repoRoot();
  const expected = resolve(
    join(dirname(fromFileUrl(import.meta.url)), "../../.."),
  );
  assertEquals(actual, expected);
});

Deno.test("workspaceRoot respects override", () => {
  const original = Deno.env.get("PSYCHED_WORKSPACE_DIR");
  try {
    Deno.env.set("PSYCHED_WORKSPACE_DIR", "~/.cache/psyched-workspace");
    const home = Deno.env.get("HOME");
    if (!home) throw new Error("HOME not defined in test environment");
    const expected = resolve(join(home, ".cache/psyched-workspace"));
    assertEquals(workspaceRoot(), expected);
  } finally {
    if (original === undefined) {
      Deno.env.delete("PSYCHED_WORKSPACE_DIR");
    } else {
      Deno.env.set("PSYCHED_WORKSPACE_DIR", original);
    }
  }
});

Deno.test("workspaceSrc relative override", () => {
  const originalDir = Deno.env.get("PSYCHED_WORKSPACE_DIR");
  const originalSrc = Deno.env.get("PSYCHED_WORKSPACE_SRC");
  try {
    Deno.env.set("PSYCHED_WORKSPACE_DIR", "workareas/custom");
    Deno.env.delete("PSYCHED_WORKSPACE_SRC");
    const expectedRoot = resolve(join(repoRoot(), "workareas/custom"));
    assertEquals(workspaceRoot(), expectedRoot);

    Deno.env.set("PSYCHED_WORKSPACE_SRC", "tree");
    const expectedSrc = resolve(join(expectedRoot, "tree"));
    assertEquals(workspaceSrc(), expectedSrc);
  } finally {
    if (originalDir === undefined) {
      Deno.env.delete("PSYCHED_WORKSPACE_DIR");
    } else {
      Deno.env.set("PSYCHED_WORKSPACE_DIR", originalDir);
    }
    if (originalSrc === undefined) {
      Deno.env.delete("PSYCHED_WORKSPACE_SRC");
    } else {
      Deno.env.set("PSYCHED_WORKSPACE_SRC", originalSrc);
    }
  }
});

Deno.test("modulesRoot resides under repo", () => {
  const root = repoRoot();
  assertStringIncludes(modulesRoot(), root);
});

Deno.test("workspaceInstall default path", () => {
  const originalDir = Deno.env.get("PSYCHED_WORKSPACE_DIR");
  const originalInstall = Deno.env.get("PSYCHED_WORKSPACE_INSTALL");
  try {
    Deno.env.delete("PSYCHED_WORKSPACE_DIR");
    Deno.env.delete("PSYCHED_WORKSPACE_INSTALL");
    assertEquals(
      workspaceInstall(),
      resolve(join(repoRoot(), "work", "install")),
    );
  } finally {
    if (originalDir === undefined) {
      Deno.env.delete("PSYCHED_WORKSPACE_DIR");
    } else {
      Deno.env.set("PSYCHED_WORKSPACE_DIR", originalDir);
    }
    if (originalInstall === undefined) {
      Deno.env.delete("PSYCHED_WORKSPACE_INSTALL");
    } else {
      Deno.env.set("PSYCHED_WORKSPACE_INSTALL", originalInstall);
    }
  }
});
