import {
  assertArrayIncludes,
  assertEquals,
  assertThrows,
} from "$std/testing/asserts.ts";
import { join } from "$std/path/mod.ts";
import { availableHosts, locateHostConfig, readHostConfig } from "./host.ts";
import { hostsRoot } from "./paths.ts";

Deno.test("availableHosts returns the sorted host list", () => {
  const hosts = availableHosts();
  assertArrayIncludes(hosts, ["motherbrain", "forebrain"]);
  const sorted = [...hosts].sort();
  assertEquals(hosts, sorted);
});

Deno.test("locateHostConfig resolves host files under hosts/", () => {
  const path = locateHostConfig("motherbrain");
  assertEquals(path, join(hostsRoot(), "motherbrain.toml"));
});

Deno.test("locateHostConfig throws for unknown hosts", () => {
  assertThrows(() => locateHostConfig("does-not-exist"));
});

Deno.test("readHostConfig parses module directives", () => {
  const config = readHostConfig("motherbrain");
  assertEquals(config.host.name, "motherbrain");
  assertArrayIncludes(
    (config.modules ?? []).map((mod) => mod.name),
    ["pilot", "imu", "foot"],
  );
});
