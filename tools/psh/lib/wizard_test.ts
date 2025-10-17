import { assertEquals } from "$std/testing/asserts.ts";
import { __test__ } from "./wizard.ts";

const {
  applyModuleToggles,
  applyServiceToggles,
} = __test__;

Deno.test("applyModuleToggles keeps env and depends_on while filtering", () => {
  const modules = [
    {
      name: "alpha",
      setup: true,
      launch: false,
      env: { A: "1" },
      depends_on: ["ros2"],
    },
    { name: "beta", setup: true, launch: true },
    { name: "gamma", setup: false, launch: false },
  ];

  const toggles = new Map([
    ["alpha", { setup: true, launch: false }],
    ["beta", { setup: false, launch: true }],
  ]);

  const result = applyModuleToggles(modules, toggles);
  assertEquals(result, [
    {
      name: "alpha",
      setup: true,
      launch: false,
      env: { A: "1" },
      depends_on: ["ros2"],
    },
    { name: "beta", setup: false, launch: true },
  ]);
});

Deno.test("applyServiceToggles drops services without selections", () => {
  const services = [
    { name: "tts", setup: true, up: true },
    { name: "llm", setup: false, up: true },
  ];

  const toggles = new Map([
    ["tts", { setup: true, up: false }],
  ]);

  const result = applyServiceToggles(services, toggles);
  assertEquals(result, [
    { name: "tts", setup: true, up: false },
  ]);
});
