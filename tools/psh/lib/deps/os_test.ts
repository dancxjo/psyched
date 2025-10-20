import { assertEquals } from "$std/testing/asserts.ts";
import {
  determineDockerChannel,
  resolveDebianReleaseInfo,
  shouldEnableUniverse,
} from "./os.ts";

Deno.test("resolveDebianReleaseInfo parses ubuntu metadata", () => {
  const info = resolveDebianReleaseInfo({
    osReleaseText: [
      "ID=ubuntu",
      "ID_LIKE=debian",
      "VERSION_CODENAME=jammy",
    ].join("\n"),
  });
  assertEquals(info, {
    id: "ubuntu",
    idLike: ["debian"],
    codename: "jammy",
  });
  assertEquals(shouldEnableUniverse(info), true);
  assertEquals(determineDockerChannel(info), "ubuntu");
});

Deno.test("resolveDebianReleaseInfo detects debian metadata", () => {
  const info = resolveDebianReleaseInfo({
    osReleaseText: [
      "ID=debian",
      "ID_LIKE=debian",
      "VERSION_CODENAME=bookworm",
    ].join("\n"),
  });
  assertEquals(info, {
    id: "debian",
    idLike: ["debian"],
    codename: "bookworm",
  });
  assertEquals(shouldEnableUniverse(info), false);
  assertEquals(determineDockerChannel(info), "debian");
});

Deno.test("resolveDebianReleaseInfo favours lsb codename fallback", () => {
  const info = resolveDebianReleaseInfo({
    osReleaseText: [
      "ID=linuxmint",
      "ID_LIKE=ubuntu debian",
    ].join("\n"),
    lsbCodename: "Uma",
  });
  assertEquals(info, {
    id: "linuxmint",
    idLike: ["ubuntu", "debian"],
    codename: "uma",
  });
  assertEquals(shouldEnableUniverse(info), true);
  assertEquals(determineDockerChannel(info), "ubuntu");
});
