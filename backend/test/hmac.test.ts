import { describe, expect, it } from "vitest";
import { b32decode, b32encode, computeMac, readMowerHeaders, verifyMac } from "../src/hmac";

// Shared with mower_mission/test/test_pairing.py and the app's pairing_test.dart.
const SECRET = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
const ROBOT = "MW-7K3Q9P";
const CLIENT = "iphone-1234";
const TIME = 1789600000;
const NONCE = "00112233445566778899aabbccddeeff";
const MAC = "d33d2137cf8c6bb75ba80ff22b9afbf32a09f2346e83617e41e96026aa2cfd42";

describe("hmac", () => {
  it("matches the shared test vector", async () => {
    expect(await computeMac(SECRET, ROBOT, CLIENT, TIME, NONCE)).toBe(MAC);
  });

  it("base32 round-trips without padding", () => {
    const bytes = new Uint8Array([0xde, 0xad, 0xbe, 0xef, 0x01]);
    const s = b32encode(bytes);
    expect(s).toBe("32W353YB");
    expect(Array.from(b32decode(s))).toEqual(Array.from(bytes));
    expect(b32decode(SECRET).byteLength).toBe(20);
  });

  it("verifies headers inside the skew window and rejects outside", async () => {
    const h = new Headers({
      "X-Mower-Robot": ROBOT,
      "X-Mower-Client": CLIENT,
      "X-Mower-Time": String(TIME),
      "X-Mower-Nonce": NONCE,
      "X-Mower-Mac": MAC,
    });
    const parsed = readMowerHeaders(h);
    if ("error" in parsed) throw new Error(parsed.error);
    expect(await verifyMac(SECRET, ROBOT, parsed, TIME + 30)).toEqual({ ok: true, client: CLIENT });
    expect((await verifyMac(SECRET, ROBOT, parsed, TIME + 61)).ok).toBe(false);
    expect((await verifyMac(SECRET, "MW-000000", parsed, TIME)).ok).toBe(false);
    expect((await verifyMac("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB", ROBOT, parsed, TIME)).ok).toBe(false);
  });

  it("rejects malformed headers", () => {
    expect(readMowerHeaders(new Headers())).toEqual({ error: "missing pairing headers" });
    const bad = new Headers({
      "X-Mower-Robot": ROBOT,
      "X-Mower-Client": CLIENT,
      "X-Mower-Time": "abc",
      "X-Mower-Nonce": NONCE,
      "X-Mower-Mac": MAC,
    });
    expect(readMowerHeaders(bad)).toEqual({ error: "bad time" });
  });
});
