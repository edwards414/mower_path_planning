// X-Mower-* pairing headers, shared with the robot (mower_mission/identity.py)
// and the app (pairing_auth.dart). One test vector for all three: see
// test/hmac.test.ts and docs/ROBOT_API.md "配對".

export const MAC_MAX_SKEW_S = 60;
export const ROBOT_ID_RE = /^MW-[A-Z0-9]{6}$/;
export const B32_KEY_RE = /^[A-Z2-7]{16,104}$/;
const NONCE_RE = /^[0-9a-fA-F]{16,64}$/;
export const ROBOT_CLIENT = "@robot";

export interface MowerHeaders {
  robot: string;
  client: string;
  time: number;
  nonce: string;
  mac: string;
}

export function readMowerHeaders(h: Headers): MowerHeaders | { error: string } {
  const robot = (h.get("X-Mower-Robot") ?? "").trim();
  const client = (h.get("X-Mower-Client") ?? "").trim();
  const timeRaw = (h.get("X-Mower-Time") ?? "").trim();
  const nonce = (h.get("X-Mower-Nonce") ?? "").trim().toLowerCase();
  const mac = (h.get("X-Mower-Mac") ?? "").trim().toLowerCase();
  if (!robot || !client || !timeRaw || !nonce || !mac) return { error: "missing pairing headers" };
  if (!/^\d{1,12}$/.test(timeRaw)) return { error: "bad time" };
  if (!NONCE_RE.test(nonce)) return { error: "bad nonce" };
  if (!/^[0-9a-f]{64}$/.test(mac)) return { error: "bad mac" };
  return { robot, client, time: Number(timeRaw), nonce, mac };
}

const B32 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";

/** RFC 4648 base32 decode; padding optional (the QR strips it). */
export function b32decode(s: string): Uint8Array {
  const clean = s.trim().toUpperCase().replace(/=+$/, "");
  const out: number[] = [];
  let bits = 0;
  let value = 0;
  for (const ch of clean) {
    const v = B32.indexOf(ch);
    if (v < 0) throw new Error("bad base32");
    value = (value << 5) | v;
    bits += 5;
    if (bits >= 8) {
      out.push((value >>> (bits - 8)) & 0xff);
      bits -= 8;
    }
  }
  return new Uint8Array(out);
}

export function b32encode(bytes: Uint8Array): string {
  let bits = 0;
  let value = 0;
  let out = "";
  for (const b of bytes) {
    value = (value << 8) | b;
    bits += 8;
    while (bits >= 5) {
      out += B32[(value >>> (bits - 5)) & 31];
      bits -= 5;
    }
  }
  if (bits > 0) out += B32[(value << (5 - bits)) & 31];
  return out;
}

export function randomKeyB32(bytes = 32): string {
  const buf = new Uint8Array(bytes);
  crypto.getRandomValues(buf);
  return b32encode(buf);
}

export function randomHex(bytes: number): string {
  const buf = new Uint8Array(bytes);
  crypto.getRandomValues(buf);
  return hex(buf);
}

export function hex(bytes: Uint8Array): string {
  let s = "";
  for (const b of bytes) s += b.toString(16).padStart(2, "0");
  return s;
}

export async function computeMac(
  keyB32: string,
  robotId: string,
  client: string,
  time: number,
  nonce: string,
): Promise<string> {
  const key = await crypto.subtle.importKey(
    "raw",
    b32decode(keyB32),
    { name: "HMAC", hash: "SHA-256" },
    false,
    ["sign"],
  );
  const msg = new TextEncoder().encode(`${robotId}\n${client}\n${Math.trunc(time)}\n${nonce}`);
  return hex(new Uint8Array(await crypto.subtle.sign("HMAC", key, msg)));
}

export type VerifyResult = { ok: true; client: string } | { ok: false; reason: string };

/**
 * Check the headers against one key. Nonce replay is the caller's job
 * (the hub keeps the nonce table); this only validates time and MAC.
 */
export async function verifyMac(
  keyB32: string,
  expectRobot: string,
  h: MowerHeaders,
  nowS = Date.now() / 1000,
): Promise<VerifyResult> {
  if (h.robot !== expectRobot) return { ok: false, reason: `wrong robot (${h.robot})` };
  if (Math.abs(nowS - h.time) > MAC_MAX_SKEW_S) {
    return { ok: false, reason: `clock skew ${Math.trunc(nowS - h.time)} s` };
  }
  const expected = await computeMac(keyB32, h.robot, h.client, h.time, h.nonce);
  const enc = new TextEncoder();
  const a = enc.encode(expected);
  const b = enc.encode(h.mac);
  if (a.byteLength !== b.byteLength || !crypto.subtle.timingSafeEqual(a, b)) {
    return { ok: false, reason: "bad mac" };
  }
  return { ok: true, client: h.client };
}

/** Constant-time comparison of two secrets of any length. */
export async function secretsEqual(a: string, b: string): Promise<boolean> {
  const enc = new TextEncoder();
  const [ha, hb] = await Promise.all([
    crypto.subtle.digest("SHA-256", enc.encode(a)),
    crypto.subtle.digest("SHA-256", enc.encode(b)),
  ]);
  return crypto.subtle.timingSafeEqual(ha, hb);
}

/** Headers an app/robot client attaches to a request (used by tests and tools). */
export async function signHeaders(
  keyB32: string,
  robotId: string,
  client: string,
  nowS = Date.now() / 1000,
): Promise<Record<string, string>> {
  const time = Math.trunc(nowS);
  const nonce = randomHex(16);
  return {
    "X-Mower-Robot": robotId,
    "X-Mower-Client": client,
    "X-Mower-Time": String(time),
    "X-Mower-Nonce": nonce,
    "X-Mower-Mac": await computeMac(keyB32, robotId, client, time, nonce),
  };
}
