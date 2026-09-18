import { SELF } from "cloudflare:test";
import { describe, expect, it } from "vitest";
import { appHeaders, BASE, PROVISION, register, ROBOT } from "./helpers";

describe("api", () => {
  it("health", async () => {
    const res = await SELF.fetch(`${BASE}/v1/health`);
    expect(res.status).toBe(200);
    expect(((await res.json()) as { ok: boolean }).ok).toBe(true);
  });

  it("register needs the provision token", async () => {
    const res = await SELF.fetch(`${BASE}/v1/robots/register`, {
      method: "POST",
      headers: { Authorization: "Bearer nope", "Content-Type": "application/json" },
      body: "{}",
    });
    expect(res.status).toBe(401);
  });

  it("register creates, then updates, then refuses another device key", async () => {
    expect((await register()).status).toBe(201);
    expect((await register()).status).toBe(200);
    const other = await register(ROBOT, "GEZDGNBVGY3TQOJQGEZDGNBVGY3TQOJQ");
    expect(other.status).toBe(409);
  });

  it("register validates the body", async () => {
    const res = await SELF.fetch(`${BASE}/v1/robots/register`, {
      method: "POST",
      headers: { Authorization: `Bearer ${PROVISION}`, "Content-Type": "application/json" },
      body: JSON.stringify({ robot_id: "nope", device_key: "x", pairing_secret: "y" }),
    });
    expect(res.status).toBe(400);
  });

  it("status needs valid pairing headers and reports offline", async () => {
    await register();
    const unauth = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/status`);
    expect(unauth.status).toBe(401);

    const badKey = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/status`, {
      headers: await appHeaders("iphone-1234", "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB"),
    });
    expect(badKey.status).toBe(401);

    const res = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/status`, { headers: await appHeaders() });
    expect(res.status).toBe(200);
    const body = (await res.json()) as { robot_id: string; online: boolean; last_seen: number | null };
    expect(body.robot_id).toBe(ROBOT);
    expect(body.online).toBe(false);
    expect(body.last_seen).toBeNull();
  });

  it("status rejects a replayed nonce", async () => {
    await register();
    const headers = await appHeaders();
    expect((await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/status`, { headers })).status).toBe(200);
    expect((await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/status`, { headers })).status).toBe(401);
  });

  it("unknown robot id shapes are rejected before touching a hub", async () => {
    const res = await SELF.fetch(`${BASE}/v1/robots/not-a-robot/status`, { headers: await appHeaders() });
    expect(res.status).toBe(400);
  });
});
