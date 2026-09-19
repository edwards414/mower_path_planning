// Phase 3 (docs/BACKEND_ARCHITECTURE.md §8): WHEP signaling relayed to the
// robot's loopback HTTP and TURN credentials for both ends.
import { SELF } from "cloudflare:test";
import { describe, expect, it } from "vitest";
import { appHeaders, BASE, connect, register, ROBOT, robotHeaders, sleep, Sock } from "./helpers";

interface HttpMsg {
  t: string;
  rid: string;
  method: string;
  path: string;
  headers: Record<string, string>;
  body_b64: string;
}

async function onlineRobot(): Promise<Sock> {
  await register();
  const robot = new Sock(await connect(`/v1/relay/robot/${ROBOT}`, await robotHeaders()));
  robot.send(JSON.stringify({ t: "hb" }));
  await sleep(20);
  return robot;
}

describe("phase 3", () => {
  it("http needs pairing headers and an online robot", async () => {
    await register();
    let res = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/http/front/whep`, { method: "POST", body: "v=0" });
    expect(res.status).toBe(401);
    res = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/http/front/whep`, {
      method: "POST",
      headers: await appHeaders("cam-1"),
      body: "v=0",
    });
    expect(res.status).toBe(503);
  });

  it("http round trip: request reaches the robot, answer and Location come back", async () => {
    const robot = await onlineRobot();
    const pending = SELF.fetch(`${BASE}/v1/robots/${ROBOT}/http/front/whep?x=1`, {
      method: "POST",
      headers: { ...(await appHeaders("cam-2")), "Content-Type": "application/sdp", Cookie: "secret=1" },
      body: "v=0\r\no=- 1 1 IN IP4 0.0.0.0\r\n",
    });
    const req = await robot.nextJson<HttpMsg>();
    expect(req.t).toBe("http");
    expect(req.rid).toMatch(/^[0-9a-f]{16}$/);
    expect(req.method).toBe("POST");
    expect(req.path).toBe("/front/whep?x=1");
    expect(req.headers["content-type"]).toBe("application/sdp");
    expect(req.headers.cookie).toBeUndefined();
    expect(atob(req.body_b64)).toBe("v=0\r\no=- 1 1 IN IP4 0.0.0.0\r\n");

    robot.send(
      JSON.stringify({
        t: "http_res",
        rid: req.rid,
        status: 201,
        headers: { "Content-Type": "application/sdp", Location: "/front/whep/abc123", "Set-Cookie": "no", ETag: '"1"' },
        body_b64: btoa("v=0\r\na=answer\r\n"),
      }),
    );
    const res = await pending;
    expect(res.status).toBe(201);
    expect(res.headers.get("content-type")).toBe("application/sdp");
    expect(res.headers.get("location")).toBe(`/v1/robots/${ROBOT}/http/front/whep/abc123`);
    expect(res.headers.get("etag")).toBe('"1"');
    expect(res.headers.get("set-cookie")).toBeNull();
    expect(await res.text()).toBe("v=0\r\na=answer\r\n");

    // DELETE of the session resource goes through the same prefix, no body.
    const del = SELF.fetch(`${BASE}/v1/robots/${ROBOT}/http/front/whep/abc123`, {
      method: "DELETE",
      headers: await appHeaders("cam-2"),
    });
    const delReq = await robot.nextJson<HttpMsg>();
    expect(delReq.method).toBe("DELETE");
    expect(delReq.path).toBe("/front/whep/abc123");
    expect(delReq.body_b64).toBe("");
    robot.send(JSON.stringify({ t: "http_res", rid: delReq.rid, status: 200, headers: {}, body_b64: "" }));
    expect((await del).status).toBe(200);
  });

  it("http fails fast when the robot drops mid-request and rejects big bodies", async () => {
    const robot = await onlineRobot();
    const big = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/http/front/whep`, {
      method: "POST",
      headers: await appHeaders("cam-3"),
      body: "x".repeat(65 * 1024),
    });
    expect(big.status).toBe(413);

    const pending = SELF.fetch(`${BASE}/v1/robots/${ROBOT}/http/front/whep`, {
      method: "POST",
      headers: await appHeaders("cam-3"),
      body: "v=0",
    });
    await robot.nextJson<HttpMsg>();
    robot.close(1001, "reboot");
    const res = await pending;
    expect(res.status).toBe(503);
  });

  it("turn: STUN only without a TURN key, for the app and the robot", async () => {
    await register();
    const res = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/turn`, { headers: await appHeaders("cam-4") });
    expect(res.status).toBe(200);
    const body = (await res.json()) as { iceServers: { urls: string[] }[]; expires_at: number | null };
    expect(body.iceServers[0].urls[0]).toMatch(/^stun:/);
    expect(body.expires_at).toBeNull();

    const robotRes = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/turn`, { headers: await robotHeaders() });
    expect(robotRes.status).toBe(200);
    const bad = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/turn`, { headers: await robotHeaders("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA") });
    expect(bad.status).toBe(401);
    const noAuth = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/turn`);
    expect(noAuth.status).toBe(401);
  });
});
