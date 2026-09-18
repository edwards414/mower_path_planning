import { SELF } from "cloudflare:test";
import { describe, expect, it } from "vitest";
import { SID_BYTES, T_BIN, T_TEXT, T_TEXT_MORE } from "../src/frames";
import { appHeaders, BASE, connect, register, ROBOT, robotHeaders, sleep, Sock } from "./helpers";

async function status(): Promise<{ online: boolean; sessions: number; lan: string | null }> {
  const res = await SELF.fetch(`${BASE}/v1/robots/${ROBOT}/status`, { headers: await appHeaders("status-reader") });
  expect(res.status).toBe(200);
  return (await res.json()) as { online: boolean; sessions: number; lan: string | null };
}

describe("relay", () => {
  it("robot socket needs the device key", async () => {
    await register();
    const bad = await connect(`/v1/relay/robot/${ROBOT}`, await robotHeaders("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"));
    expect(bad.status).toBe(401);
    const notWs = await SELF.fetch(`${BASE}/v1/relay/robot/${ROBOT}`, { headers: await robotHeaders() });
    expect(notWs.status).toBe(426);
  });

  it("app is refused while the robot is offline", async () => {
    await register();
    const res = await connect(`/v1/relay/app/${ROBOT}`, await appHeaders());
    expect(res.status).toBe(503);
  });

  it("heartbeat turns the robot online, frames flow both ways, close propagates", async () => {
    await register();
    const robotRes = await connect(`/v1/relay/robot/${ROBOT}`, await robotHeaders());
    expect(robotRes.status).toBe(101);
    expect(robotRes.headers.get("Sec-WebSocket-Protocol")).toBe("mrelay1");
    const robot = new Sock(robotRes);

    expect((await status()).online).toBe(false);
    robot.send(JSON.stringify({ t: "hb", lan: "192.168.1.5", info: { api_version: 2 }, telemetry: { battery: 80 } }));
    await sleep(50);
    const s = await status();
    expect(s.online).toBe(true);
    expect(s.lan).toBe("192.168.1.5");

    // app connects: robot gets an "open" with the app's pairing headers
    const appRes = await connect(`/v1/relay/app/${ROBOT}`, await appHeaders("iphone-1"));
    expect(appRes.status).toBe(101);
    const app = new Sock(appRes);
    const open = await robot.nextJson<{ t: string; sid: string; client: string; headers: Record<string, string> }>();
    expect(open.t).toBe("open");
    expect(open.sid).toMatch(/^[0-9a-f]{16}$/);
    expect(open.client).toBe("iphone-1");
    expect(open.headers["x-mower-client"] ?? open.headers["X-Mower-Client"]).toBe("iphone-1");
    expect((await status()).sessions).toBe(1);

    // app -> robot: a plain text message becomes [T_TEXT][sid][payload]
    app.send('{"op":"subscribe","topic":"/odom"}');
    const toRobot = await robot.nextBytes();
    expect(toRobot[0]).toBe(T_TEXT);
    const sidHex = Array.from(toRobot.subarray(1, 1 + SID_BYTES))
      .map((b) => b.toString(16).padStart(2, "0"))
      .join("");
    expect(sidHex).toBe(open.sid);
    expect(new TextDecoder().decode(toRobot.subarray(1 + SID_BYTES))).toBe('{"op":"subscribe","topic":"/odom"}');

    // app -> robot: an mrelay1 chunk frame keeps its type
    const chunk = new Uint8Array([T_TEXT_MORE, ...new TextEncoder().encode("part1")]);
    app.send(chunk);
    const chunkAtRobot = await robot.nextBytes();
    expect(chunkAtRobot[0]).toBe(T_TEXT_MORE);

    // robot -> app: [T_BIN][sid][payload] arrives as [T_BIN][payload]
    const payload = new TextEncoder().encode('{"op":"publish"}');
    const fromRobot = new Uint8Array(1 + SID_BYTES + payload.byteLength);
    fromRobot[0] = T_BIN;
    fromRobot.set(toRobot.subarray(1, 1 + SID_BYTES), 1);
    fromRobot.set(payload, 1 + SID_BYTES);
    robot.send(fromRobot);
    const atApp = await app.nextBytes();
    expect(atApp[0]).toBe(T_BIN);
    expect(new TextDecoder().decode(atApp.subarray(1))).toBe('{"op":"publish"}');

    // robot says the session failed auth: app is closed with 4401
    robot.send(JSON.stringify({ t: "open_err", sid: open.sid, code: 401, reason: "bad mac" }));
    expect(await app.closeCode()).toBe(4401);
    const closeMsg = await robot.nextJson<{ t: string; sid: string }>();
    expect(closeMsg.t).toBe("close");
    expect(closeMsg.sid).toBe(open.sid);
    expect((await status()).sessions).toBe(0);
    robot.close(1000, "done");
  });

  it("app close reaches the robot and robot drop closes apps", async () => {
    await register();
    const robot = new Sock(await connect(`/v1/relay/robot/${ROBOT}`, await robotHeaders()));
    robot.send(JSON.stringify({ t: "hb" }));
    await sleep(20);

    const app = new Sock(await connect(`/v1/relay/app/${ROBOT}`, await appHeaders("iphone-2")));
    const open = await robot.nextJson<{ sid: string }>();
    app.close(1000, "bye");
    const closeMsg = await robot.nextJson<{ t: string; sid: string; code: number }>();
    expect(closeMsg).toMatchObject({ t: "close", sid: open.sid, code: 1000 });

    const app2 = new Sock(await connect(`/v1/relay/app/${ROBOT}`, await appHeaders("iphone-3")));
    await robot.next(); // open
    robot.close(1001, "reboot");
    expect(await app2.closeCode()).toBe(1012);
    expect((await status()).online).toBe(false);
  });

  it("a newer robot socket replaces the old one", async () => {
    await register();
    const first = new Sock(await connect(`/v1/relay/robot/${ROBOT}`, await robotHeaders()));
    const second = new Sock(await connect(`/v1/relay/robot/${ROBOT}`, await robotHeaders()));
    expect(await first.closeCode()).toBe(4000);
    second.send(JSON.stringify({ t: "hb" }));
    await sleep(20);
    expect((await status()).online).toBe(true);
    second.close(1000, "done");
  });
});
