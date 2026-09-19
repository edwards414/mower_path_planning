// RobotHub: one Durable Object per robot (getByName(robot_id)).
//
// Holds the robot's outbound relay socket plus zero or more app sockets and
// pipes mrelay1 frames between them (docs/BACKEND_ARCHITECTURE.md §5, §6).
// Verifies the X-Mower-* headers of every incoming socket (keys from D1,
// nonce table here), keeps presence from the robot's heartbeats and writes
// it back to D1 at most once a minute.

import { DurableObject } from "cloudflare:workers";
import { getClientKey, getRobot, logEvent, writeRobotSeen } from "./db";
import { appToRobot, robotToApp, SUBPROTOCOL } from "./frames";
import { MAC_MAX_SKEW_S, randomHex, readMowerHeaders, ROBOT_CLIENT, verifyMac, type MowerHeaders } from "./hmac";
import { generateIceServers, STUN_ONLY, turnConfigured, turnTtlS, type IceServersResult } from "./turn";

const D1_WRITEBACK_S = 60;
const NONCE_TTL_S = 2 * MAC_MAX_SKEW_S;
const CLOSE_REPLACED = 4000; // a newer robot socket took over
const CLOSE_ROBOT_GONE = 1012; // "service restart": robot dropped, app should retry
const CLOSE_UNAUTHORIZED = 4401; // the robot's own auth proxy refused the session
const HTTP_MAX_BODY = 64 * 1024; // a WHEP SDP is a few KB
const HTTP_TIMEOUT_MS = 15_000;
// Request headers worth carrying to the robot and response headers worth
// carrying back; everything else (Host, cookies, CF-*) stays on its side.
const HTTP_REQ_HEADERS = ["content-type", "accept", "if-match"];
const HTTP_RES_HEADERS = ["content-type", "location", "etag", "accept-patch", "link"];

type Attachment =
  | { kind: "robot"; connectedAt: number }
  | { kind: "app"; sid: string; client: string; connectedAt: number };

interface Heartbeat {
  at: number;
  lan: string | null;
  info: unknown;
  telemetry: unknown;
}

export interface RobotStatus {
  robot_id: string;
  online: boolean;
  last_seen: number | null;
  lan: string | null;
  info: unknown;
  telemetry: unknown;
  sessions: number;
}

type RobotControl =
  | { t: "hb"; info?: unknown; telemetry?: unknown; lan?: string }
  | { t: "opened"; sid: string }
  | { t: "open_err"; sid: string; code?: number; reason?: string }
  | { t: "close"; sid: string; code?: number; reason?: string }
  | { t: "pair_result"; req: string; ok: boolean }
  | { t: "http_res"; rid: string; status: number; headers?: Record<string, string>; body_b64?: string };

interface HttpRes {
  status: number;
  headers: Record<string, string>;
  body_b64: string;
}

export class RobotHub extends DurableObject<Env> {
  private robotId: string | null = null;
  /** Relayed HTTP requests waiting for the robot's http_res (phase 3). */
  private pendingHttp = new Map<string, { resolve: (r: HttpRes) => void; timer: ReturnType<typeof setTimeout> }>();

  constructor(ctx: DurableObjectState, env: Env) {
    super(ctx, env);
    ctx.blockConcurrencyWhile(async () => {
      this.ctx.storage.sql.exec(
        "CREATE TABLE IF NOT EXISTS nonces (nonce TEXT PRIMARY KEY, at INTEGER NOT NULL)",
      );
      this.robotId = (await this.ctx.storage.get<string>("robot_id")) ?? null;
    });
    // Keep the robot's socket alive through NAT/4G without waking the object.
    this.ctx.setWebSocketAutoResponse(new WebSocketRequestResponsePair("ping", "pong"));
  }

  // ---- HTTP entry (the Worker forwards /robot, /app and /status here) ----

  async fetch(request: Request): Promise<Response> {
    const url = new URL(request.url);
    const robotId = request.headers.get("X-Hub-Robot-Id");
    if (!robotId) return json({ error: "missing robot id" }, 500);
    if (this.robotId !== robotId) {
      this.robotId = robotId;
      await this.ctx.storage.put("robot_id", robotId);
    }

    const parsed = readMowerHeaders(request.headers);
    if ("error" in parsed) return json({ error: parsed.error }, 401);

    switch (url.pathname) {
      case "/robot":
        return this.acceptRobot(request, parsed);
      case "/app":
        return this.acceptApp(request, parsed);
      case "/status": {
        const auth = await this.verifyApp(parsed);
        if (!auth.ok) return json({ error: auth.reason }, 401);
        return json(await this.status());
      }
      case "/turn": {
        // Both ends need the same TURN credentials: the phone for its own
        // relay candidates, the robot's agent to configure MediaMTX.
        const auth = parsed.client === ROBOT_CLIENT ? await this.verifyRobot(parsed) : await this.verifyApp(parsed);
        if (!auth.ok) return json({ error: auth.reason }, 401);
        return json(await this.iceServers());
      }
      default:
        if (url.pathname === "/http" || url.pathname.startsWith("/http/")) {
          const auth = await this.verifyApp(parsed);
          if (!auth.ok) return json({ error: auth.reason }, 401);
          return this.relayHttp(request, url.pathname.slice("/http".length) + url.search);
        }
        return json({ error: "not found" }, 404);
    }
  }

  // ---- phase 3: HTTP relayed to the robot's loopback (WHEP signaling) ----

  private async relayHttp(request: Request, path: string): Promise<Response> {
    const robotWs = this.robotSocket();
    if (!robotWs) return json({ error: "robot offline" }, 503);
    const declared = Number(request.headers.get("Content-Length") ?? "0");
    if (declared > HTTP_MAX_BODY) return json({ error: "body too large" }, 413);
    const body = new Uint8Array(await request.arrayBuffer());
    if (body.byteLength > HTTP_MAX_BODY) return json({ error: "body too large" }, 413);

    const headers: Record<string, string> = {};
    for (const name of HTTP_REQ_HEADERS) {
      const v = request.headers.get(name);
      if (v) headers[name] = v;
    }
    const rid = randomHex(8);
    const answer = new Promise<HttpRes>((resolve) => {
      const timer = setTimeout(() => {
        this.pendingHttp.delete(rid);
        resolve({ status: 504, headers: { "content-type": "application/json" }, body_b64: b64encode(new TextEncoder().encode(JSON.stringify({ error: "robot did not answer" }))) });
      }, HTTP_TIMEOUT_MS);
      this.pendingHttp.set(rid, { resolve, timer });
    });
    const sent = safeSend(
      robotWs,
      JSON.stringify({ t: "http", rid, method: request.method, path, headers, body_b64: body.byteLength ? b64encode(body) : "" }),
    );
    if (!sent) {
      const p = this.pendingHttp.get(rid);
      if (p) clearTimeout(p.timer);
      this.pendingHttp.delete(rid);
      return json({ error: "robot offline" }, 503);
    }
    const res = await answer;
    const out = new Headers();
    for (const [name, value] of Object.entries(res.headers)) {
      const lower = name.toLowerCase();
      if (!HTTP_RES_HEADERS.includes(lower)) continue;
      // MediaMTX answers WHEP with an absolute-path Location; keep it under
      // this robot's /http prefix so the app's DELETE comes back through here.
      out.set(lower, lower === "location" && value.startsWith("/") ? `/v1/robots/${this.robotId}/http${value}` : value);
    }
    return new Response(res.body_b64 ? b64decode(res.body_b64) : null, { status: res.status, headers: out });
  }

  private onHttpRes(msg: Extract<RobotControl, { t: "http_res" }>): void {
    const p = this.pendingHttp.get(msg.rid);
    if (!p) return;
    clearTimeout(p.timer);
    this.pendingHttp.delete(msg.rid);
    const status = Number.isInteger(msg.status) && msg.status >= 100 && msg.status <= 599 ? msg.status : 502;
    const headers: Record<string, string> = {};
    if (msg.headers && typeof msg.headers === "object") {
      for (const [k, v] of Object.entries(msg.headers)) if (typeof v === "string") headers[k] = v;
    }
    p.resolve({ status, headers, body_b64: typeof msg.body_b64 === "string" ? msg.body_b64 : "" });
  }

  private failPendingHttp(): void {
    for (const [rid, p] of this.pendingHttp) {
      clearTimeout(p.timer);
      p.resolve({ status: 503, headers: { "content-type": "application/json" }, body_b64: b64encode(new TextEncoder().encode(JSON.stringify({ error: "robot disconnected" }))) });
      this.pendingHttp.delete(rid);
    }
  }

  // ---- phase 3: TURN credentials, cached per robot ----

  private async iceServers(): Promise<IceServersResult> {
    if (!turnConfigured(this.env)) return { iceServers: STUN_ONLY, expires_at: null };
    const now = Math.trunc(Date.now() / 1000);
    const cached = await this.ctx.storage.get<IceServersResult>("turn");
    // Serve the same credentials to everyone until half their life is over, so
    // the robot's MediaMTX (which reloads its WebRTC server on a change) sees
    // a new set only about twice per TTL.
    if (cached && cached.expires_at !== null && cached.expires_at - now > turnTtlS(this.env) / 2) return cached;
    try {
      const fresh = await generateIceServers(this.env);
      await this.ctx.storage.put("turn", fresh);
      return fresh;
    } catch (err) {
      console.error(JSON.stringify({ level: "error", robot: this.robotId, err: String(err) }));
      if (cached && cached.expires_at !== null && cached.expires_at > now) return cached;
      return { iceServers: STUN_ONLY, expires_at: null };
    }
  }

  // ---- RPC ----

  async status(): Promise<RobotStatus> {
    const hb = await this.ctx.storage.get<Heartbeat>("hb");
    const online = this.robotSocket() !== null && hb !== undefined && this.isFresh(hb.at);
    return {
      robot_id: this.robotId ?? "",
      online,
      last_seen: hb?.at ?? null,
      lan: hb?.lan ?? null,
      info: hb?.info ?? null,
      telemetry: online ? (hb?.telemetry ?? null) : null,
      sessions: this.ctx.getWebSockets("app").length,
    };
  }

  // ---- socket acceptance ----

  private async acceptRobot(request: Request, h: MowerHeaders): Promise<Response> {
    if (request.headers.get("Upgrade")?.toLowerCase() !== "websocket") {
      return json({ error: "expected websocket" }, 426);
    }
    const robot = await getRobot(this.env.DB, this.robotId!);
    if (!robot) return json({ error: "unknown robot" }, 401);
    if (h.client !== ROBOT_CLIENT) return json({ error: "robot client id required" }, 401);
    const v = await verifyMac(robot.device_key, this.robotId!, h);
    if (!v.ok) return json({ error: v.reason }, 401);
    if (!this.nonceFresh(h.nonce)) return json({ error: "replayed nonce" }, 401);

    for (const old of this.ctx.getWebSockets("robot")) {
      safeClose(old, CLOSE_REPLACED, "replaced by a newer connection");
    }
    const pair = new WebSocketPair();
    const [client, server] = [pair[0], pair[1]];
    const att: Attachment = { kind: "robot", connectedAt: Date.now() };
    this.ctx.acceptWebSocket(server, ["robot"]);
    server.serializeAttachment(att);
    await this.ctx.storage.put("robot_connected_at", att.connectedAt);
    await this.ctx.storage.setAlarm(Date.now() + this.offlineAfterMs());
    this.ctx.waitUntil(logEvent(this.env.DB, this.robotId, null, "robot.connected"));
    return upgradeResponse(client, request);
  }

  private async acceptApp(request: Request, h: MowerHeaders): Promise<Response> {
    if (request.headers.get("Upgrade")?.toLowerCase() !== "websocket") {
      return json({ error: "expected websocket" }, 426);
    }
    const auth = await this.verifyApp(h);
    if (!auth.ok) return json({ error: auth.reason }, 401);
    const robotWs = this.robotSocket();
    if (!robotWs) return json({ error: "robot offline" }, 503);

    const sid = randomHex(8);
    const pair = new WebSocketPair();
    const [client, server] = [pair[0], pair[1]];
    const att: Attachment = { kind: "app", sid, client: h.client, connectedAt: Date.now() };
    this.ctx.acceptWebSocket(server, ["app", `s:${sid}`]);
    server.serializeAttachment(att);

    // Forward the app's pairing headers so the robot's own gate re-checks them.
    const headers: Record<string, string> = {};
    request.headers.forEach((value, name) => {
      if (name.toLowerCase().startsWith("x-mower-")) headers[name] = value;
    });
    const sent = safeSend(robotWs, JSON.stringify({ t: "open", sid, client: h.client, headers }));
    if (!sent) {
      safeClose(server, CLOSE_ROBOT_GONE, "robot offline");
    }
    this.ctx.waitUntil(logEvent(this.env.DB, this.robotId, h.client, "session.open", sid));
    return upgradeResponse(client, request);
  }

  private async verifyRobot(h: MowerHeaders): Promise<{ ok: true } | { ok: false; reason: string }> {
    const robot = await getRobot(this.env.DB, this.robotId!);
    if (!robot) return { ok: false, reason: "unknown robot" };
    const v = await verifyMac(robot.device_key, this.robotId!, h);
    if (!v.ok) return { ok: false, reason: v.reason };
    if (!this.nonceFresh(h.nonce)) return { ok: false, reason: "replayed nonce" };
    return { ok: true };
  }

  private async verifyApp(h: MowerHeaders): Promise<{ ok: true } | { ok: false; reason: string }> {
    if (h.client === ROBOT_CLIENT) return { ok: false, reason: "reserved client id" };
    const row = await getClientKey(this.env.DB, this.robotId!, h.client);
    if (!row) return { ok: false, reason: "not paired" };
    const v = await verifyMac(row.key, this.robotId!, h);
    if (!v.ok) return { ok: false, reason: v.reason };
    if (!this.nonceFresh(h.nonce)) return { ok: false, reason: "replayed nonce" };
    return { ok: true };
  }

  // ---- socket events (hibernation API) ----

  async webSocketMessage(ws: WebSocket, message: string | ArrayBuffer): Promise<void> {
    const att = ws.deserializeAttachment() as Attachment | null;
    if (!att) return;
    if (att.kind === "robot") {
      if (typeof message === "string") await this.onRobotControl(message);
      else this.onRobotFrame(message);
      return;
    }
    // app -> robot
    const robotWs = this.robotSocket();
    if (!robotWs) {
      safeClose(ws, CLOSE_ROBOT_GONE, "robot offline");
      return;
    }
    const frame = appToRobot(message, att.sid);
    if (!frame) {
      safeClose(ws, 1003, "bad mrelay1 frame");
      return;
    }
    safeSend(robotWs, frame);
  }

  async webSocketClose(ws: WebSocket, code: number, reason: string): Promise<void> {
    await this.onSocketGone(ws, code, reason);
  }

  async webSocketError(ws: WebSocket, error: unknown): Promise<void> {
    await this.onSocketGone(ws, 1011, String(error));
  }

  private async onSocketGone(ws: WebSocket, code: number, reason: string): Promise<void> {
    const att = ws.deserializeAttachment() as Attachment | null;
    if (!att) return;
    if (att.kind === "robot") {
      // Ignore the close of a socket we replaced: the live one is still there.
      const live = this.robotSocket();
      if (live && live !== ws) return;
      for (const app of this.ctx.getWebSockets("app")) {
        safeClose(app, CLOSE_ROBOT_GONE, "robot disconnected");
      }
      this.failPendingHttp();
      this.ctx.waitUntil(logEvent(this.env.DB, this.robotId, null, "robot.disconnected", `${code} ${reason}`));
      return;
    }
    const robotWs = this.robotSocket();
    if (robotWs) {
      safeSend(robotWs, JSON.stringify({ t: "close", sid: att.sid, code: validCloseCode(code), reason }));
    }
    this.ctx.waitUntil(logEvent(this.env.DB, this.robotId, att.client, "session.close", `${att.sid} ${code}`));
  }

  // ---- robot messages ----

  private async onRobotControl(text: string): Promise<void> {
    let msg: RobotControl;
    try {
      msg = JSON.parse(text) as RobotControl;
    } catch {
      return;
    }
    switch (msg.t) {
      case "hb":
        await this.onHeartbeat(msg);
        return;
      case "opened":
        return;
      case "open_err": {
        const app = this.appSocket(msg.sid);
        if (app) safeClose(app, msg.code === 401 ? CLOSE_UNAUTHORIZED : 1011, msg.reason ?? "open failed");
        return;
      }
      case "close": {
        const app = this.appSocket(msg.sid);
        if (app) safeClose(app, validCloseCode(msg.code ?? 1000), msg.reason ?? "");
        return;
      }
      case "http_res":
        this.onHttpRes(msg);
        return;
      default:
        return; // pair_result: phase 2
    }
  }

  private onRobotFrame(buf: ArrayBuffer): void {
    const parsed = robotToApp(buf);
    if (!parsed) return;
    const app = this.appSocket(parsed.sid);
    if (!app) {
      const robotWs = this.robotSocket();
      if (robotWs) safeSend(robotWs, JSON.stringify({ t: "close", sid: parsed.sid, code: 1000, reason: "no such session" }));
      return;
    }
    safeSend(app, parsed.frame);
  }

  private async onHeartbeat(msg: Extract<RobotControl, { t: "hb" }>): Promise<void> {
    const now = Date.now() / 1000;
    const hb: Heartbeat = {
      at: Math.trunc(now),
      lan: typeof msg.lan === "string" ? msg.lan : null,
      info: msg.info ?? null,
      telemetry: msg.telemetry ?? null,
    };
    await this.ctx.storage.put("hb", hb);
    await this.ctx.storage.setAlarm(Date.now() + this.offlineAfterMs());
    const lastWrite = (await this.ctx.storage.get<number>("d1_written_at")) ?? 0;
    if (now - lastWrite >= D1_WRITEBACK_S) {
      await this.ctx.storage.put("d1_written_at", now);
      this.ctx.waitUntil(writeRobotSeen(this.env.DB, this.robotId!, hb.at, hb.lan, hb.info));
    }
  }

  async alarm(): Promise<void> {
    const hb = await this.ctx.storage.get<Heartbeat>("hb");
    if (hb && this.isFresh(hb.at)) {
      await this.ctx.storage.setAlarm(Date.now() + this.offlineAfterMs());
      return;
    }
    // No heartbeat in time: the socket is dead even if the close never arrived.
    for (const ws of this.ctx.getWebSockets("robot")) safeClose(ws, 1001, "heartbeat timeout");
    for (const ws of this.ctx.getWebSockets("app")) safeClose(ws, CLOSE_ROBOT_GONE, "robot offline");
    if (hb && this.robotId) {
      await writeRobotSeen(this.env.DB, this.robotId, hb.at, hb.lan, hb.info);
    }
  }

  // ---- helpers ----

  private robotSocket(): WebSocket | null {
    const sockets = this.ctx.getWebSockets("robot");
    return sockets.length ? sockets[sockets.length - 1] : null;
  }

  private appSocket(sid: string): WebSocket | null {
    const sockets = this.ctx.getWebSockets(`s:${sid}`);
    return sockets.length ? sockets[0] : null;
  }

  private offlineAfterMs(): number {
    return Number(this.env.OFFLINE_AFTER_S || "35") * 1000;
  }

  private isFresh(atS: number): boolean {
    return Date.now() / 1000 - atS <= Number(this.env.OFFLINE_AFTER_S || "35");
  }

  /** true when the nonce was not seen inside the skew window; records it. */
  private nonceFresh(nonce: string): boolean {
    const now = Math.trunc(Date.now() / 1000);
    this.ctx.storage.sql.exec("DELETE FROM nonces WHERE at < ?", now - NONCE_TTL_S);
    const seen = this.ctx.storage.sql.exec("SELECT 1 FROM nonces WHERE nonce = ?", nonce).toArray();
    if (seen.length) return false;
    this.ctx.storage.sql.exec("INSERT INTO nonces (nonce, at) VALUES (?, ?)", nonce, now);
    return true;
  }
}

function upgradeResponse(client: WebSocket, request: Request): Response {
  const headers = new Headers();
  const offered = request.headers.get("Sec-WebSocket-Protocol") ?? "";
  if (offered.split(",").map((s) => s.trim()).includes(SUBPROTOCOL)) {
    headers.set("Sec-WebSocket-Protocol", SUBPROTOCOL);
  }
  return new Response(null, { status: 101, webSocket: client, headers });
}

function json(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "Content-Type": "application/json" },
  });
}

function b64encode(bytes: Uint8Array): string {
  let s = "";
  for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
  return btoa(s);
}

function b64decode(s: string): Uint8Array {
  const bin = atob(s);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

function safeSend(ws: WebSocket, data: string | Uint8Array): boolean {
  try {
    ws.send(data);
    return true;
  } catch {
    return false;
  }
}

function safeClose(ws: WebSocket, code: number, reason: string): void {
  try {
    ws.close(code, reason.slice(0, 120));
  } catch {
    // already closed
  }
}

/** Codes a server may pass to close(): 1000, 1001, 1003, 1007-1011, 3000-4999. */
function validCloseCode(code: number): number {
  if (code === 1000 || code === 1001 || code === 1003) return code;
  if (code >= 1007 && code <= 1011) return code;
  if (code >= 3000 && code <= 4999) return code;
  return 1000;
}
