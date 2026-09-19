// Mower backend Worker: routing only. Presence, auth and relaying live in the
// RobotHub Durable Object; the registry lives in D1 (src/db.ts).

import { registerRobot } from "./db";
import { B32_KEY_RE, ROBOT_ID_RE, secretsEqual } from "./hmac";

export { RobotHub } from "./hub";

const MAX_BODY = 16 * 1024;

export default {
  async fetch(request, env): Promise<Response> {
    try {
      return await route(request, env);
    } catch (err) {
      console.error(JSON.stringify({ level: "error", path: new URL(request.url).pathname, err: String(err) }));
      return json({ error: "internal error" }, 500);
    }
  },
} satisfies ExportedHandler<Env>;

async function route(request: Request, env: Env): Promise<Response> {
  const url = new URL(request.url);
  const parts = url.pathname.split("/").filter(Boolean); // ["v1", ...]
  if (parts[0] !== "v1") return json({ error: "not found" }, 404);

  if (parts[1] === "health" && parts.length === 2) {
    return json({ ok: true, ts: Math.trunc(Date.now() / 1000) });
  }

  if (parts[1] === "robots" && parts[2] === "register" && parts.length === 3) {
    if (request.method !== "POST") return json({ error: "method not allowed" }, 405);
    return register(request, env);
  }

  // /v1/robots/{id}/status | /turn | /http/*
  if (parts[1] === "robots" && parts.length >= 4) {
    const robotId = parts[2];
    if (!ROBOT_ID_RE.test(robotId)) return json({ error: "bad robot id" }, 400);
    if (parts.length === 4 && (parts[3] === "status" || parts[3] === "turn")) {
      if (request.method !== "GET") return json({ error: "method not allowed" }, 405);
      return forwardToHub(request, env, robotId, `/${parts[3]}`);
    }
    // Phase 3: any method, forwarded to the robot's local HTTP (WHEP signaling).
    if (parts[3] === "http" && parts.length >= 5) {
      return forwardToHub(request, env, robotId, `/http/${parts.slice(4).join("/")}`, url.search);
    }
  }

  // /v1/relay/{app|robot}/{id}
  if (parts[1] === "relay" && parts.length === 4 && (parts[2] === "app" || parts[2] === "robot")) {
    const robotId = parts[3];
    if (!ROBOT_ID_RE.test(robotId)) return json({ error: "bad robot id" }, 400);
    if (request.headers.get("Upgrade")?.toLowerCase() !== "websocket") {
      return json({ error: "expected websocket upgrade" }, 426);
    }
    return forwardToHub(request, env, robotId, `/${parts[2]}`);
  }

  return json({ error: "not found" }, 404);
}

function forwardToHub(request: Request, env: Env, robotId: string, path: string, search = ""): Promise<Response> {
  const stub = env.ROBOT_HUB.getByName(robotId);
  const headers = new Headers(request.headers);
  headers.set("X-Hub-Robot-Id", robotId);
  const url = new URL(request.url);
  url.pathname = path;
  url.search = search;
  const hasBody = request.method !== "GET" && request.method !== "HEAD";
  return stub.fetch(new Request(url, { method: request.method, headers, body: hasBody ? request.body : null }));
}

interface RegisterBody {
  robot_id?: unknown;
  name?: unknown;
  model?: unknown;
  device_key?: unknown;
  pairing_secret?: unknown;
}

async function register(request: Request, env: Env): Promise<Response> {
  const auth = request.headers.get("Authorization") ?? "";
  const token = auth.startsWith("Bearer ") ? auth.slice(7).trim() : "";
  if (!env.PROVISION_TOKEN || !token || !(await secretsEqual(token, env.PROVISION_TOKEN))) {
    return json({ error: "bad provision token" }, 401);
  }
  const len = Number(request.headers.get("Content-Length") ?? "0");
  if (len > MAX_BODY) return json({ error: "body too large" }, 413);
  let body: RegisterBody;
  try {
    body = (await request.json()) as RegisterBody;
  } catch {
    return json({ error: "bad json" }, 400);
  }
  const robotId = str(body.robot_id);
  const deviceKey = str(body.device_key).toUpperCase();
  const pairingSecret = str(body.pairing_secret).toUpperCase();
  const name = str(body.name).slice(0, 64);
  const model = str(body.model).slice(0, 64);
  if (!ROBOT_ID_RE.test(robotId)) return json({ error: "bad robot_id" }, 400);
  if (!B32_KEY_RE.test(deviceKey)) return json({ error: "bad device_key" }, 400);
  if (!B32_KEY_RE.test(pairingSecret)) return json({ error: "bad pairing_secret" }, 400);

  const outcome = await registerRobot(env.DB, {
    robot_id: robotId,
    name,
    model,
    device_key: deviceKey,
    pairing_secret: pairingSecret,
  });
  if (outcome === "key_mismatch") {
    return json({ error: "robot_id already registered with another device_key" }, 409);
  }
  return json({ robot_id: robotId, outcome }, outcome === "created" ? 201 : 200);
}

function str(v: unknown): string {
  return typeof v === "string" ? v.trim() : "";
}

function json(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "Content-Type": "application/json" },
  });
}
