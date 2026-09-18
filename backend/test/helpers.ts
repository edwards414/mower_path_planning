import { SELF } from "cloudflare:test";
import { signHeaders } from "../src/hmac";

export const ROBOT = "MW-7K3Q9P";
export const DEVICE_KEY = "MFRGGZDFMZTWQ2LKNNWG23TPOBYXE43UOV3HO6DZ";
export const STICKER = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
export const PROVISION = "test-provision-token";
export const BASE = "https://api.test";

export async function register(robotId = ROBOT, deviceKey = DEVICE_KEY): Promise<Response> {
  return SELF.fetch(`${BASE}/v1/robots/register`, {
    method: "POST",
    headers: { Authorization: `Bearer ${PROVISION}`, "Content-Type": "application/json" },
    body: JSON.stringify({
      robot_id: robotId,
      name: "bench",
      model: "lubancat-v1",
      device_key: deviceKey,
      pairing_secret: STICKER,
    }),
  });
}

export async function appHeaders(client = "iphone-1234", key = STICKER): Promise<Record<string, string>> {
  return signHeaders(key, ROBOT, client);
}

export async function robotHeaders(key = DEVICE_KEY): Promise<Record<string, string>> {
  return signHeaders(key, ROBOT, "@robot");
}

export async function connect(path: string, headers: Record<string, string>): Promise<Response> {
  return SELF.fetch(`${BASE}${path}`, {
    headers: { ...headers, Upgrade: "websocket", "Sec-WebSocket-Protocol": "mrelay1" },
  });
}

/** Client side of a 101 response with buffered messages (no lost events). */
export class Sock {
  readonly ws: WebSocket;
  private queue: (string | ArrayBuffer)[] = [];
  private waiters: ((m: string | ArrayBuffer) => void)[] = [];
  readonly closed: Promise<{ code: number; reason: string }>;

  constructor(res: Response) {
    const ws = res.webSocket;
    if (!ws) throw new Error(`no websocket on ${res.status} response`);
    this.ws = ws;
    ws.binaryType = "arraybuffer";
    this.closed = new Promise((resolve) => {
      ws.addEventListener("close", (ev) => resolve({ code: ev.code, reason: ev.reason }), { once: true });
    });
    ws.addEventListener("message", (ev) => {
      const data = ev.data as string | ArrayBuffer;
      const w = this.waiters.shift();
      if (w) w(data);
      else this.queue.push(data);
    });
    ws.accept();
  }

  send(data: string | Uint8Array): void {
    this.ws.send(data);
  }

  close(code = 1000, reason = ""): void {
    this.ws.close(code, reason);
  }

  next(timeoutMs = 2000): Promise<string | ArrayBuffer> {
    const queued = this.queue.shift();
    if (queued !== undefined) return Promise.resolve(queued);
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error("timeout waiting for message")), timeoutMs);
      this.waiters.push((m) => {
        clearTimeout(timer);
        resolve(m);
      });
    });
  }

  async nextJson<T>(): Promise<T> {
    return JSON.parse((await this.next()) as string) as T;
  }

  async nextBytes(): Promise<Uint8Array> {
    return new Uint8Array((await this.next()) as ArrayBuffer);
  }

  async closeCode(timeoutMs = 2000): Promise<number> {
    const timer = new Promise<never>((_, reject) =>
      setTimeout(() => reject(new Error("timeout waiting for close")), timeoutMs),
    );
    return (await Promise.race([this.closed, timer])).code;
  }
}

export const sleep = (ms: number): Promise<void> => new Promise((r) => setTimeout(r, ms));
