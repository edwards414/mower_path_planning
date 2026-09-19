// Cloudflare Realtime TURN: short-lived credentials for the phone and the
// robot's MediaMTX (docs/BACKEND_ARCHITECTURE.md §8). The TURN key itself
// never leaves the Worker; both ends only ever see generated credentials.

export interface IceServer {
  urls: string[];
  username?: string;
  credential?: string;
}

export interface IceServersResult {
  iceServers: IceServer[];
  /** unix seconds when the TURN credentials stop working; null = STUN only */
  expires_at: number | null;
}

export const STUN_ONLY: IceServer[] = [{ urls: ["stun:stun.cloudflare.com:3478"] }];
const DEFAULT_TTL_S = 86400;
const GENERATE_URL = "https://rtc.live.cloudflare.com/v1/turn/keys";

export function turnConfigured(env: Env): boolean {
  return Boolean(env.TURN_KEY_ID && env.TURN_KEY_API_TOKEN);
}

export function turnTtlS(env: Env): number {
  const n = Number(env.TURN_TTL_S || DEFAULT_TTL_S);
  return Number.isFinite(n) && n >= 60 ? Math.trunc(n) : DEFAULT_TTL_S;
}

/** POST .../credentials/generate-ice-servers; throws on any failure. */
export async function generateIceServers(env: Env, fetcher: typeof fetch = fetch): Promise<IceServersResult> {
  const ttl = turnTtlS(env);
  const res = await fetcher(`${GENERATE_URL}/${env.TURN_KEY_ID}/credentials/generate-ice-servers`, {
    method: "POST",
    headers: { Authorization: `Bearer ${env.TURN_KEY_API_TOKEN}`, "Content-Type": "application/json" },
    body: JSON.stringify({ ttl }),
  });
  if (res.status !== 201 && res.status !== 200) {
    throw new Error(`TURN credentials: HTTP ${res.status} ${(await res.text()).slice(0, 200)}`);
  }
  const body = (await res.json()) as { iceServers?: unknown };
  const servers = normalizeIceServers(body.iceServers);
  if (!servers.some((s) => s.username)) throw new Error("TURN credentials: no TURN entry in the response");
  return { iceServers: servers, expires_at: Math.trunc(Date.now() / 1000) + ttl };
}

/** Accepts both the `{urls, username, credential}` list and a single object. */
function normalizeIceServers(v: unknown): IceServer[] {
  const list = Array.isArray(v) ? v : v ? [v] : [];
  const out: IceServer[] = [];
  for (const item of list) {
    if (!item || typeof item !== "object") continue;
    const o = item as Record<string, unknown>;
    const urls = Array.isArray(o.urls) ? o.urls.filter((u) => typeof u === "string") : typeof o.urls === "string" ? [o.urls] : [];
    if (!urls.length) continue;
    const s: IceServer = { urls: urls as string[] };
    if (typeof o.username === "string") s.username = o.username;
    if (typeof o.credential === "string") s.credential = o.credential;
    out.push(s);
  }
  return out;
}
