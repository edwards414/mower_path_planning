# mower-backend

Fleet backend for the mower: robot registry, presence and the app ↔ robot
relay. Cloudflare Worker + one `RobotHub` Durable Object per robot + D1.
Design and protocol: [`../docs/BACKEND_ARCHITECTURE.md`](../docs/BACKEND_ARCHITECTURE.md).

```
src/index.ts   routing: /v1/health, /v1/robots/register, /v1/robots/{id}/status, /v1/relay/{app|robot}/{id}
src/hub.ts     RobotHub DO: HMAC gate, robot + app sockets, mrelay1 pass-through, heartbeat/presence
src/hmac.ts    X-Mower-* headers (same test vector as the robot and the app)
src/frames.ts  mrelay1 framing constants and sid add/strip
src/db.ts      every D1 query
migrations/    D1 schema
test/          vitest (@cloudflare/vitest-plugin): hmac, api, relay
```

## Develop

`node_modules` and `.wrangler` are symlinks into `~/.cache/mower-backend` so
iCloud never syncs them; recreate them if missing:

```bash
mkdir -p ~/.cache/mower-backend/node_modules ~/.cache/mower-backend/.wrangler
ln -sfn ~/.cache/mower-backend/node_modules node_modules
ln -sfn ~/.cache/mower-backend/.wrangler .wrangler
npm install
cp .dev.vars.example .dev.vars          # PROVISION_TOKEN for `npm run dev`
npm run types                           # worker-configuration.d.ts (gitignored)
npm test                                # vitest, in workerd
npm run dev                             # http://localhost:8787
npx wrangler d1 migrations apply mower --local
```

A robot against the local server: `MOWER_BACKEND_URL=http://<mac-ip>:8787
MOWER_PROVISION_TOKEN=dev-provision-token` in `/opt/mower/.env`, then
`mower_agent` registers and connects. An app QR for it carries
`h=ws://<mac-ip>:8787/v1/relay/app`.

## Deploy (once per account)

```bash
npx wrangler login
npx wrangler d1 create mower            # copy database_id into wrangler.jsonc
npx wrangler d1 migrations apply mower --remote
npx wrangler secret put PROVISION_TOKEN # the value robots put in /opt/mower/.env
# Camera across networks (docs/BACKEND_ARCHITECTURE.md §8): a Cloudflare
# Realtime TURN key (dashboard -> Realtime -> TURN -> Create). Without these
# the /turn endpoint answers STUN only and video works on the LAN only.
npx wrangler secret put TURN_KEY_ID
npx wrangler secret put TURN_KEY_API_TOKEN
npx wrangler deploy                     # route api.mower.fxrbindi.com (custom_domain)
```

Deployed 2026-09-18 to `https://api.mower.fxrbindi.com` (D1 `mower`). The
certificate for a multi-level custom domain takes a few minutes after the
first deploy; until then TLS fails. The zone's Browser Integrity Check
rejects the `Python-urllib` User-Agent (403, error 1010): `mower_agent`
sends `mower-agent/<api> (<robot_id>)`, and any other Python client must set
its own User-Agent too.

Then on each robot: `MOWER_BACKEND_URL=https://api.mower.fxrbindi.com`,
`MOWER_PROVISION_TOKEN=…`, `PAIR_RELAY_URL=wss://api.mower.fxrbindi.com/v1/relay/app`
in `/opt/mower/.env`, restart, `sudo mower-pair` for a fresh QR.

## Operations

```bash
npx wrangler tail                                   # live logs
npx wrangler d1 execute mower --remote --command "SELECT robot_id,name,last_seen,lan FROM robots"
npx wrangler d1 execute mower --remote --command "SELECT ts,robot_id,client_id,kind,detail FROM events ORDER BY ts DESC LIMIT 50"
```
