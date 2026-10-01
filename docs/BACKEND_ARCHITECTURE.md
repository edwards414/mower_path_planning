# 後台架構（設備註冊、配對、遠端中繼）

狀態：規劃定案，實作進行中（2026-09-18）。程式碼在 `backend/`（Cloudflare Worker）與
`src/mower_mission/mower_mission/mower_agent.py`（機器人端）。

## 1. 為什麼要有後台

現況的遠端路徑是「機器人 → WireGuard `10.77.0.2` → 家裡 Ubuntu 主機 → cloudflared →
`control.fxrbindi.com`」，只對得上一台機器人，App 內還編譯了一組 Cloudflare service token。
配對貼紙的 QR 直接帶 HMAC secret，誰拍到貼紙誰就能控制，且無法只撤銷某一支手機。

4G SIM 卡在電信商 CGNAT 後面，手機永遠無法主動連進機器人。所以後台**不管 IP**，而是：

1. **設備註冊表**：`robot_id` ↔ 裝置金鑰、名稱、擁有者、版本。
2. **在線狀態**：機器人主動撥出一條長連線並每 10 秒心跳，後台因此知道誰在線、最後位置與 LAN IP。
3. **配對與授權**：每支手機一把獨立金鑰，可單獨撤銷；貼紙 QR 最終只帶 `robot_id` 與認領碼。
4. **中繼**：手機與機器人都撥入後台，後台對接 rosbridge 訊框。機器人本機的
   `rosbridge_auth_proxy` 保留，仍是最後一道閘門，機器人不必信任雲端。

## 2. 整體形狀

```
手機 App ──HTTPS──▶ Worker  /v1/robots/{id}/status  …（控制面 API，D1）
手機 App ──WSS───▶ Worker  /v1/relay/app/{id} ───┐
                                                  ├─▶ RobotHub Durable Object（每台一個）
機器人 mower-agent ──WSS 主動撥出──▶ /v1/relay/robot/{id} ┘
        │
        ├─▶ ws://127.0.0.1:9090  rosbridge_auth_proxy（驗 X-Mower-* 標頭）──▶ rosbridge 127.0.0.1:9091
        └─▶ ws://127.0.0.1:9091  訂閱 /robot/info、/robot/telemetry 做心跳
```

平台：Cloudflare Workers + Durable Objects（WebSocket Hibernation）+ D1。
理由：rosbridge 是純 JSON WebSocket，「一台機器人一個 Durable Object」剛好對應；兩邊都撥入，
不開任何入站埠，沒有伺服器要維護；`mower.fxrbindi.com` 已在 Cloudflare 上。

影像：同一個 Wi-Fi 直連機器人上的 MediaMTX；跨網路時 WHEP 訊令經後台轉到機器人、媒體走
Cloudflare TURN，見 §8。WireGuard 與家裡主機不再參與。

## 3. 身分與金鑰

| 對象 | 金鑰 | 存放 | 用途 |
|---|---|---|---|
| 機器人 | `device_key`（256 bit，base32） | 機器人 `identity.json`；後台 `robots.device_key` | agent 撥入 relay 時的 HMAC |
| 貼紙 secret（現有） | `secret`（160 bit，base32） | 機器人 `identity.json`；後台 `robot_clients` 的 `client_id='*'` | 階段 1 相容：任何掃過貼紙的手機 |
| 每支手機（階段 2） | 256 bit | 手機 keychain；後台 `robot_clients`；機器人 `clients.json` | 每支手機獨立驗證與撤銷 |

HMAC 格式沿用 `docs/ROBOT_API.md` 的「配對」：
`X-Mower-Robot / Client / Time / Nonce / Mac`，`mac = HMAC-SHA256(key, "robot_id\nclient\ntime\nnonce")`，
時間差 ±60 s，nonce 不可重放。機器人撥入時 `client` 固定為 `@robot`，key 為 `device_key`。
Worker 與機器人用同一組測試向量（`test/test_pairing.py`、`backend/test/hmac.test.ts`）。

金鑰以明文存 D1（Cloudflare 靜態加密）。之後可改 Ed25519 讓後台只存公鑰，agent 端需要
`cryptography` 套件，目前映像沒有，先不做。

## 4. 註冊

機器人 agent 每次啟動先 `POST /v1/robots/register`，`Authorization: Bearer <MOWER_PROVISION_TOKEN>`：

```json
{"robot_id": "MW-7K3Q9P", "name": "lubancat", "device_key": "<base32>",
 "pairing_secret": "<identity.json 的 secret>", "model": "lubancat-v1"}
```

Upsert：`robot_id` 不存在就建立；存在且 `device_key` 相同就更新名稱與 secret；`device_key` 不同回 409
（防止別台冒用同一個 id）。`pairing_secret` 寫進 `robot_clients(robot_id, '*')`，所以 `mower-pair --rotate`
之後 agent 重啟即同步。`MOWER_PROVISION_TOKEN` 放 `/opt/mower/.env`（root 可讀），是全機隊共用的
安裝密鑰；階段 2 改成一次性 token。

## 5. Relay 協定 `mrelay1`

兩條 WebSocket 都以 `Sec-WebSocket-Protocol: mrelay1` 協商。Workers 單一 WebSocket 訊息上限 1 MiB，
rosbridge 的地圖訊息可到數十 MB，所以**分片在兩端做，Hub 只換頭不重組**，Hub 不需要為訊框保存狀態。

### 5.1 手機 ↔ Hub（`/v1/relay/app/{robot_id}`）

升級請求帶 X-Mower-* 標頭。Hub 驗證通過且機器人在線才回 101；機器人離線回 503，驗證失敗回 401。
連上後每個 WebSocket 訊息是二進位：

```
byte 0  type   0x01 文字（最後一片）  0x11 文字（還有下一片）
               0x02 二進位（最後一片） 0x12 二進位（還有下一片）
byte 1… payload（rosbridge 訊框的一段，分片大小 ≤ 512 KiB）
```

App 端把連續的 0x11 片段串起來直到 0x01，再交給既有的 rosbridge 解析；送出時同樣切片。
LAN 直連時不用這層，App 依連線路徑決定是否套用。

### 5.2 機器人 ↔ Hub（`/v1/relay/robot/{robot_id}`）

升級請求帶 X-Mower-* 標頭（`client=@robot`，key=`device_key`）。控制訊息是 JSON 文字訊框，
資料訊框是二進位，比 5.1 多 8 bytes 的 session id：

```
byte 0     type（同 5.1）
byte 1..8  sid（Hub 產生的 64-bit 隨機 id）
byte 9…    payload
```

控制訊息：

| 方向 | 訊息 | 說明 |
|---|---|---|
| 機器人→Hub | `{"t":"hb","info":{…},"telemetry":{…},"lan":"192.168.1.5"}` | 每 10 s；`info` 是 `/robot/info`，`telemetry` 是 `/robot/telemetry` 的摘要 |
| Hub→機器人 | `{"t":"open","sid":"<16 hex>","headers":{"X-Mower-Robot":…}}` | 手機來了，請對本機 `:9090` 開一條連線並帶上這些標頭 |
| 機器人→Hub | `{"t":"opened","sid"}` / `{"t":"open_err","sid","code":401,"reason":"…"}` | 本機 auth proxy 的結果；Hub 據此關掉手機端（close code 4401） |
| 兩邊 | `{"t":"close","sid","code":1000,"reason":""}` | 任一端關閉 session |
| Hub→機器人 | `{"t":"clients","clients":[{"client_id","key","label"}]}` | 階段 2：同步每支手機的金鑰到 `clients.json` |
| Hub→機器人 | `{"t":"pair_confirm","req":"…","label":"…","ttl":60}` → `{"t":"pair_result","req","ok":true}` | 階段 2：實體按鍵確認 |
| Hub→機器人 | `{"t":"http","rid","method","path":"/front/whep","headers":{…},"body_b64":"…"}` | §8：手機的 WHEP 請求；agent 只轉 `/<path>/whep[/<session>]` 到 `127.0.0.1:8889` |
| 機器人→Hub | `{"t":"http_res","rid","status":201,"headers":{"Content-Type","Location","ETag"},"body_b64":"…"}` | 上一則的回應；Hub 把以 `/` 開頭的 `Location` 改寫到 `/v1/robots/{id}/http` 之下 |
| Hub→機器人 | `{"t":"http","rid","method","path","headers","body_b64"}` → `{"t":"http_res","rid","status","headers","body_b64"}` | 階段 3：WHEP 信令轉發 |

Hub 送 `open` 後不等 `opened` 就開始轉送手機的訊框；agent 把該 sid 的訊框先排隊，等本機連線建立再送。
機器人斷線時 Hub 以 1012 關掉所有手機連線；機器人重連時舊的 robot socket 被關掉。

## 6. Durable Object `RobotHub`

`getByName(robot_id)`，一台一個。內容：

- `fetch()`：`/robot` 與 `/app` 兩種升級；驗 HMAC（金鑰從 D1 查，nonce 存 DO 的 SQLite 表，時間窗 120 s）。
- WebSocket Hibernation：`acceptWebSocket(ws, ['robot'])` / `acceptWebSocket(ws, ['app', 's:<sid>'])`，
  每條連線的 sid 與 client_id 用 `serializeAttachment` 保存，休眠後可恢復。
- 訊框轉送：手機→機器人加 sid，機器人→手機拆 sid，用 tag 找對應 socket。
- 心跳：寫 `last_hb`、`last_seen` 到 DO storage；每 60 s 才回寫一次 D1 `robots.last_seen/lan/info`；
  `setAlarm(now + 35 s)`，鬧鐘到了沒新心跳就標離線。
- RPC `status()` → `{online, last_seen, lan, info, telemetry}` 給 HTTP API 用。

## 7. HTTP API（`/v1`）

| 方法與路徑 | 驗證 | 說明 |
|---|---|---|
| `GET /v1/health` | 無 | 存活 |
| `POST /v1/robots/register` | provision token | §4 |
| `GET /v1/robots/{id}/status` | X-Mower-* | 在線狀態與最近心跳摘要 |
| `GET /v1/relay/app/{id}` | X-Mower-* | WebSocket 升級，§5.1 |
| `GET /v1/relay/robot/{id}` | X-Mower-*（`@robot`） | WebSocket 升級，§5.2 |
| 階段 2：`POST /v1/auth/…`、`GET /v1/me/robots`、`POST /v1/robots/{id}/pair`、`…/clients/{cid}/revoke` | JWT | 帳號、清單、配對、撤銷 |
| `ANY /v1/robots/{id}/http/*` | X-Mower-* | 轉到機器人本機的 MediaMTX（WHEP 訊令，§8）；機器人離線 503、15 s 沒回 504 |
| `GET /v1/robots/{id}/turn` | X-Mower-*（手機或 `@robot`） | `{"iceServers":[…],"expires_at":…}`：Cloudflare TURN 短效帳密；沒設 TURN key 時只有 STUN、`expires_at` 為 null |

D1 資料表（`backend/migrations/0001_init.sql`）：`robots`、`robot_clients`、`users`、`pairing_requests`、`events`。

## 8. 影像（階段 3，已實作）

手機永遠與機器人上的 MediaMTX 建 WebRTC，只是訊令與媒體的路徑依連線方式不同：

| | 同一個 Wi-Fi（route=lan） | 跨網路（route=relay） |
|---|---|---|
| WHEP 訊令 | `http://<lan ip>:8889/front/whep` | `POST /v1/robots/{id}/http/front/whep`（X-Mower-*）→ Hub `{"t":"http"}` → agent → `127.0.0.1:8889` |
| ICE servers | 無（host candidate） | 手機 `GET /v1/robots/{id}/turn`；機器人 agent 每小時同一端點 → `PATCH 127.0.0.1:9997/v3/config/global/patch` `webrtcICEServers2` |
| 媒體 | 直連 UDP 8189 | ICE 選：直連可通就直連，否則 Cloudflare TURN |

TURN 用 Cloudflare Realtime TURN（每月 1,000 GB 免費，之後每 GB 0.05 美元，只算 TURN 送到客戶端
的方向；2 Mbit/s 約 0.9 GB/小時）。後台以 `TURN_KEY_ID` / `TURN_KEY_API_TOKEN`（wrangler secret）
向 `rtc.live.cloudflare.com` 取 `TURN_TTL_S`（預設 86400）秒的帳密，每台機器人的 Hub 快取一份、
過半壽命才換新，所以 MediaMTX（改 ICE 設定會重啟 WebRTC listener）一天只重載約兩次。
agent 只把 `turn:…3478?transport=udp|tcp` 兩條寫進 MediaMTX；手機拿完整清單（含 `turns:443`）。
`deploy/mediamtx.yml` 現在只有一份（WHEP :8889、API 只綁 127.0.0.1:9997），`mediamtx.lan.yml` 已移除。

安全邊界：agent 只轉發 `^/[a-z0-9_-]+/whep(/<session>)?$` 的 GET/POST/PATCH/DELETE/OPTIONS，
其他（WHIP 發布、API）一律 403——MediaMTX 信任 loopback 發布者，而 agent 就是 loopback。
Hub 只帶 `Content-Type / Accept / If-Match` 過去、只帶 `Content-Type / Location / ETag / Accept-Patch / Link` 回來，body 上限 64 KiB。

## 9. 階段

| 階段 | 交付 | 效果 |
|---|---|---|
| 0 | D1 註冊表、agent 註冊與心跳、`status` API | App 能看多台在線狀態，遠端連線仍走舊路 |
| 1 | RobotHub 中繼、agent 對接本機 auth proxy、App 走 relay 與 `mrelay1` 分片 | 拿掉 `control.fxrbindi.com → 10.77.0.2`，多台皆可遠端 |
| 2 | 帳號、實體確認配對、每支手機獨立金鑰、撤銷、一次性 provision token（設計稿：[BACKEND_PHASE2_DESIGN.md](BACKEND_PHASE2_DESIGN.md)，待審） | 貼紙不再帶 secret，api_version 升 3 |
| 3 | WHEP 加 TURN（已實作，§8）、後台管 OTA 通道 | WireGuard 與家裡主機退役 |

階段 0 與 1 的程式碼一起做（同一個 Worker、同一個 agent），部署時先只開心跳也可以。

## 10. 機器人端與 App 端改動

- `mower_agent`（`mower_mission` 套件，console script，`mower_bringup/launch/rosbridge.launch.py` 帶起）：
  讀 `identity.json`，第一次啟動產生 `device_key` 寫回；註冊；維持 relay 連線（指數退避重連）；
  心跳；依 `open` 開本機 session 並轉送；沒有 `MOWER_BACKEND_URL` 時什麼都不做（模擬、開發）。
- `/opt/mower/.env` 新增 `MOWER_BACKEND_URL=https://api.mower.fxrbindi.com`、`MOWER_PROVISION_TOKEN=`；
  `docker-compose.yaml` 傳進容器。
- App：`RosbridgeService` 增加「relay 模式」的 `mrelay1` 分片層；「我的機器人」以 `status` API 顯示在線；
  連線順序 LAN 先（心跳回報的 LAN IP，或 QR 的 `l`），失敗再 relay。`PAIR_RELAY_URL` 改指到
  `wss://api.mower.fxrbindi.com/v1/relay/app/<robot_id>`。移除 Dart define 的 Cloudflare service token。

## 11. 部署

```bash
cd backend
npm install
npx wrangler login                       # 一次
npx wrangler d1 create mower             # 把 database_id 填進 wrangler.jsonc
npx wrangler d1 migrations apply mower --remote
npx wrangler secret put PROVISION_TOKEN
npx wrangler deploy                      # 自訂網域 api.mower.fxrbindi.com 在 wrangler.jsonc routes
```

本機開發：`npm run dev`（miniflare，D1 與 DO 都在本機），`npm test`（vitest，跑在 workerd 裡）。

已部署（2026-09-18）：`https://api.mower.fxrbindi.com`，D1 `mower`。兩個實務注意事項：
多層子網域的憑證在 deploy 後約 5 分鐘才簽好，之前 TLS 會失敗；zone 的 Browser Integrity Check
會擋 `Python-urllib` 這個 User-Agent（403，error 1010），所以 agent 用 `mower-agent/<api> (<robot_id>)`，
任何新的 Python 客戶端都要自帶 User-Agent。
