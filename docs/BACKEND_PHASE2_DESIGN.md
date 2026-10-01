# 後台階段 2 設計：帳號、實體確認配對、每支手機金鑰、撤銷、一次性 provision token

狀態：**設計稿，待審，尚未實作**（2026-10-01）。上層架構見
[BACKEND_ARCHITECTURE.md](BACKEND_ARCHITECTURE.md)（§3 身分、§5 `mrelay1`、§7 API、§9 階段表）。
本文把 §9 的「階段 2」細化到可以直接動工的程度：資料表、API、訊息、機器人端行為、遷移順序、
測試，以及還需要你決定的事（§12）。

## 1. 目標與非目標

目標：

1. **貼紙不再帶 secret。** QR 只帶 `robot_id` 與一次性認領碼；拍到貼紙不等於能控制。
2. **每支手機一把獨立金鑰**（256 bit），可單獨撤銷，撤銷後連線立即斷開。
3. **配對要人在車旁按實體按鈕確認**，遠端無法單方面加入新手機。
4. **帳號**：擁有者登入後看到自己的機器人清單，管理已配對的手機。
5. **一次性 provision token**：每台機器人安裝時用一次即失效，取代全機隊共用的 `MOWER_PROVISION_TOKEN`。
6. `api_version` 升 3，舊 App（貼紙 secret）在過渡期內仍可用，到期後關閉。

非目標（本階段不做）：

- Ed25519 / 公鑰化（§3 已記為之後再做；仍用 HMAC 對稱金鑰）。
- 多擁有者 / 細緻權限（只有 owner 與 member 兩種角色，member 權限 = 能連線控制）。
- OTA 通道管理（屬階段 3 剩下的部分）。
- App 端 UI 實作（App 不在本 repo；本文定義它要呼叫的 API 與流程）。

## 2. 現況與必須先修的缺陷

| 項目 | 現況（階段 1） | 問題 |
|---|---|---|
| 手機驗證 | Hub `verifyApp` → `getClientKey`：先找該 `client_id` 未撤銷的列，**找不到就退回 `client_id='*'`（貼紙 secret）** | 一支手機自己的金鑰被撤銷後，查詢只排除 `revoked_at IS NULL` 的列，於是**退回貼紙 secret 仍能通過**。階段 1 沒有個別手機的列，所以目前沒有實際影響；階段 2 一上就會變成撤銷無效。修法見 §6.2 |
| 機器人本機閘門 | `rosbridge_auth_proxy.py` 與 `mower_ws_bridge/src/auth.rs` 只認 `identity.json` 的 `secret` 一把 | 本機 LAN 連線無法做到每支手機撤銷 |
| 註冊 | 全機隊共用 `MOWER_PROVISION_TOKEN`（`/opt/mower/.env`） | 任一台機器人外洩 token = 可註冊任意新 `robot_id` |
| 貼紙 | `…/pair?v=1&id=…&s=<secret>&…` | 誰拍到誰能控制；換 secret 要重印 |
| 撤銷 | `mower-pair --rotate` 換 secret，所有手機重掃 | 無法只踢掉一支 |

## 3. 身分模型

```
users ─1:N─ robot_members ─N:1─ robots ─1:N─ robot_clients
  (帳號)     (owner / member)              (每支手機一把 HMAC 金鑰)
```

- **user**：一個帳號（登入方式見 §4）。
- **robot_members**：`(robot_id, user_id, role)`，`role ∈ {owner, member}`。一台機器人恰好一個 owner。
  `robots.owner_id` 保留作快取（= role owner 的那位），查詢以 `robot_members` 為準。
- **robot_clients**：沿用現有表，新增 `user_id`（哪個帳號的手機）。`client_id` 是 App 安裝時產生的
  穩定 id（現有 `X-Mower-Client`）。金鑰由**後台產生**，只在配對成功的那次 HTTPS 回應給手機一次。
- 機器人仍以 `device_key` + `client=@robot` 撥入，不變。

## 4. 帳號與登入

建議：**Email 一次性碼（OTP）+ 後台簽發的 JWT**，不引入第三方身分服務。

1. `POST /v1/auth/email/start {email}` → 寄 6 位數碼（10 分鐘有效，最多 5 次嘗試，同 email 每分鐘 1 次）。
2. `POST /v1/auth/email/verify {email, code}` → `{access_token, refresh_token, user}`。首次驗證即建立 user。
3. `access_token`：JWT HS256，15 分鐘，`sub=user_id`；簽章金鑰 `JWT_SECRET`（wrangler secret，支援
   `JWT_SECRET_PREV` 輪替期間兩把都驗）。
4. `refresh_token`：256 bit 隨機值，D1 只存 SHA-256，30 天，**每次使用即換新**（rotation）；舊的再被用
   = 外洩跡象 → 撤銷該 session 整串。
5. `POST /v1/auth/logout` 撤銷目前 session。

寄信需要一個寄送服務（Cloudflare Email Sending / Resend / 其他），這是 §12 的待決事項。若你偏好
Sign in with Apple / Google，§5 以後的設計不受影響，只換掉 §4 這一節。

**JWT 只用於控制面 API**（清單、配對、撤銷）。連線控制機器人（relay、`/status`、`/turn`、`/http/*`）
仍用 X-Mower-* HMAC（每支手機的金鑰），所以機器人本機閘門不必懂帳號。

## 5. 配對流程（實體按鈕確認）

### 5.1 新貼紙 QR（`v=2`）

```
https://mower.fxrbindi.com/pair?v=2&id=MW-7K3Q9P&c=<claim code>&n=<name>
```

- `claim code`：80 bit、base32（16 字元），由 `mower-pair` 產生，存在機器人 `identity.json`
  （`claim_code`），註冊時以 SHA-256 雜湊送到後台（`robots.claim_hash`）。
- 認領碼**只證明「拍過這台車的貼紙」**，不能單獨完成配對；還要 §5.3 的按鈕確認。所以貼紙外洩的風險
  降成「有人能在你車旁按按鈕時發起配對」。
- 不再帶 `h`（relay URL）與 `l`（LAN IP）：App 由後台 API 取得 relay 位址，LAN IP 由心跳 `lan` 提供。

### 5.2 誰可以發起

| 情況 | 規則 |
|---|---|
| 機器人沒有 owner | 任何已登入帳號，帶正確 `claim_code` → 配對成功後成為 **owner** |
| 機器人有 owner，發起者就是 owner/member | 可以為自己的新手機配對（換手機、平板），帶 `claim_code` 或不帶皆可 |
| 機器人有 owner，發起者是其他帳號 | 需要 owner 先 `POST /v1/robots/{id}/invites` 產生邀請碼（24 h、一次性），發起時帶 `invite`；成功後成為 **member** |

所有情況都還要 §5.3 的實體按鈕確認。owner 轉移：`POST /v1/robots/{id}/transfer {user_id}`（owner 呼叫），
或「重設」流程：在車旁 `sudo mower-pair --reset-owner`（清空 owner、換 claim code、撤銷全部手機）。

### 5.3 時序

```
App (已登入)                    Worker / RobotHub                     機器人 agent             閘門 / 底盤
 │ POST /v1/robots/{id}/pair     │                                     │                         │
 │ {client_id, label, claim_code │                                     │                         │
 │  | invite}  (JWT)             │                                     │                         │
 │──────────────────────────────▶│ 驗 JWT、claim/invite、機器人在線     │                         │
 │                               │ 建 pairing_requests(pending, 60 s)  │                         │
 │◀── 202 {request_id, ttl:60} ──│ {"t":"pair_confirm","req","label",  │                         │
 │                               │  "ttl":60} ────────────────────────▶│ 提示：LED 藍色呼吸 + 嗶聲│
 │ GET …/pair/{request_id}       │                                     │ 等短按（§5.4）           │
 │ (長輪詢 ≤ 25 s，重試到 ttl)    │                                     │◀── /mower_base/telemetry │
 │                               │◀──── {"t":"pair_result","req",      │     power.press_ms       │
 │                               │       "ok":true} ───────────────────│                         │
 │                               │ confirmed：產生 256-bit key，寫      │                         │
 │                               │ robot_clients(user_id)、members     │                         │
 │                               │ {"t":"clients",…} ─────────────────▶│ 寫 clients.json（原子）  │
 │◀── 200 {state:"confirmed",    │                                     │ 閘門重新載入 ───────────▶│
 │     key, client_id, robot}    │                                     │                         │
```

- 金鑰**只在第一次讀到 confirmed 的回應中給出**，之後同一 `request_id` 再查只回狀態（`key_delivered=1`）。
- 逾時（60 s 沒按）→ `expired`；機器人回 `ok:false` → `denied`；機器人中途離線 → `expired`。
- 同一台機器人同時只允許一個 pending 請求（第二個回 409），避免按一次按鈕確認到別人的請求。
- `pair_confirm` 帶 `label`（例如「Edward 的 iPhone」），機器人可在 `/robot/info` 或 Qt 面板顯示
  「是否要配對：Edward 的 iPhone」，讓車旁的人知道自己在確認什麼。

### 5.4 按鈕語意（不改韌體）

機器人唯一的實體按鈕是電源鍵（`PB0`）。韌體語意：按住 ≥ 3 s（`POWER_MANAGER_SHUTDOWN_HOLD_MS`）
觸發關機；低功耗時按住 ≥ 1 s 喚醒。STM32 以 `0x86` Power Status 回報 `press_ms`（按住的毫秒數，放開為 0），
`mower_hardware` 已放進 `/mower_base/telemetry` 的 `power.press_ms`（mower_rs `mower_base` 相同）。

確認動作定義為：**pending 期間的一次短按**——觀察到 `press_ms` 由 0 上升、最高值落在 **200–1500 ms**，
然後回到 0。超過 1500 ms 不算確認（避免與關機長按混淆，3 s 前放開也不會關機）。
這段判斷放在機器人端一個小模組（建議放進 `robot_info`／`mower_agent` 都可，見 §7.3），不需要動韌體。

回饋：pending 時 LED 藍色呼吸 + 蜂鳴器一聲；確認成功兩短聲；逾時/拒絕一長聲。
（燈效與蜂鳴器已有 UART 命令可用。）

## 6. 撤銷

### 6.1 API

- `DELETE /v1/robots/{id}/clients/{client_id}`（owner：任何一支；member：只能撤自己的）。
- `DELETE /v1/robots/{id}/members/{user_id}`（owner）：撤銷該帳號的全部手機並移出 members。
- 使用者自己「登出並解除此手機」= 撤自己的 client。

### 6.2 效果

1. D1：`robot_clients.revoked_at = now`。
2. Hub：立刻關掉該 `client_id` 的所有 app socket（close code `4403 revoked`）；WHEP 代理也以 client 檢查。
3. Hub → 機器人：推新的 `{"t":"clients",…}`；機器人重寫 `clients.json`，本機閘門拒絕該手機的 **LAN** 連線，
   並關掉它已建立的本機連線。
4. **修 §2 的缺陷**：`getClientKey` 改為「該 `client_id` 有任何一列（含已撤銷）就只用那一列，已撤銷即拒絕；
   完全沒有列時才考慮 `'*'`，而且只在 legacy 模式（§9）開著時」。

### 6.3 機器人離線時的撤銷

機器人離線時 LAN 上被撤銷的手機仍能連到本機閘門，直到機器人下次連上後台同步 `clients.json`。緩解：

- agent 每次 relay 連線建立後，Hub 都先送一次完整 `clients`（不是差量），並帶 `version`（單調遞增）。
- `clients.json` 記 `synced_at`；`/robot/info` 回報 `clients_synced_at`，App 可提示「機器人離線，撤銷將於連線後生效」。
- 緊急情況：車旁 `sudo mower-pair --revoke <client_id>` 或 `--reset-owner` 直接改本機檔案。

## 7. 機器人端改動

### 7.1 `clients.json`（`~/.mower/clients.json`，0600，原子寫入）

```json
{"version": 17, "synced_at": 1790000000,
 "legacy_sticker": true,
 "clients": [{"client_id": "ios-3f2a…", "key": "<base32>", "label": "Edward 的 iPhone"}]}
```

- `legacy_sticker`：由後台決定（§9），true 時本機閘門仍接受 `identity.json` 的 `secret`（`client` 任意）。
- 檔案不存在 = 尚未同步過 = 等同只有 legacy（維持階段 1 行為），所以先上機器人端程式不會鎖死任何人。

### 7.2 本機閘門（兩份實作要同步）

`rosbridge_auth_proxy.py`（`identity.py` `verify_mac`）與 `mower_ws_bridge/src/auth.rs`：

1. 讀 `X-Mower-Client`；在 `clients.json` 找到該 client → 只用它的 key 驗。
2. 找不到 → `legacy_sticker` 為 true（或沒有 `clients.json`）時用 `secret` 驗，否則 401。
3. 檔案變動時重新載入（inotify 或每次連線前比對 mtime）；被移除的 client 的現有連線關閉（close 4403）。
4. 測試向量沿用 `docs/ROBOT_API.md`「配對」的那組，另加「per-client key」一組，三端（Python、Rust、Worker）共用。

### 7.3 agent（`mower_agent.py` 與 mower_rs `mower_agent`，兩份）

目前兩者收到 `clients` / `pair_confirm` 都只記 log（`mower_agent.py:444`、`mower_agent/src/lib.rs:530`）。要做：

- `clients` → 驗格式 → 原子寫 `clients.json`（version 不比現有新就忽略）。
- `pair_confirm` → 交給按鈕確認器（§5.4），結果回 `pair_result`；同時只處理一個。
- 按鈕確認器需要 `/mower_base/telemetry`：agent 已訂閱本機 rosbridge 的 `/robot/info`、`/robot/telemetry`
  做心跳，加訂 `/mower_base/telemetry`（只取 `power`）即可。燈效/蜂鳴器走既有 UART 命令的 topic。

### 7.4 `mower-pair`

- 新增 `claim_code`（安裝時產生，`--rotate-claim` 換新並重印貼紙）。
- `--json` 輸出 `v=2` 的 URL；`deploy/mower-pair-sheet.py` 跟著改。
- `--revoke <client_id>`、`--reset-owner`：改本機 `clients.json`，並在下次連線時以
  `{"t":"local_reset"}` 通知 Hub（Hub 照做撤銷／清 owner，記 event）。

## 8. 一次性 provision token

- 新表 `provision_tokens(token_hash, created_at, expires_at, used_at, used_by_robot, note)`。
- 管理端產生：`npx wrangler d1 execute` 腳本或一個 admin-only API（`ADMIN_TOKEN` secret 保護）
  `POST /v1/admin/provision-tokens {count, ttl_days, note}` → 回明文 token（只此一次）。
- `install.sh` 把 token 寫進 `/opt/mower/.env` 的 `MOWER_PROVISION_TOKEN`（與現在相同位置），
  agent 第一次 `register` 成功後**刪除該行**並在 `identity.json` 記 `registered_at`；之後重啟只靠
  `device_key` 撥入，不再呼叫 register（`device_key` 已在後台）。
- 換 `device_key`（重灌）需要新 token；同 `robot_id` 不同 `device_key` 仍回 409，除非 token 是 owner
  在 App 上為該 `robot_id` 產生的「重新安裝 token」（`POST /v1/robots/{id}/reinstall-token`，owner）。
- 過渡：`register` 同時接受舊的共用 `PROVISION_TOKEN`（env 存在時），§9 第 4 步移除。

## 9. 相容與遷移順序

| 步驟 | 內容 | 舊 App 影響 |
|---|---|---|
| 1 | D1 migration `0002`（§10）；修 `getClientKey`（§6.2-4）；Worker 加 auth、pair、revoke、invite API；`LEGACY_STICKER=1` | 無 |
| 2 | 機器人端：閘門讀 `clients.json`、agent 處理 `clients`/`pair_confirm`、按鈕確認器、`mower-pair` v2；`ROBOT_API_VERSION` 3 | 無（沒有 `clients.json` 或 `legacy_sticker=true` 時行為不變） |
| 3 | App 支援 v2 貼紙、登入、配對、撤銷；App 支援範圍 `api_version` 2–3 | 舊 App 照用貼紙 secret |
| 4 | 全部機器人升級後：換印 v2 貼紙、`mower-pair --rotate`（舊 secret 失效）、後台 `LEGACY_STICKER=0`（`clients` 帶 `legacy_sticker:false`）、移除共用 `PROVISION_TOKEN` | 舊 App 失效，需更新並重新配對 |

`api_version` 3 的 `/robot/info` 新增：`pairing_mode`（`legacy` / `per_client`）、`clients_synced_at`、
`pairing_pending`（`{label, expires_at}` 或 null）。

## 10. 資料表（`backend/migrations/0002_phase2.sql`）

```sql
ALTER TABLE robots ADD COLUMN claim_hash TEXT;            -- sha256(claim_code) hex
ALTER TABLE robot_clients ADD COLUMN user_id TEXT REFERENCES users(user_id);
ALTER TABLE robot_clients ADD COLUMN last_used_at INTEGER;

CREATE TABLE robot_members (
  robot_id TEXT NOT NULL REFERENCES robots(robot_id) ON DELETE CASCADE,
  user_id  TEXT NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
  role     TEXT NOT NULL CHECK (role IN ('owner','member')),
  created_at INTEGER NOT NULL,
  PRIMARY KEY (robot_id, user_id)
);
CREATE UNIQUE INDEX robot_one_owner ON robot_members(robot_id) WHERE role = 'owner';

CREATE TABLE sessions (                                    -- refresh tokens
  session_id TEXT PRIMARY KEY, user_id TEXT NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
  refresh_hash TEXT NOT NULL UNIQUE, created_at INTEGER NOT NULL, expires_at INTEGER NOT NULL,
  rotated_at INTEGER, revoked_at INTEGER, user_agent TEXT
);
CREATE TABLE email_codes (
  email TEXT PRIMARY KEY, code_hash TEXT NOT NULL, expires_at INTEGER NOT NULL,
  attempts INTEGER NOT NULL DEFAULT 0, last_sent_at INTEGER NOT NULL
);
CREATE TABLE invites (
  code_hash TEXT PRIMARY KEY, robot_id TEXT NOT NULL REFERENCES robots(robot_id) ON DELETE CASCADE,
  created_by TEXT NOT NULL, created_at INTEGER NOT NULL, expires_at INTEGER NOT NULL,
  used_by TEXT, used_at INTEGER
);
CREATE TABLE provision_tokens (
  token_hash TEXT PRIMARY KEY, created_at INTEGER NOT NULL, expires_at INTEGER NOT NULL,
  robot_id TEXT,                 -- set for a reinstall token bound to one robot
  used_at INTEGER, used_by_robot TEXT, note TEXT NOT NULL DEFAULT ''
);
-- pairing_requests (0001) gains who asked and whether the key was handed out
ALTER TABLE pairing_requests ADD COLUMN user_id TEXT;
ALTER TABLE pairing_requests ADD COLUMN key_delivered INTEGER NOT NULL DEFAULT 0;
ALTER TABLE robots ADD COLUMN clients_version INTEGER NOT NULL DEFAULT 0;
```

所有「碼／token」只存 SHA-256；比對用 `secretsEqual`（已有，常數時間）。

## 11. API 總表（新增）

| 方法與路徑 | 驗證 | 說明 |
|---|---|---|
| `POST /v1/auth/email/start` | 無（速率限制） | 寄 OTP |
| `POST /v1/auth/email/verify` | 無（速率限制） | → access / refresh token |
| `POST /v1/auth/refresh` | refresh token | 輪替 |
| `POST /v1/auth/logout` | JWT | 撤銷 session |
| `GET /v1/me/robots` | JWT | `[{robot_id, name, role, online, last_seen, my_clients:[…]}]` |
| `POST /v1/robots/{id}/pair` | JWT | §5.3，`{client_id, label, claim_code?, invite?}` → 202 |
| `GET /v1/robots/{id}/pair/{request_id}` | JWT | 長輪詢；confirmed 首次帶 `key` |
| `GET /v1/robots/{id}/clients` | JWT（owner 全部、member 自己） | 已配對手機清單（不含 key） |
| `DELETE /v1/robots/{id}/clients/{client_id}` | JWT | §6 |
| `POST /v1/robots/{id}/invites` | JWT owner | → `{invite, expires_at}` |
| `DELETE /v1/robots/{id}/members/{user_id}` | JWT owner | 移除成員與其手機 |
| `POST /v1/robots/{id}/transfer` | JWT owner | 轉移 owner |
| `POST /v1/robots/{id}/reinstall-token` | JWT owner | §8 |
| `POST /v1/admin/provision-tokens` | `ADMIN_TOKEN` | §8 |

新的 Hub 控制訊息：`clients` 加 `version`、`legacy_sticker`；`pair_confirm` / `pair_result`（§5.3）；
機器人→Hub `{"t":"local_reset","what":"revoke|owner","client_id"?}`；Hub→手機 close code `4403`（revoked）。

## 12. 待你決定

1. **登入方式**：Email OTP（建議，需選寄信服務：Cloudflare Email Sending / Resend / 其他）或 Apple/Google 登入。
2. **離線配對**：機器人沒有網路時要不要能配對？本設計配對必經後台（才能有帳號與撤銷）；離線只能用
   legacy 貼紙（過渡期）。若要長期支援純 LAN 機器人，需要另一套本機配對（車旁按鈕 + 本機產生金鑰），
   工作量約 +30 %。
3. **member 角色**要不要限制（例如 member 不能 `/system/update`、不能改區域）？目前設計 member = 完整控制。
4. **legacy 截止**：第 4 步（舊 App 失效）的時間點。
5. **按鈕確認的回饋**：LED/蜂鳴器的樣式，或改用 Qt 面板／App 以外的另一個確認方式。

## 13. 測試計畫

- Worker（vitest，沿用 `backend/test`）：OTP 流程與速率限制、JWT 驗簽與輪替、refresh 重用偵測、
  配對狀態機（pending → confirmed/denied/expired、單一 pending、key 只交付一次）、撤銷即斷線（4403）、
  `getClientKey` 不再退回 `'*'`、invite / transfer / members 權限矩陣、一次性 provision token 重用 409。
- 三端共用 HMAC 測試向量加「per-client key」一組（`test_pairing.py`、`hmac.test.ts`、App `pairing_test.dart`、
  `mower_ws_bridge` auth 測試）。
- 機器人端：`clients.json` 解析/原子寫/版本比較；閘門 per-client 與 legacy 規則（Python 與 Rust 同一組案例）；
  按鈕確認器用錄好的 `press_ms` 序列（短按、長按、按住不放、逾時）做單元測試。
- 端到端：`tools/` 加一支類似 `coverage_cancel_check.py` 的黑箱腳本：本機起 `wrangler dev` + agent +
  假底盤（`fake_base.py` 送 `press_ms`），跑「配對 → 連線 → 撤銷 → 被斷線 → 重新配對」。

## 14. 工作量估計

| 部分 | 估計 |
|---|---|
| Worker：auth、配對、撤銷、invite、provision、migration、測試 | 4–5 天 |
| 機器人端：閘門 ×2、agent ×2、按鈕確認器、`mower-pair` v2、測試 | 3–4 天 |
| App：登入、v2 貼紙、配對/撤銷 UI、HMAC 改 per-client | 依 App 現況，另估 |
| 端到端黑箱與上機驗證 | 1–2 天 |
