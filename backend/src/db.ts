// D1 access. Keep every query here so the schema has one home.

export interface RobotRow {
  robot_id: string;
  name: string;
  model: string;
  device_key: string;
  owner_id: string | null;
  created_at: number;
  updated_at: number;
  last_seen: number | null;
  lan: string | null;
  info: string | null;
}

export interface ClientRow {
  robot_id: string;
  client_id: string;
  key: string;
  label: string;
  created_at: number;
  revoked_at: number | null;
}

export const nowS = (): number => Math.trunc(Date.now() / 1000);

export async function getRobot(db: D1Database, robotId: string): Promise<RobotRow | null> {
  return db.prepare("SELECT * FROM robots WHERE robot_id = ?").bind(robotId).first<RobotRow>();
}

/** The key for one phone, falling back to the sticker secret ('*'). */
export async function getClientKey(
  db: D1Database,
  robotId: string,
  clientId: string,
): Promise<ClientRow | null> {
  const own = await db
    .prepare("SELECT * FROM robot_clients WHERE robot_id = ? AND client_id = ? AND revoked_at IS NULL")
    .bind(robotId, clientId)
    .first<ClientRow>();
  if (own) return own;
  return db
    .prepare("SELECT * FROM robot_clients WHERE robot_id = ? AND client_id = '*' AND revoked_at IS NULL")
    .bind(robotId)
    .first<ClientRow>();
}

export interface RegisterInput {
  robot_id: string;
  name: string;
  model: string;
  device_key: string;
  pairing_secret: string;
}

export type RegisterOutcome = "created" | "updated" | "key_mismatch";

export async function registerRobot(db: D1Database, r: RegisterInput): Promise<RegisterOutcome> {
  const t = nowS();
  const existing = await getRobot(db, r.robot_id);
  if (existing && existing.device_key !== r.device_key) return "key_mismatch";
  const stmts = [
    existing
      ? db
          .prepare("UPDATE robots SET name = ?, model = ?, updated_at = ? WHERE robot_id = ?")
          .bind(r.name, r.model, t, r.robot_id)
      : db
          .prepare(
            "INSERT INTO robots (robot_id, name, model, device_key, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?)",
          )
          .bind(r.robot_id, r.name, r.model, r.device_key, t, t),
    db
      .prepare(
        "INSERT INTO robot_clients (robot_id, client_id, key, label, created_at) VALUES (?, '*', ?, 'sticker', ?) " +
          "ON CONFLICT(robot_id, client_id) DO UPDATE SET key = excluded.key, revoked_at = NULL",
      )
      .bind(r.robot_id, r.pairing_secret, t),
    db
      .prepare("INSERT INTO events (ts, robot_id, kind, detail) VALUES (?, ?, ?, ?)")
      .bind(t, r.robot_id, existing ? "robot.updated" : "robot.registered", r.name),
  ];
  await db.batch(stmts);
  return existing ? "updated" : "created";
}

export async function writeRobotSeen(
  db: D1Database,
  robotId: string,
  lastSeen: number,
  lan: string | null,
  info: unknown,
): Promise<void> {
  await db
    .prepare("UPDATE robots SET last_seen = ?, lan = ?, info = ? WHERE robot_id = ?")
    .bind(lastSeen, lan, info === undefined ? null : JSON.stringify(info), robotId)
    .run();
}

export async function logEvent(
  db: D1Database,
  robotId: string | null,
  clientId: string | null,
  kind: string,
  detail?: string,
): Promise<void> {
  await db
    .prepare("INSERT INTO events (ts, robot_id, client_id, kind, detail) VALUES (?, ?, ?, ?, ?)")
    .bind(nowS(), robotId, clientId, kind, detail ?? null)
    .run();
}
