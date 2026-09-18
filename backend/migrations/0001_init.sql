-- Fleet registry. See docs/BACKEND_ARCHITECTURE.md.

CREATE TABLE IF NOT EXISTS robots (
  robot_id   TEXT PRIMARY KEY,            -- MW-XXXXXX
  name       TEXT NOT NULL DEFAULT '',
  model      TEXT NOT NULL DEFAULT '',
  device_key TEXT NOT NULL,               -- base32, HMAC key of the robot's relay connection
  owner_id   TEXT,                        -- users.user_id, NULL until claimed (phase 2)
  created_at INTEGER NOT NULL,
  updated_at INTEGER NOT NULL,
  last_seen  INTEGER,                     -- unix s of the last heartbeat written back by the hub
  lan        TEXT,                        -- last LAN address reported by the robot
  info       TEXT                         -- last /robot/info JSON (versions), for the fleet list
);

-- One HMAC key per phone. client_id '*' is the sticker secret from identity.json
-- (phase 1 compatibility: any phone that scanned the sticker).
CREATE TABLE IF NOT EXISTS robot_clients (
  robot_id   TEXT NOT NULL REFERENCES robots(robot_id) ON DELETE CASCADE,
  client_id  TEXT NOT NULL,
  key        TEXT NOT NULL,               -- base32
  label      TEXT NOT NULL DEFAULT '',
  created_at INTEGER NOT NULL,
  revoked_at INTEGER,
  PRIMARY KEY (robot_id, client_id)
);

-- Phase 2: owner accounts.
CREATE TABLE IF NOT EXISTS users (
  user_id    TEXT PRIMARY KEY,
  email      TEXT NOT NULL UNIQUE,
  created_at INTEGER NOT NULL
);

-- Phase 2: a phone asked to be paired; the robot confirms with a button press.
CREATE TABLE IF NOT EXISTS pairing_requests (
  request_id TEXT PRIMARY KEY,
  robot_id   TEXT NOT NULL REFERENCES robots(robot_id) ON DELETE CASCADE,
  client_id  TEXT NOT NULL,
  label      TEXT NOT NULL DEFAULT '',
  state      TEXT NOT NULL,               -- pending | confirmed | denied | expired
  created_at INTEGER NOT NULL,
  expires_at INTEGER NOT NULL
);

-- Audit trail: connections, pairings, revocations.
CREATE TABLE IF NOT EXISTS events (
  id        INTEGER PRIMARY KEY AUTOINCREMENT,
  ts        INTEGER NOT NULL,
  robot_id  TEXT,
  client_id TEXT,
  kind      TEXT NOT NULL,
  detail    TEXT
);
CREATE INDEX IF NOT EXISTS events_robot_ts ON events(robot_id, ts);
