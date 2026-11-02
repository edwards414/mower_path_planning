"""The outdoor harness's hard check: never the real robot.

Every tool in tools/outdoor calls `check()` before it opens a connection. It
refuses (exit 3) when:
  - MOWER_PAIR_FILE is unset, missing, under ~/.cache/mower-backend (the
    app's real pairings), or names a robot id that is not a dev identity
    (tools/ab's MW-E2TEST or the sim's MW-SIM*);
  - MOWER_BACKEND_URL is set to anything but a local address;
  - a WebSocket URL is given that is not the sim's (the compose service `sim`,
    or loopback).
Development only; see deploy/sim/outdoor/outdoor.sh.
"""
import json
import os
import sys
from urllib.parse import urlparse

DEV_IDS = ("MW-E2TEST",)
DEV_PREFIXES = ("MW-SIM",)
LOCAL_HOSTS = ("sim", "127.0.0.1", "localhost", "::1")


def refuse(why):
    sys.stderr.write(f"outdoor safety: refusing to run: {why}\n")
    sys.exit(3)


def check(ws_url=None):
    pair = os.environ.get("MOWER_PAIR_FILE")
    if not pair:
        refuse("MOWER_PAIR_FILE is unset (point it at tools/ab/fixtures/pair.json)")
    path = os.path.realpath(os.path.expanduser(pair))
    if "mower-backend" in path or os.path.basename(path).startswith("pair-MW-"):
        refuse(f"MOWER_PAIR_FILE={pair} is a real pairing, not the sim's")
    try:
        robot = json.load(open(path))["robot_id"]
    except (OSError, ValueError, KeyError) as e:
        refuse(f"MOWER_PAIR_FILE={pair}: {e}")
    if robot not in DEV_IDS and not robot.startswith(DEV_PREFIXES):
        refuse(f"MOWER_PAIR_FILE names robot {robot}, not a dev identity")
    backend = os.environ.get("MOWER_BACKEND_URL", "")
    if backend and urlparse(backend).hostname not in LOCAL_HOSTS:
        refuse(f"MOWER_BACKEND_URL={backend} is not local")
    if ws_url is not None and urlparse(ws_url).hostname not in LOCAL_HOSTS:
        refuse(f"{ws_url} is not the sim")
    return path


def headers(client):
    """The pairing gate's HMAC headers for the sim pair (probe_app.py's)."""
    import base64
    import hashlib
    import hmac
    import time
    d = json.load(open(check()))
    s = d["secret"].strip().upper()
    s += "=" * (-len(s) % 8)
    secret = base64.b32decode(s)
    t = int(time.time())
    nonce = os.urandom(16).hex()
    mac = hmac.new(secret, f"{d['robot_id']}\n{client}\n{t}\n{nonce}".encode(), hashlib.sha256).hexdigest()
    return {"X-Mower-Robot": d["robot_id"], "X-Mower-Client": client, "X-Mower-Time": str(t),
            "X-Mower-Nonce": nonce, "X-Mower-Mac": mac}


if __name__ == "__main__":
    print("ok:", check(sys.argv[1] if len(sys.argv) > 1 else None))
