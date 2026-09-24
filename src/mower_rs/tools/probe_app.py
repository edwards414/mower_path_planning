#!/usr/bin/env python3
"""App-contract probe: everything the phone app can subscribe to or call through
mower_ws_bridge (src/mower_bringup/config/ws_bridge.yaml), checked the way the
app uses it. Read-only: no mutating service is called, nothing is published.

usage: probe_app.py ws://HOST:9090 [seconds]
exit 0 = every REQUIRED topic delivered and every service answered; 1 otherwise.
"""
import asyncio, base64, hashlib, hmac, json, os, sys, time
import websockets

PAIR = os.path.expanduser(os.environ.get("MOWER_PAIR_FILE",
                          "~/.cache/mower-backend/pair-MW-0CFP37.json"))
d = json.load(open(PAIR))
robot = d["robot_id"]
s = d["secret"].strip().upper(); s += "=" * (-len(s) % 8)
secret = base64.b32decode(s)
client = "claude-app-probe"
t = int(time.time()); nonce = os.urandom(16).hex()
mac = hmac.new(secret, f"{robot}\n{client}\n{t}\n{nonce}".encode(), hashlib.sha256).hexdigest()
headers = {"X-Mower-Robot": robot, "X-Mower-Client": client, "X-Mower-Time": str(t),
           "X-Mower-Nonce": nonce, "X-Mower-Mac": mac}

# What the app needs at connect. REQUIRED must arrive; OPTIONAL is reported only
# (needs outdoors / manual driving / the opt-in recorder node).
REQUIRED = ["/robot/online", "/robot/info", "/robot/telemetry", "/battery_state",
            "/pid_autotune/status", "/site_list",
            "/adapter/robot_pose", "/adapter/map_layers/map_grid",
            "/adapter/coverage_settings", "/adapter/zone_summaries"]
# --save FILE writes the delivered-topic set as the baseline; --baseline FILE
# fails if any topic the baseline delivered is missing now (so map_datum and
# the inflated layers, empty on main indoors, are compared like-for-like).
SAVE = sys.argv[sys.argv.index("--save") + 1] if "--save" in sys.argv else None
BASE = sys.argv[sys.argv.index("--baseline") + 1] if "--baseline" in sys.argv else None
OPTIONAL = ["/fix", "/gps/fix", "/manual_command_clock", "/mower_recorder/status",
            "/mower_recorder/bags", "/mower_recorder/command_result"]
SERVICES = [("/rosapi/topics", {}), ("/check_nav_status", {}), ("/rosapi/get_time", {})]

url = sys.argv[1]
dur = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else 20.0


async def main():
    t0 = time.monotonic()
    async with websockets.connect(url, additional_headers=headers, max_size=None) as ws:
        t_conn = time.monotonic() - t0
        # rosapi first so /adapter/* can be expanded like the app's fleet provider
        await ws.send(json.dumps({"op": "call_service", "service": "/rosapi/topics", "id": "topics"}))
        adapters, topics_all = [], None
        while topics_all is None and time.monotonic() - t0 < 10:
            m = json.loads(await asyncio.wait_for(ws.recv(), 10))
            if m.get("op") == "service_response" and m.get("id") == "topics":
                topics_all = m.get("values", {}).get("topics", [])
                adapters = sorted(x for x in topics_all if x.startswith("/adapter/"))
        subs = sorted(set(REQUIRED + OPTIONAL + adapters))
        for tp in subs:
            await ws.send(json.dumps({"op": "subscribe", "topic": tp}))
        for i, (srv, args) in enumerate(SERVICES):
            await ws.send(json.dumps({"op": "call_service", "service": srv, "id": f"s{i}", "args": args}))
        counts, first, sizes, resp = {}, {}, {}, {}
        end = time.monotonic() + dur
        while time.monotonic() < end:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=max(0.05, end - time.monotonic()))
            except asyncio.TimeoutError:
                break
            m = json.loads(raw)
            if m.get("op") == "publish":
                tp = m["topic"]; counts[tp] = counts.get(tp, 0) + 1
                first.setdefault(tp, round(time.monotonic() - t0, 2)); sizes[tp] = max(sizes.get(tp, 0), len(raw))
            elif m.get("op") == "service_response":
                resp[m.get("id")] = bool(m.get("result", True))
        missing = [tp for tp in REQUIRED if tp not in counts]
        missing_adapter = []
        if SAVE:
            json.dump({"delivered": sorted(counts), "topics": sorted(topics_all or [])}, open(SAVE, "w"), indent=1)
        if BASE:
            b = json.load(open(BASE))
            missing_adapter = [tp for tp in b["delivered"] if tp not in counts]
            gone = [tp for tp in b["topics"] if tp not in (topics_all or [])]
            if gone:
                print("topics_gone_from_rosapi=" + json.dumps(gone)); missing_adapter += gone
        svc = {SERVICES[int(k[1:])][0]: v for k, v in resp.items() if k.startswith("s")}
        svc_missing = [srv for srv, _ in SERVICES if srv not in svc]
        print(f"connect_s={t_conn:.2f} topics_listed={len(topics_all or [])} adapters={len(adapters)}")
        print("required_first_s=" + json.dumps({tp: first.get(tp) for tp in REQUIRED}))
        print("rates_hz=" + json.dumps({tp: round(counts.get(tp, 0) / dur, 2) for tp in ["/robot/online", "/robot/telemetry", "/adapter/robot_pose", "/battery_state"]}))
        print("adapter_topics=" + json.dumps({tp: counts.get(tp, 0) for tp in adapters}))
        print("optional=" + json.dumps({tp: counts.get(tp, 0) for tp in OPTIONAL}))
        print("services=" + json.dumps(svc) + (" MISSING " + json.dumps(svc_missing) if svc_missing else ""))
        print("max_msg_bytes=" + json.dumps({tp: sizes[tp] for tp in sorted(sizes, key=sizes.get, reverse=True)[:3]}))
        ok = not missing and not missing_adapter and not svc_missing and all(svc.values())
        print(("APP_CONTRACT_OK" if ok else "APP_CONTRACT_FAIL") + (" missing=" + json.dumps(missing + missing_adapter) if (missing or missing_adapter) else ""))
        return 0 if ok else 1

sys.exit(asyncio.run(main()))
