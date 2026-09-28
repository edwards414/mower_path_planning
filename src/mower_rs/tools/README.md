# mower_rs/tools

Verification harnesses. The `*_compare.py` ones run a Python/C++ node and its
Rust replacement against the same fakes and diff every message; the three at
the bottom drive an A/B measurement on the real robot.

| tool | what it does |
|---|---|
| `shadow_compare.py` | separate mower_rs binaries vs one `mower_rsd` process, every topic compared (Phase A5) |
| `fake_base.py`, `base_harness.py`, `base_compare.py`, `base_ab.sh` | a fake STM32 on a pty, then the `ros2_control` chain vs `mower_base` against it (Phase B); `base_ab.sh` runs one side in the runtime image under the robot's CycloneDDS, `base_compare.py` exits 1 on any endpoint QoS difference, `--scenario pull` is the cable pull (recipe in `../README.md`, mower_base "Verification") |
| `localize_compare.py` | `dual_ekf_navsat.launch.py` vs `mower_localize` on one synthetic sensor stream (Phase C) |
| `probe_app.py` | **the app contract**: connects to `mower_ws_bridge` with the app's HMAC headers, subscribes to everything the app subscribes to and calls the read-only services, then passes or fails against a saved baseline |
| `switch.sh`, `measure.sh` | flip `IMAGE_TAG` / `NAV_COMPOSITION` / `RUST_DAEMON` / any `KEY=VAL` on the robot and restore; per-process CPU, loopback packet rate and DDS thread split over 10 s |

## Checking the app still works

`probe_app.py` is what proves a `rust_*` switch did not break the phone app,
without a phone. It reads the pairing secret from
`~/.cache/mower-backend/pair-<robot>.json` (override with `MOWER_PAIR_FILE`),
so it authenticates exactly as the app does.

```bash
# once, against a known-good image, to record what the app receives
python3 probe_app.py ws://192.168.0.114:9090 20 --save app_contract_baseline.json
# after every switch: fails if any topic the baseline delivered is missing
python3 probe_app.py ws://192.168.0.114:9090 20 --baseline app_contract_baseline.json
```

It prints the per-topic first-message latency and rates, the `/adapter/*` set
expanded from `/rosapi/topics`, the service answers, and ends in
`APP_CONTRACT_OK` or `APP_CONTRACT_FAIL` with the missing topics named.
`app_contract_baseline.json` here is the 2026-09-23 recording from the robot
on `main` (18 topics delivered, 22 listed); indoors `/adapter/map_datum` and
the three inflated map layers are legitimately empty, which is why the check
compares against a recording rather than a fixed list.

## Measuring on the robot

```bash
scp switch.sh measure.sh cat@<robot>:/tmp/
ssh cat@<robot> 'sudo systemctl stop mower-update.timer'            # no auto-update mid-test
ssh cat@<robot> 'bash /tmp/switch.sh <tag> <nav_composition> <rust_daemon> <compose|keep> [KEY=VAL ...]'
ssh cat@<robot> 'sleep 90; bash /tmp/measure.sh'
ssh cat@<robot> 'bash /tmp/switch.sh restore && sudo systemctl start mower-update.timer'
```

`switch.sh` backs `.env` and `docker-compose.yaml` up once as
`*.bak-rosfree-test` and `restore` puts both back. Note `measure.sh` reports
the box's idle percentage too, but on the RK3568 the governor moves between
1.4 and 2.0 GHz, so compare per-process CPU and the loopback packet rate, not
the idle figure.
