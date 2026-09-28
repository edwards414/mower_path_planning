//! Record/replay test for [`BaseCycle`].
//!
//! `tests/vectors/replay_synthetic.mowerlog` is a serial capture in the
//! [`record`] format; `replay_synthetic.json` holds the tick schedule
//! (cmd_vel, side-channel requests) and the expected odometry / joint states.
//! Both come from a harness that links the original C++ protocol and the
//! original `diff_drive_controller` odometry + `control_toolbox` rate limiter
//! (see the crate README), with the cycle glue transcribed from
//! `mower_system.cpp`.
//!
//! The test feeds only the **rx** records into `BaseCycle::on_rx` and checks
//! that every byte it writes matches the recorded **tx** records and that the
//! odometry matches to 1e-12. That is the verification method Phase B of
//! `docs/ROS_FREE_PLAN.md` calls for, because the real serial port cannot be
//! opened by two processes at once: record ros2_control on the robot, replay
//! it into the Rust driver offline.

use mower_base_core::cycle::{BaseConfig, BaseCycle, LedRequest};
use mower_base_core::diff_drive::{Command, Twist};
use mower_base_core::protocol::PidConfig;
use mower_base_core::record::{self, Direction};
use mower_base_core::TWO_PI;
use serde_json::Value;

const TOL: f64 = 1e-12;

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn num(v: &Value) -> f64 {
    match v {
        Value::Number(n) => n.as_f64().unwrap(),
        Value::String(s) if s == "nan" => f64::NAN,
        other => panic!("not a number: {other}"),
    }
}

fn close(got: f64, want: f64, what: &str) {
    let scale = want.abs().max(1.0);
    assert!(
        (got - want).abs() <= TOL * scale,
        "{what}: got {got:.17e}, want {want:.17e}"
    );
}

#[test]
fn replay_synthetic_recording() {
    let log = record::decode(include_bytes!("vectors/replay_synthetic.mowerlog"))
        .expect("the recording parses");
    let expect: Value = serde_json::from_str(include_str!("vectors/replay_synthetic.json"))
        .expect("expectations are valid JSON");

    let rx: Vec<&record::Record> = log.iter().filter(|r| r.dir == Direction::Rx).collect();
    let tx: Vec<&record::Record> = log.iter().filter(|r| r.dir == Direction::Tx).collect();
    assert!(rx.len() > 200, "the recording should hold a real rx stream");

    let ticks = expect["ticks"].as_array().unwrap();
    assert_eq!(tx.len(), ticks.len() + 1, "one tx per tick plus the activation burst");

    let t0 = expect["t0_ns"].as_i64().unwrap();
    let tick_period = expect["tick_period_ns"].as_i64().unwrap();

    // on_configure + on_activate at t0 (`previous_publish_timestamp_` is seeded
    // there): stop the wheels and the blade, then ask for 0x87.
    let (mut base, activation) =
        BaseCycle::new(BaseConfig::default(), t0).expect("mower config is valid");
    assert_eq!(
        hex(&activation.bytes),
        expect["activation_tx"].as_str().unwrap(),
        "activation burst"
    );
    assert_eq!(hex(&activation.bytes), hex(&tx[0].bytes));

    let mut next_rx = 0usize;
    for (k, want) in ticks.iter().enumerate() {
        let t = want["t_ns"].as_i64().unwrap();
        assert_eq!(t, t0 + k as i64 * tick_period);

        // read(): hand over every chunk the port would have returned by now
        while next_rx < rx.len() && rx[next_rx].t_ns <= t {
            base.on_rx(&rx[next_rx].bytes, rx[next_rx].t_ns);
            next_rx += 1;
        }

        // side-channel requests that arrived since the last cycle
        for req in want["requests"].as_array().unwrap() {
            match req["kind"].as_str().unwrap() {
                "blade" => base.request_blade(
                    req["permille"].as_i64().unwrap() as i32,
                    req["ttl_ms"].as_i64().unwrap(),
                    t,
                ),
                "led" => base.request_led(LedRequest {
                    mode: req["mode"].as_u64().unwrap() as u8,
                    r: req["r"].as_u64().unwrap() as u8,
                    g: req["g"].as_u64().unwrap() as u8,
                    b: req["b"].as_u64().unwrap() as u8,
                    period_ms: req["period_ms"].as_u64().unwrap() as u16,
                    serial: req["serial"].as_u64().unwrap() as u8,
                }),
                "servo" => base.request_servo(
                    req["pulse_us"].as_i64().unwrap(),
                    req["hold_ms"].as_i64().unwrap(),
                ),
                "override" => base.request_wheel_override(
                    req["left"].as_i64().unwrap() as i32,
                    req["right"].as_i64().unwrap() as i32,
                    req["ttl_ms"].as_i64().unwrap(),
                    t,
                ),
                "pid" => {
                    let l = req["left"].as_array().unwrap();
                    let r = req["right"].as_array().unwrap();
                    base.request_pid(PidConfig {
                        left_kp: num(&l[0]) as f32,
                        left_ki: num(&l[1]) as f32,
                        left_kd: num(&l[2]) as f32,
                        right_kp: num(&r[0]) as f32,
                        right_ki: num(&r[1]) as f32,
                        right_kd: num(&r[2]) as f32,
                        persist_to_flash: req["persist"].as_bool().unwrap(),
                        closed_loop_enabled: req["closed_loop"].as_bool().unwrap(),
                    })
                }
                other => panic!("unknown request kind {other}"),
            }
        }

        let cmd = want["cmd"].as_array().map(|a| Command {
            twist: Twist::new(num(&a[0]), num(&a[1])),
            stamp_ns: t,
        });
        let (got_tx, got_odom, got_joints) = base.tick(cmd, t, t);

        let got_tx = got_tx.expect("every cycle writes at least the wheel command");
        assert_eq!(
            hex(&got_tx.bytes),
            hex(&tx[k + 1].bytes),
            "tx bytes at tick {k} (t = {t})"
        );

        let joints = got_joints.expect("joint states every cycle");
        let wj = &want["joints"];
        close(joints.positions[0], num(&wj["pos"][0]), &format!("tick {k} left pos"));
        close(joints.positions[1], num(&wj["pos"][1]), &format!("tick {k} right pos"));
        close(joints.velocities[0], num(&wj["vel"][0]), &format!("tick {k} left vel"));
        close(joints.velocities[1], num(&wj["vel"][1]), &format!("tick {k} right vel"));
        assert_eq!(joints.stamp_ns, t);

        close(
            base.feedback_age_s(),
            num(&want["feedback_age_s"]),
            &format!("tick {k} feedback age"),
        );
        assert_eq!(
            base.crc_errors() as u64,
            want["crc_errors"].as_u64().unwrap(),
            "tick {k} crc errors"
        );

        match (&got_odom, want["odom"].as_object()) {
            (None, None) => {}
            (Some(o), Some(_)) => {
                let w = &want["odom"];
                close(o.x, num(&w["x"]), &format!("tick {k} odom x"));
                close(o.y, num(&w["y"]), &format!("tick {k} odom y"));
                close(o.yaw, num(&w["yaw"]), &format!("tick {k} odom yaw"));
                close(o.qz, num(&w["qz"]), &format!("tick {k} odom qz"));
                close(o.qw, num(&w["qw"]), &format!("tick {k} odom qw"));
                close(o.linear_x, num(&w["linear"]), &format!("tick {k} odom linear"));
                close(o.angular_z, num(&w["angular"]), &format!("tick {k} odom angular"));
                assert_eq!(o.stamp_ns, t);
            }
            (a, b) => panic!("tick {k} publish gate disagrees: {:?} vs {:?}", a.is_some(), b.is_some()),
        }
    }

    assert_eq!(next_rx, rx.len(), "the whole rx stream was consumed");
    assert_eq!(
        base.crc_errors() as u64,
        expect["crc_errors_total"].as_u64().unwrap(),
        "the injected line noise and corrupt frame were counted"
    );
    assert!(base.crc_errors() > 0, "the recording must exercise the resync path");
    // The 0x86 SHUTDOWN_REQUESTED near the end is acked exactly once.
    assert_eq!(base.take_shutdown_request(), Some(1));
    assert_eq!(base.take_shutdown_request(), None);
    let fw = base.firmware_info().expect("0x87 was seen");
    assert_eq!((fw.major, fw.minor, fw.patch), (0, 6, 0));
    assert!(!base.firmware_protocol_mismatch());
}

fn cmd(linear_x: f64, angular_z: f64, t: i64) -> Option<Command> {
    Some(Command { twist: Twist::new(linear_x, angular_z), stamp_ns: t })
}

/// Fail-closed: a serial error latches, and every following cycle emits the
/// stop burst and no odometry until the port is known good again.
#[test]
fn fault_latches_and_stops() {
    let (mut base, _) = BaseCycle::new(BaseConfig::default(), 0).unwrap();
    let (tx, _, _) = base.tick(cmd(0.4, 0.0, 40_000_000), 40_000_000, 40_000_000);
    assert!(tx.unwrap().frames.iter().any(|(t, _)| *t == 0x01));

    base.fault();
    assert!(base.faulted());
    let (tx, odom, joints) = base.tick(cmd(0.4, 0.0, 80_000_000), 80_000_000, 80_000_000);
    let tx = tx.unwrap();
    assert!(odom.is_none(), "a faulted cycle publishes no odometry");
    assert!(joints.is_some());
    let types: Vec<u8> = tx.frames.iter().map(|(t, _)| *t).collect();
    assert_eq!(types, vec![0x01, 0x02], "stop burst: wheels then blade");
    // both payloads are zero permille
    assert_eq!(&tx.bytes[6..10], &[0, 0, 0, 0]);

    base.clear_fault(120_000_000);
    assert!(!base.faulted());
}

/// The cmd_vel timeout does not drop the command, it ramps it down at
/// `max_acceleration` — that ramp is the safety-zero deceleration.
#[test]
fn cmd_vel_timeout_decelerates_rather_than_dropping() {
    let (mut base, _) = BaseCycle::new(BaseConfig::default(), 0).unwrap();
    let dt = 40_000_000i64;
    let mut t = 0i64;
    // drive up to the limit
    for _ in 0..60 {
        t += dt;
        base.tick(cmd(0.55, 0.0, t), t, t);
    }
    let moving = base.rad_s_to_permille(base.left().cmd_velocity);
    assert!(moving > 800, "should be near full speed, got {moving}");

    // go silent: nothing for the first 0.5 s, then the reference is zeroed
    // and the limiter brings it down over 0.55 / 0.8 = 0.6875 s
    let mut permilles = Vec::new();
    for _ in 0..40 {
        t += dt;
        base.tick(None, t, t);
        permilles.push(base.rad_s_to_permille(base.left().cmd_velocity));
    }
    assert!(base.diff_drive().command_timed_out());
    assert!(
        permilles.windows(2).all(|w| w[1] <= w[0]),
        "the ramp must be monotonic: {permilles:?}"
    );
    assert_eq!(*permilles.last().unwrap(), 0);
    let first_zero = permilles.iter().position(|p| *p == 0).unwrap();
    assert!(
        (13..=32).contains(&first_zero),
        "0.5 s timeout + ~0.69 s ramp at 25 Hz, got {first_zero} cycles"
    );
}

/// No 0x85 for `feedback_timeout_s` zeroes the reported wheel velocities;
/// positions are held, because the encoder count is absolute.
#[test]
fn feedback_timeout_zeroes_velocities_but_holds_positions() {
    use mower_base_core::protocol::{build_frame, WHEEL_FEEDBACK_STATUS};
    let (mut base, _) = BaseCycle::new(BaseConfig::default(), 0).unwrap();

    let frame = |counts: i32, rpm: i16| {
        let mut p = [0u8; 24];
        p[2..4].copy_from_slice(&rpm.to_le_bytes());
        p[6..8].copy_from_slice(&rpm.to_le_bytes());
        p[12..16].copy_from_slice(&counts.to_le_bytes());
        p[16..20].copy_from_slice(&counts.to_le_bytes());
        build_frame(WHEEL_FEEDBACK_STATUS, 0, &p)
    };
    base.on_rx(&frame(0, 0), 0);
    base.on_rx(&frame(8896, 5800), 100_000_000); // one full turn, 58.00 rpm
    base.tick(None, 100_000_000, 100_000_000);
    let pos = base.left().pos;
    assert!((pos - TWO_PI).abs() < 1e-12, "one revolution, got {pos}");
    assert!((base.left().vel - 58.0 * TWO_PI / 60.0).abs() < 1e-12);

    // 0.6 s later, still nothing
    base.tick(None, 700_000_000, 700_000_000);
    assert_eq!(base.left().vel, 0.0);
    assert_eq!(base.right().vel, 0.0);
    assert_eq!(base.left().pos, pos, "positions are held, not reset");
    assert!(base.feedback_age_s() > 0.5);
}

/// The 32-bit encoder counter wraps; the difference must stay signed.
#[test]
fn encoder_counter_wrap_is_handled() {
    use mower_base_core::protocol::{build_frame, WHEEL_FEEDBACK_STATUS};
    let (mut base, _) = BaseCycle::new(BaseConfig::default(), 0).unwrap();
    let frame = |counts: i32| {
        let mut p = [0u8; 24];
        p[12..16].copy_from_slice(&counts.to_le_bytes());
        p[16..20].copy_from_slice(&counts.to_le_bytes());
        build_frame(WHEEL_FEEDBACK_STATUS, 0, &p)
    };
    base.on_rx(&frame(i32::MAX - 5), 0);
    base.on_rx(&frame(i32::MIN + 4), 50_000_000); // +10 counts across the wrap
    let expected = 10.0 * TWO_PI / 8896.0;
    assert!(
        (base.left().pos - expected).abs() < 1e-15,
        "wrapped by {} instead of 10 counts",
        base.left().pos / (TWO_PI / 8896.0)
    );
}
