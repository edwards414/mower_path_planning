//! The rear-light overlay of the 0x03 frame, as `MowerSystem::send_led_if_needed` sends it: the base light from
//! `/mower_base/led_command` with the `REAR_RECORDING` bit while a recorder's deadline holds, SHOW (keep what is on
//! the strips) with it when no base light was ever asked for, on every change, and again every
//! `led_resend_period_s`.

use mower_base_core::cycle::{BaseConfig, BaseCycle, Event, LedRequest, RearLightSources};
use mower_base_core::protocol::{self as proto, FrameParser};
use mower_base_core::TimeNs;

const MS: TimeNs = 1_000_000;
const T0: TimeNs = 86_400_000 * MS;

/// Every 0x03 payload a tick at `t` writes.
fn led_frames(base: &mut BaseCycle, t: TimeNs) -> Vec<Vec<u8>> {
    let (tx, _, _) = base.tick(None, t, t);
    let mut out = Vec::new();
    let mut parser = FrameParser::new();
    if let Some(tx) = tx {
        parser.feed(&tx.bytes, |kind, _, payload| {
            if kind == proto::WS2812_COMMAND {
                out.push(payload.to_vec());
            }
        });
    }
    out
}

/// The 0x03 frames of the ticks every 40 ms over `[from, to)`, with their times.
fn run(base: &mut BaseCycle, from: TimeNs, to: TimeNs) -> Vec<(TimeNs, Vec<u8>)> {
    let mut out = Vec::new();
    let mut t = from;
    while t < to {
        for f in led_frames(base, t) {
            out.push((t, f));
        }
        t += 40 * MS;
    }
    out
}

fn rear_events(base: &mut BaseCycle) -> Vec<bool> {
    base.take_events()
        .into_iter()
        .filter_map(|e| match e {
            Event::RearLight { on } => Some(on),
            _ => None,
        })
        .collect()
}

fn new_base() -> BaseCycle {
    BaseCycle::new(BaseConfig::default(), T0).unwrap().0
}

#[test]
fn with_no_light_asked_for_the_overlay_rides_on_show_and_goes_once_it_expires() {
    let mut base = new_base();
    // nothing asked for: no 0x03 at all
    assert!(run(&mut base, T0 + 40 * MS, T0 + 7_000 * MS).is_empty());
    let _ = base.take_events();

    // path_record starts recording: one SHOW with the overlay
    let mut sources = RearLightSources::default();
    let start = T0 + 7_000 * MS;
    base.request_rear_light(sources.update("path_record", true, start, 6_000 * MS));
    let frames = run(&mut base, start, start + 4_000 * MS);
    assert_eq!(frames.len(), 1, "{frames:?}");
    assert_eq!(frames[0].1, [proto::LED_SHOW, 0, 0, 0, 0, 0, proto::LED_OVERLAY_REAR_RECORDING, 0]);
    assert_eq!(rear_events(&mut base), [true]);

    // refreshed every 2 s, as the recorders do: re-sent every 5 s, still on
    let mut t = start + 2_000 * MS;
    let mut frames = Vec::new();
    while t < start + 11_000 * MS {
        base.request_rear_light(sources.update("path_record", true, t, 6_000 * MS));
        frames.extend(run(&mut base, t.max(start + 4_000 * MS), t + 2_000 * MS));
        t += 2_000 * MS;
    }
    assert!(frames.iter().all(|(_, p)| p[6] == proto::LED_OVERLAY_REAR_RECORDING), "{frames:?}");
    assert_eq!(frames.len(), 2, "a resend every 5 s: {frames:?}");
    assert!(rear_events(&mut base).is_empty());

    // the recorder dies: 6 s after its last refresh one SHOW without the overlay (after the 5 s resend, still on),
    // then nothing
    let last = t - 2_000 * MS;
    let frames = run(&mut base, t, last + 20_000 * MS);
    let off: Vec<&(TimeNs, Vec<u8>)> = frames.iter().filter(|(_, p)| p[6] == 0).collect();
    assert_eq!(off.len(), 1, "{frames:?}");
    assert_eq!(off[0].1, [proto::LED_SHOW, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!(frames.last().unwrap(), off[0], "nothing after the overlay went off");
    let off_at = off[0].0 - last;
    assert!(off_at >= 6_000 * MS && off_at < 6_040 * MS, "{} ms", off_at / MS);
    assert_eq!(rear_events(&mut base), [false]);
}

#[test]
fn the_overlay_rides_on_the_light_asked_for_and_one_source_s_off_clears_only_itself() {
    let mut base = new_base();
    let white = LedRequest { mode: proto::LED_ALL_ON, r: 110, g: 110, b: 110, period_ms: 0, serial: 1 };
    base.request_led(white);
    let t = T0 + 40 * MS;
    let frames = run(&mut base, t, t + 400 * MS);
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].1, [1, 110, 110, 110, 0, 0, 0, 0]);

    // two recorders on
    let mut sources = RearLightSources::default();
    let t = t + 400 * MS;
    base.request_rear_light(sources.update("bag", true, t, 6_000 * MS));
    base.request_rear_light(sources.update("path_record", true, t, 6_000 * MS));
    let frames = run(&mut base, t, t + 400 * MS);
    assert_eq!(frames.iter().map(|(_, p)| p.clone()).collect::<Vec<_>>(), [vec![1, 110, 110, 110, 0, 0, 1, 0]]);

    // a new base light keeps the overlay
    let t = t + 400 * MS;
    base.request_led(LedRequest { mode: proto::LED_ORBIT, r: 255, g: 180, b: 0, period_ms: 1600, serial: 2 });
    let frames = run(&mut base, t, t + 400 * MS);
    assert_eq!(frames[0].1, [6, 255, 180, 0, 0x40, 0x06, 1, 0]);

    // the app's recording stops: the bag still records, the overlay stays
    let t = t + 400 * MS;
    base.request_rear_light(sources.update("path_record", false, t, 6_000 * MS));
    assert!(run(&mut base, t, t + 400 * MS).is_empty());
    // and so does the bag: off at once
    let t = t + 400 * MS;
    base.request_rear_light(sources.update("bag", false, t, 6_000 * MS));
    let frames = run(&mut base, t, t + 400 * MS);
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].1, [6, 255, 180, 0, 0x40, 0x06, 0, 0]);
    assert_eq!(rear_events(&mut base), [true, false]);
}
