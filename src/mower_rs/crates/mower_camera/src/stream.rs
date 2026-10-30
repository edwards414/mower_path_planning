//! gscam 2.0.2's stream (`init_stream`, `publish_stream`, `cleanup_stream`,
//! `run`) on gstreamer-rs, for `image_encoding: jpeg`, `sync_sink: false`,
//! `preroll: false`, `use_gst_timestamps: true`, `reopen_on_eof: true`:
//!
//! - `gst_parse_launch` of the pipeline text;
//! - an appsink whose caps are `image/jpeg`, `sync` off, linked to the
//!   element of the bin's unlinked source pad (or, for a single element,
//!   both put in a new pipeline);
//! - the stamp offset taken ([`crate::stamp::offset`]), the pipeline
//!   PAUSED (waiting for the state change), then PLAYING;
//! - each sample: its buffer's first memory is the JPEG (gscam maps
//!   `gst_buffer_get_memory(buf, 0)`), stamped `pts + base_time + offset`;
//! - no more samples (end of stream): the pipeline set to NULL, and, with
//!   `reopen_on_eof`, a new stream (a new offset).
//!
//! What gscam did not do: an error on the bus ends the stream too, and the
//! next starts [`Options::restart_after`] later (launch respawned a dead
//! gscam after 3 s; a hung one stayed hung). The pull waits at most 100 ms,
//! so that a stop is seen.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;

/// How the stream runs.
#[derive(Debug, Clone)]
pub struct Options {
    /// gscam's `gscam_config`
    pub pipeline: String,
    pub reopen_on_eof: bool,
    /// After an error (launch's `respawn_delay` was 3 s).
    pub restart_after: Duration,
}

/// What the stream tells its caller.
pub enum Event<'a> {
    /// A stream started: its offset (ns).
    Started { offset: i64 },
    /// A sample: its first memory, and its stamp (ns of ROS time).
    Frame { jpeg: &'a [u8], stamp_ns: i64 },
    /// It ended: why, and whether it will be opened again.
    Ended { why: String, again: bool },
}

fn ros_now_ns() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_nanos() as i64).unwrap_or(0)
}

/// `init_stream`: the pipeline with its appsink, PAUSED.
fn open(text: &str) -> Result<(gst::Pipeline, gst_app::AppSink, i64), String> {
    let launched = gst::parse::launch(text).map_err(|e| e.to_string())?;
    let sink = gst_app::AppSink::builder().caps(&gst::Caps::builder("image/jpeg").build()).build();
    sink.set_property("sync", false);
    let pipeline = match launched.clone().downcast::<gst::Pipeline>() {
        Ok(pipeline) => {
            let outpad = pipeline.find_unlinked_pad(gst::PadDirection::Src).ok_or("the pipeline has no unlinked source pad")?;
            let outelement = outpad.parent_element().ok_or("the unlinked source pad has no element")?;
            pipeline.add(&sink).map_err(|e| format!("gst_bin_add() failed: {e}"))?;
            outelement
                .link(&sink)
                .map_err(|_| format!("GStreamer: cannot link outelement(\"{}\") -> sink", outelement.name()))?;
            pipeline
        }
        Err(single) => {
            let pipeline = gst::Pipeline::new();
            pipeline.add_many([&single, sink.upcast_ref()]).map_err(|e| e.to_string())?;
            single.link(&sink).map_err(|_| "GStreamer: cannot link launchpipe -> sink".to_string())?;
            pipeline
        }
    };
    // Calibration between ROS time and GStreamer's timestamps
    let clock = gst::SystemClock::obtain();
    let ct = clock.time().nseconds();
    let offset = crate::stamp::offset(ros_now_ns(), ct);
    pipeline.set_state(gst::State::Paused).map_err(|e| format!("Failed to PAUSE stream: {e}"))?;
    let (changed, _, _) = pipeline.state(gst::ClockTime::NONE);
    if changed.is_err() {
        let _ = pipeline.set_state(gst::State::Null);
        return Err("Failed to PAUSE stream, check your gstreamer configuration.".into());
    }
    Ok((pipeline, sink, offset))
}

/// The first error on the pipeline's bus, if one came.
fn bus_error(pipeline: &gst::Pipeline) -> Option<String> {
    let bus = pipeline.bus()?;
    while let Some(msg) = bus.pop_filtered(&[gst::MessageType::Error]) {
        if let gst::MessageView::Error(err) = msg.view() {
            let from = err.src().map(|s| s.path_string().to_string()).unwrap_or_default();
            return Some(format!("{from}: {} ({})", err.error(), err.debug().map(|d| d.to_string()).unwrap_or_default()));
        }
    }
    None
}

/// `publish_stream` until the stream ends, `stop` is set or an error comes.
/// `Ok(true)`: the stream ended (end of stream); `Ok(false)`: stopped.
fn publish(
    pipeline: &gst::Pipeline, sink: &gst_app::AppSink, offset: i64, stop: &AtomicBool, on: &mut impl FnMut(Event),
) -> Result<bool, String> {
    pipeline.set_state(gst::State::Playing).map_err(|_| "Could not start stream!".to_string())?;
    loop {
        if stop.load(Ordering::Acquire) {
            return Ok(false);
        }
        if let Some(e) = bus_error(pipeline) {
            return Err(e);
        }
        let Some(sample) = sink.try_pull_sample(gst::ClockTime::from_mseconds(100)) else {
            if sink.is_eos() {
                return Ok(true);
            }
            continue;
        };
        let Some(buffer) = sample.buffer() else { return Ok(true) };
        let base_time = pipeline.base_time().map(|t| t.nseconds()).unwrap_or(u64::MAX);
        let stamp_ns = crate::stamp::stamp_ns(buffer.pts().map(|t| t.nseconds()), base_time, offset);
        if buffer.n_memory() == 0 {
            continue;
        }
        let memory = buffer.peek_memory(0);
        let Ok(map) = memory.map_readable() else { continue };
        on(Event::Frame { jpeg: map.as_slice(), stamp_ns });
    }
}

/// gscam's `run` until `stop` is set: open, publish, clean up, and again at
/// the end of a stream (`reopen_on_eof`) or, after `restart_after`, an error.
pub fn run(opts: &Options, stop: &AtomicBool, mut on: impl FnMut(Event)) {
    if let Err(e) = gst::init() {
        on(Event::Ended { why: format!("gst_init: {e}"), again: false });
        return;
    }
    while !stop.load(Ordering::Acquire) {
        let (outcome, again, wait) = match open(&opts.pipeline) {
            Err(e) => (format!("Failed to initialize the stream: {e}"), true, opts.restart_after),
            Ok((pipeline, sink, offset)) => {
                on(Event::Started { offset });
                let r = publish(&pipeline, &sink, offset, stop, &mut on);
                // cleanup_stream
                let _ = pipeline.set_state(gst::State::Null);
                match r {
                    Ok(false) => return,
                    Ok(true) => ("Stream ended.".to_string(), opts.reopen_on_eof, Duration::ZERO),
                    Err(e) => (e, true, opts.restart_after),
                }
            }
        };
        on(Event::Ended { why: outcome, again });
        if !again {
            return;
        }
        // the restart's wait, a stop seen within 100 ms
        let mut left = wait;
        while !left.is_zero() && !stop.load(Ordering::Acquire) {
            let step = left.min(Duration::from_millis(100));
            std::thread::sleep(step);
            left -= step;
        }
    }
}
