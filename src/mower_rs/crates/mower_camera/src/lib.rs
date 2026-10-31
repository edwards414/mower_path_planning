//! The data-collection front camera without gscam (E5-6, E5 design 4.7 and
//! D6): what gscam 2.0.2 did with `image_encoding: jpeg`, so that the `bag`
//! module's `camera_front` instance publishes the same messages.
//!
//! - [`info`]: the calibration file as camera_info_manager loads it
//!   (camera_calibration_parsers `readCalibrationYml`).
//! - [`stamp`]: gscam's `use_gst_timestamps` stamp: the buffer's PTS plus the
//!   pipeline's base time plus an offset between ROS time and GStreamer's
//!   system clock, taken once per stream.
//! - [`stream`] (feature `gst`): gscam's `init_stream` / `publish_stream` /
//!   `run`: the pipeline text parsed, an appsink of `image/jpeg` linked to
//!   its unlinked source pad (sync off), PAUSED then PLAYING, each sample's
//!   first memory as the JPEG, the stream reopened at its end
//!   (`reopen_on_eof`); unlike gscam, an error on the bus restarts the
//!   stream after 3 s (gscam hung in `pull_sample`, and only launch's
//!   respawn after 3 s helped when it died).

pub mod info;
pub mod stamp;
#[cfg(feature = "gst")]
pub mod stream;

/// Whether this build captures (feature `gst`).
pub const CAPTURES: bool = cfg!(feature = "gst");
