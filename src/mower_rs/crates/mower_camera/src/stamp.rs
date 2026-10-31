//! gscam's stamp with `use_gst_timestamps: true` (gscam.cpp 2.0.2):
//!
//! ```text
//! init_stream:    time_offset_ = now().nanoseconds() - GST_TIME_AS_NSECONDS(gst_clock_get_time(system clock))
//! publish_stream: stamp = rclcpp::Time(GST_TIME_AS_NSECONDS(buf->pts + base_time) + time_offset_)
//! ```
//!
//! The offset is taken once per stream (each reopen takes it again), so the
//! capture time maps to ROS time as it stood when the stream started.

/// `time_offset_`: ROS time minus GStreamer's system clock, in ns.
pub fn offset(ros_now_ns: i64, gst_system_clock_ns: u64) -> i64 {
    ros_now_ns.wrapping_sub(gst_system_clock_ns as i64)
}

/// A buffer's stamp in ns: `pts + base_time + offset`, in GStreamer's
/// unsigned arithmetic for the first sum (`GstClockTime`), then rclcpp's
/// signed nanoseconds. A buffer without a PTS has `GST_CLOCK_TIME_NONE`
/// (`u64::MAX`), as in gscam.
pub fn stamp_ns(pts: Option<u64>, base_time: u64, offset: i64) -> i64 {
    let pts = pts.unwrap_or(u64::MAX);
    (pts.wrapping_add(base_time) as i64).wrapping_add(offset)
}

/// `builtin_interfaces/Time` of a stamp, as `rclcpp::Time` gives it:
/// whole seconds and the nanoseconds left (for the stamps a camera makes,
/// which are after 1970).
pub fn to_parts(ns: i64) -> (i32, u32) {
    let sec = ns.div_euclid(1_000_000_000);
    (sec as i32, ns.rem_euclid(1_000_000_000) as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_stamp_is_pts_plus_base_time_plus_the_stream_s_offset() {
        // the stream starts: ROS 1793000000.5 s, GStreamer's clock 120 s
        let off = offset(1_793_000_000_500_000_000, 120_000_000_000);
        assert_eq!(off, 1_792_999_880_500_000_000);
        // a frame 0.4 s into a pipeline whose base time is 120.1 s
        let s = stamp_ns(Some(400_000_000), 120_100_000_000, off);
        assert_eq!(s, 1_793_000_001_000_000_000);
        assert_eq!(to_parts(s), (1_793_000_001, 0));
        assert_eq!(to_parts(1_793_000_001_123_456_789), (1_793_000_001, 123_456_789));
    }
}
