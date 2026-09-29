// Minimal stand-in for rclcpp/time.hpp so diff_drive_controller/odometry.cpp
// compiles without ROS. Only what Odometry touches: construction from a
// nanosecond count (Odometry's ctor does `timestamp_(0.0)`) and seconds().
// rclcpp::Time stores an int64 nanosecond count and seconds() is
// nanoseconds / 1e9 in double, which is what the Rust port does too.
#pragma once
#include <cstdint>

namespace rclcpp {
class Time {
public:
  Time(int64_t nanoseconds = 0) : ns_(nanoseconds) {}
  Time(double nanoseconds) : ns_(static_cast<int64_t>(nanoseconds)) {}
  double seconds() const { return static_cast<double>(ns_) / 1e9; }
  int64_t nanoseconds() const { return ns_; }
private:
  int64_t ns_;
};
}  // namespace rclcpp
