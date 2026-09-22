// Oracle harness for the diff_drive_controller half of the base driver.
//
// Links the original ros2_controllers/jazzy diff_drive_controller/src/odometry.cpp
// and the original control_toolbox RateLimiter (via diff_drive_controller's
// SpeedLimiter header). rclcpp::Time and rcpputils::RollingMeanAccumulator are
// stubbed (see stub/); the stubs are byte-for-byte the upstream semantics the
// port relies on (int64 ns time, sum/count rolling mean).
//
// The per-cycle glue (update_reference_from_subscribers +
// update_and_write_commands) is transcribed from
// diff_drive_controller/src/diff_drive_controller.cpp.
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <queue>
#include <array>
#include <string>
#include <vector>

#include "diff_drive_controller/odometry.hpp"
#include "diff_drive_controller/speed_limiter.hpp"

using diff_drive_controller::Odometry;
using diff_drive_controller::SpeedLimiter;

static const double NaN = std::numeric_limits<double>::quiet_NaN();

static std::string num(double v)
{
  if (std::isnan(v)) return "\"nan\"";
  if (std::isinf(v)) return v > 0 ? "\"inf\"" : "\"-inf\"";
  char b[64];
  std::snprintf(b, sizeof(b), "%.17g", v);
  return b;
}

int main()
{
  std::string out = "{\n";

  // ---- RollingMeanAccumulator ------------------------------------------
  {
    out += "  \"rolling_mean\": [\n";
    const size_t windows[] = {1, 3, 10};
    const double vals[] = {1.0, 2.0, -3.5, 4.25, 0.0, 7.125, -1.0, 2.5, 3.0, 4.0, 5.0, 6.0};
    bool first = true;
    for (size_t w : windows) {
      rcpputils::RollingMeanAccumulator<double> acc(w);
      std::string means;
      for (size_t i = 0; i < sizeof(vals) / sizeof(*vals); ++i) {
        acc.accumulate(vals[i]);
        if (i) means += ", ";
        means += num(acc.getRollingMean());
      }
      char b[1024];
      std::snprintf(b, sizeof(b), "%s    {\"window\": %zu, \"values\": [", first ? "" : ",\n", w);
      out += b;
      for (size_t i = 0; i < sizeof(vals) / sizeof(*vals); ++i) {
        out += (i ? ", " : "") + num(vals[i]);
      }
      out += "], \"means\": [" + means + "]}";
      first = false;
    }
    out += "\n  ],\n";
  }

  // ---- SpeedLimiter ----------------------------------------------------
  {
    struct LimCase {
      const char * name;
      double min_v, max_v, max_acc_rev, max_acc, max_dec, max_dec_rev, min_jerk, max_jerk;
    };
    const LimCase cfgs[] = {
      // what mower_controllers.yaml gives after the deprecated has_* handling:
      // velocity +-0.55 / +-2.0, acceleration symmetric +-0.8 / +-2.0, no jerk
      {"mower_linear", -0.55, 0.55, NaN, 0.8, NaN, NaN, NaN, NaN},
      {"mower_angular", -2.0, 2.0, NaN, 2.0, NaN, NaN, NaN, NaN},
      {"no_limits", NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN},
      {"velocity_only", -1.0, 2.0, NaN, NaN, NaN, NaN, NaN, NaN},
      {"asymmetric_accel", -1.0, 1.0, -0.4, 0.9, -0.7, 0.5, NaN, NaN},
      {"with_jerk", -1.0, 1.0, NaN, 1.0, NaN, NaN, -5.0, 5.0},
    };
    struct Sample { double v, v0, v1, dt; };
    const Sample samples[] = {
      {0.0, 0.0, 0.0, 0.04}, {1.0, 0.0, 0.0, 0.04}, {-1.0, 0.0, 0.0, 0.04},
      {0.5, 0.4, 0.3, 0.04}, {0.0, 0.5, 0.45, 0.04}, {-0.5, 0.5, 0.5, 0.04},
      {3.0, 0.5, 0.2, 0.04}, {-3.0, -0.5, -0.2, 0.04}, {0.1, -0.1, -0.2, 0.04},
      {0.2, 0.2, 0.2, 0.04}, {2.0, 1.0, 0.0, 0.2}, {-2.0, -1.0, 0.0, 0.2},
      {0.55, 0.0, 0.0, 0.04}, {0.56, 0.55, 0.54, 0.04}, {1.0, 0.9, 1.1, 0.04},
    };
    out += "  \"speed_limiter\": [\n";
    bool first = true;
    for (const auto & c : cfgs) {
      SpeedLimiter lim(c.min_v, c.max_v, c.max_acc_rev, c.max_acc, c.max_dec, c.max_dec_rev,
        c.min_jerk, c.max_jerk);
      for (const auto & s : samples) {
        double v_all = s.v, v_val = s.v, v_d1 = s.v, v_d2 = s.v;
        double f_all = lim.limit(v_all, s.v0, s.v1, s.dt);
        double f_val = lim.limit_velocity(v_val);
        double f_d1 = lim.limit_acceleration(v_d1, s.v0, s.dt);
        double f_d2 = lim.limit_jerk(v_d2, s.v0, s.v1, s.dt);
        char b[1536];
        std::snprintf(b, sizeof(b),
          "%s    {\"cfg\": \"%s\", \"params\": [%s, %s, %s, %s, %s, %s, %s, %s], "
          "\"in\": [%s, %s, %s, %s], "
          "\"limit\": {\"v\": %s, \"factor\": %s}, \"limit_value\": {\"v\": %s, \"factor\": %s}, "
          "\"limit_first_derivative\": {\"v\": %s, \"factor\": %s}, "
          "\"limit_second_derivative\": {\"v\": %s, \"factor\": %s}}",
          first ? "" : ",\n", c.name, num(c.min_v).c_str(), num(c.max_v).c_str(),
          num(c.max_acc_rev).c_str(), num(c.max_acc).c_str(), num(c.max_dec).c_str(),
          num(c.max_dec_rev).c_str(), num(c.min_jerk).c_str(), num(c.max_jerk).c_str(),
          num(s.v).c_str(), num(s.v0).c_str(), num(s.v1).c_str(), num(s.dt).c_str(),
          num(v_all).c_str(), num(f_all).c_str(), num(v_val).c_str(), num(f_val).c_str(),
          num(v_d1).c_str(), num(f_d1).c_str(), num(v_d2).c_str(), num(f_d2).c_str());
        out += b;
        first = false;
      }
    }
    out += "\n  ],\n";
  }

  // ---- Odometry --------------------------------------------------------
  {
    // position_feedback path: Odometry::update(left_pos, right_pos, t)
    out += "  \"odometry_update\": [\n";
    struct OdomCase {
      const char * name;
      double sep, lr, rr;
      size_t window;
      bool init;  // call init(t0) or leave timestamp_ at 0
    };
    const OdomCase cases[] = {
      {"mower", 0.40, 0.10, 0.10, 10, true},
      {"mower_no_init", 0.40, 0.10, 0.10, 10, false},
      {"window1", 0.40, 0.10, 0.10, 1, true},
      {"asym_radius", 0.35, 0.098, 0.102, 4, true},
    };
    bool first = true;
    for (const auto & c : cases) {
      Odometry odom(c.window);
      odom.setWheelParams(c.sep, c.lr, c.rr);
      odom.setVelocityRollingWindowSize(c.window);
      const int64_t t0 = 1700000000000000000LL;  // a realistic ROS wall stamp
      if (c.init) odom.init(rclcpp::Time(t0));
      // left/right wheel positions in rad, 25 Hz
      double lp = 0.0, rp = 0.0;
      std::string steps;
      for (int i = 1; i <= 60; ++i) {
        double lv, rv;
        if (i <= 15) { lv = 4.0; rv = 4.0; }            // straight
        else if (i <= 30) { lv = 2.0; rv = 6.0; }       // arc left
        else if (i <= 35) { lv = 0.0; rv = 0.0; }       // stop (dt only)
        else if (i <= 50) { lv = -3.0; rv = 3.0; }      // spin in place
        else { lv = 5.0; rv = 4.9; }                    // near-straight
        lp += lv * 0.04;
        rp += rv * 0.04;
        const int64_t t = t0 + static_cast<int64_t>(i) * 40000000LL;
        bool ok = odom.update(lp, rp, rclcpp::Time(t));
        char b[768];
        std::snprintf(b, sizeof(b),
          "%s      {\"t_ns\": %lld, \"left_pos\": %s, \"right_pos\": %s, \"ok\": %s, "
          "\"x\": %s, \"y\": %s, \"heading\": %s, \"linear\": %s, \"angular\": %s}",
          i == 1 ? "" : ",\n", static_cast<long long>(t), num(lp).c_str(), num(rp).c_str(),
          ok ? "true" : "false", num(odom.getX()).c_str(), num(odom.getY()).c_str(),
          num(odom.getHeading()).c_str(), num(odom.getLinear()).c_str(),
          num(odom.getAngular()).c_str());
        steps += b;
      }
      char h[512];
      std::snprintf(h, sizeof(h),
        "%s    {\"name\": \"%s\", \"wheel_separation\": %s, \"left_radius\": %s, "
        "\"right_radius\": %s, \"window\": %zu, \"init\": %s, \"t0_ns\": %lld, \"steps\": [\n",
        first ? "" : ",\n", c.name, num(c.sep).c_str(), num(c.lr).c_str(), num(c.rr).c_str(),
        c.window, c.init ? "true" : "false", 1700000000000000000LL);
      out += h + steps + "\n    ]}";
      first = false;
    }
    out += "\n  ],\n";
  }

  // updateFromVelocity / updateOpenLoop, and the two integration branches
  {
    out += "  \"odometry_misc\": [\n";
    bool first = true;
    // updateFromVelocity: the arguments are wheel *distances* over the step
    {
      Odometry odom(10);
      odom.setWheelParams(0.40, 0.10, 0.10);
      const int64_t t0 = 1000000000LL;
      odom.init(rclcpp::Time(t0));
      std::string steps;
      for (int i = 1; i <= 20; ++i) {
        double lv = 0.1 * std::sin(i * 0.3), rv = 0.1 * std::cos(i * 0.21);
        int64_t t = t0 + static_cast<int64_t>(i) * 40000000LL;
        bool ok = odom.updateFromVelocity(lv, rv, rclcpp::Time(t));
        char b[768];
        std::snprintf(b, sizeof(b),
          "%s      {\"t_ns\": %lld, \"left\": %s, \"right\": %s, \"ok\": %s, \"x\": %s, \"y\": %s, "
          "\"heading\": %s, \"linear\": %s, \"angular\": %s}",
          i == 1 ? "" : ",\n", static_cast<long long>(t), num(lv).c_str(), num(rv).c_str(),
          ok ? "true" : "false", num(odom.getX()).c_str(), num(odom.getY()).c_str(),
          num(odom.getHeading()).c_str(), num(odom.getLinear()).c_str(), num(odom.getAngular()).c_str());
        steps += b;
      }
      out += std::string(first ? "" : ",\n") +
        "    {\"name\": \"update_from_velocity\", \"t0_ns\": 1000000000, \"steps\": [\n" + steps + "\n    ]}";
      first = false;
    }
    // updateOpenLoop, incl. the |angular| < 1e-6 Runge-Kutta 2 branch
    {
      Odometry odom(10);
      odom.setWheelParams(0.40, 0.10, 0.10);
      const int64_t t0 = 1000000000LL;
      odom.init(rclcpp::Time(t0));
      const double lin[] = {0.5, 0.5, 0.5, 0.0, -0.4, 0.3, 0.3, 0.3};
      const double ang[] = {0.0, 1e-7, 2.4999e-5, 1.0, -1.0, 1e-6, -1e-6, 0.5};
      std::string steps;
      for (int i = 0; i < 8; ++i) {
        int64_t t = t0 + static_cast<int64_t>(i + 1) * 40000000LL;
        odom.updateOpenLoop(lin[i], ang[i], rclcpp::Time(t));
        char b[768];
        std::snprintf(b, sizeof(b),
          "%s      {\"t_ns\": %lld, \"linear\": %s, \"angular\": %s, \"x\": %s, \"y\": %s, "
          "\"heading\": %s, \"out_linear\": %s, \"out_angular\": %s}",
          i == 0 ? "" : ",\n", static_cast<long long>(t), num(lin[i]).c_str(), num(ang[i]).c_str(),
          num(odom.getX()).c_str(), num(odom.getY()).c_str(), num(odom.getHeading()).c_str(),
          num(odom.getLinear()).c_str(), num(odom.getAngular()).c_str());
        steps += b;
      }
      out += ",\n    {\"name\": \"update_open_loop\", \"t0_ns\": 1000000000, \"steps\": [\n" + steps + "\n    ]}";
    }
    out += "\n  ],\n";
  }

  // ---- full controller cycle ------------------------------------------
  // Transcribed from diff_drive_controller.cpp update_reference_from_subscribers()
  // + update_and_write_commands(), parameterised from mower_controllers.yaml.
  {
    const double wheel_separation = 1.0 * 0.40;
    const double left_wheel_radius = 1.0 * 0.10;
    const double right_wheel_radius = 1.0 * 0.10;
    const double cmd_vel_timeout = 0.5;
    const int64_t publish_period_ns = static_cast<int64_t>(1.0 / 25.0 * 1e9);

    Odometry odom(10);
    odom.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
    odom.setVelocityRollingWindowSize(10);
    SpeedLimiter lim_lin(-0.55, 0.55, NaN, 0.8, NaN, NaN, NaN, NaN);
    SpeedLimiter lim_ang(-2.0, 2.0, NaN, 2.0, NaN, NaN, NaN, NaN);

    std::queue<std::array<double, 2>> prev2;
    prev2.push({{0.0, 0.0}});
    prev2.push({{0.0, 0.0}});

    const int64_t t0 = 1700000000000000000LL;
    const int64_t dt_ns = 40000000LL;  // 25 Hz
    int64_t previous_publish = t0;     // set in on_configure

    // Reference held between cycles (the exported reference interfaces)
    double ref_lin = NaN, ref_ang = NaN;
    // last command message and its stamp
    double cmd_lin = NaN, cmd_ang = NaN;
    int64_t cmd_stamp = t0;
    bool command_timed_out = false;

    // simple plant: the wheels reach last cycle's commanded velocity exactly
    double plant_left_vel = 0.0, plant_right_vel = 0.0;
    double left_pos = 0.0, right_pos = 0.0;

    std::string steps;
    const int N = 200;
    for (int i = 0; i < N; ++i) {
      const int64_t t = t0 + static_cast<int64_t>(i) * dt_ns;
      const double period = static_cast<double>(dt_ns) / 1e9;

      // new cmd_vel this cycle?
      bool have_cmd = true;
      double in_lin = 0.0, in_ang = 0.0;
      if (i < 40) { in_lin = 0.4; in_ang = 0.0; }
      else if (i < 80) { in_lin = 0.4; in_ang = 0.8; }
      else if (i < 100) { have_cmd = false; }          // silence -> cmd_vel timeout
      else if (i < 140) { in_lin = -0.3; in_ang = -1.5; }
      else if (i < 160) { in_lin = 5.0; in_ang = 10.0; }  // beyond the velocity limits
      else { in_lin = 0.0; in_ang = 0.0; }
      if (have_cmd) { cmd_lin = in_lin; cmd_ang = in_ang; cmd_stamp = t; }

      // --- update_reference_from_subscribers -------------------------------
      const double age = static_cast<double>(t) / 1e9 - static_cast<double>(cmd_stamp) / 1e9;
      const bool timeout_disabled = cmd_vel_timeout == 0.0;
      if (!timeout_disabled && age > cmd_vel_timeout) {
        ref_lin = 0.0;
        ref_ang = 0.0;
        command_timed_out = true;
      } else if (std::isfinite(cmd_lin) && std::isfinite(cmd_ang)) {
        command_timed_out = false;
        ref_lin = cmd_lin;
        ref_ang = cmd_ang;
      }

      // --- update_and_write_commands ---------------------------------------
      double linear_command = ref_lin;
      double angular_command = ref_ang;
      double vel_left = 0.0, vel_right = 0.0;
      bool wrote = false;
      bool published = false;
      if (std::isfinite(linear_command) && std::isfinite(angular_command)) {
        double & last_linear = prev2.back()[0];
        double & second_to_last_linear = prev2.front()[0];
        double & last_angular = prev2.back()[1];
        double & second_to_last_angular = prev2.front()[1];
        lim_lin.limit(linear_command, last_linear, second_to_last_linear, period);
        lim_ang.limit(angular_command, last_angular, second_to_last_angular, period);
        prev2.pop();
        prev2.push({{linear_command, angular_command}});

        odom.update(left_pos, right_pos, rclcpp::Time(t));

        if (previous_publish + publish_period_ns < t) {
          previous_publish += publish_period_ns;
          published = true;
        }

        vel_left = (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
        vel_right = (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;
        wrote = true;
      }

      const double heading = odom.getHeading();
      const double qz = std::sin(heading * 0.5);
      const double qw = std::cos(heading * 0.5);

      char b[1536];
      std::snprintf(b, sizeof(b),
        "%s    {\"i\": %d, \"t_ns\": %lld, \"cmd\": %s, \"left_pos\": %s, \"right_pos\": %s, "
        "\"timed_out\": %s, \"linear_command\": %s, \"angular_command\": %s, "
        "\"wrote\": %s, \"velocity_left\": %s, \"velocity_right\": %s, "
        "\"published\": %s, \"x\": %s, \"y\": %s, \"heading\": %s, \"qz\": %s, \"qw\": %s, "
        "\"odom_linear\": %s, \"odom_angular\": %s}",
        i == 0 ? "" : ",\n", i, static_cast<long long>(t),
        have_cmd ? ("[" + num(in_lin) + ", " + num(in_ang) + "]").c_str() : "null",
        num(left_pos).c_str(), num(right_pos).c_str(), command_timed_out ? "true" : "false",
        num(linear_command).c_str(), num(angular_command).c_str(), wrote ? "true" : "false",
        num(vel_left).c_str(), num(vel_right).c_str(), published ? "true" : "false",
        num(odom.getX()).c_str(), num(odom.getY()).c_str(), num(heading).c_str(),
        num(qz).c_str(), num(qw).c_str(), num(odom.getLinear()).c_str(),
        num(odom.getAngular()).c_str());
      steps += b;

      // advance the plant for the next cycle
      left_pos += plant_left_vel * period;
      right_pos += plant_right_vel * period;
      plant_left_vel = vel_left;
      plant_right_vel = vel_right;
    }
    out += "  \"controller_cycle\": {\n"
           "    \"wheel_separation\": 0.4, \"wheel_radius\": 0.1, \"cmd_vel_timeout\": 0.5,\n"
           "    \"update_rate\": 25.0, \"publish_rate\": 25.0, \"velocity_rolling_window_size\": 10,\n"
           "    \"t0_ns\": 1700000000000000000,\n"
           "    \"pose_covariance_diagonal\": [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],\n"
           "    \"twist_covariance_diagonal\": [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],\n"
           "    \"steps\": [\n" + steps + "\n    ]\n  }\n";
  }

  out += "}\n";
  std::fputs(out.c_str(), stdout);
  return 0;
}
