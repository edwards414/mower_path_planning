// Oracle for mower_nav_core::smoother: drives the installed
// nav2_velocity_smoother (Jazzy 1.3.13) VelocitySmoother node, configured
// from nav2_no_map_params.yaml (and from a few other parameter sets to reach
// the branches this robot does not), and dumps every input command, every
// timer firing and every message it publishes as JSON.
//
// The node is driven deterministically, never spun:
// * its ROS clock has the time override enabled and is set before each call,
//   so now() is exactly the event time;
// * the wall timer is cancelled after activation and smootherTimer() is
//   called directly, as is inputCommandStampedCallback() (a subclass
//   re-exposes the protected members);
// * the output is the real publication on cmd_vel_smoothed, taken by an
//   intra-process subscription in this process. Whether a call published is
//   known exactly: the harness puts a sentinel in last_cmd_.header.frame_id
//   (a field the smoother never reads) and the timer overwrites last_cmd_ iff
//   it reaches publish().
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_velocity_smoother/velocity_smoother.hpp"
#include "nlohmann/json.hpp"
#include "rcl/time.h"
#include "rcl/timer.h"
#include "rclcpp/rclcpp.hpp"

using nlohmann::json;
using geometry_msgs::msg::TwistStamped;
using V6 = std::array<double, 6>;  // linear x y z, angular x y z

static const int64_t kS = 1000000000LL, kMs = 1000000LL;
static const int64_t kT0 = 1761400000LL * kS;  // 2025-10-25
static const char * kSentinel = "<oracle: not published>";
static const double kNan = std::numeric_limits<double>::quiet_NaN();
static const double kInf = std::numeric_limits<double>::infinity();

struct OracleSmoother : public nav2_velocity_smoother::VelocitySmoother
{
  explicit OracleSmoother(const rclcpp::NodeOptions & options)
  : VelocitySmoother(options) {}
  using VelocitySmoother::inputCommandStampedCallback;
  using VelocitySmoother::inputCommandCallback;
  using VelocitySmoother::smootherTimer;
  bool stopped() const {return stopped_;}
  const void * command() const {return command_.get();}
  TwistStamped & last_cmd() {return last_cmd_;}
  int64_t timer_period_ns() const
  {
    int64_t period = 0;
    if (rcl_timer_get_period(timer_->get_timer_handle().get(), &period) != RCL_RET_OK) {
      throw std::runtime_error("rcl_timer_get_period");
    }
    return period;
  }
  void cancel_timer() {timer_->cancel();}
};

static void check_ok(rcl_ret_t ret, const char * what)
{
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(what);
  }
}

// NaN and infinities as strings: JSON has no literal for them.
static json num(double v)
{
  if (std::isnan(v)) {return "nan";}
  if (std::isinf(v)) {return v > 0 ? "inf" : "-inf";}
  return v;
}

static json v6_json(const V6 & v)
{
  json a = json::array();
  for (double x : v) {a.push_back(num(x));}
  return a;
}

static builtin_interfaces::msg::Time to_stamp(int64_t ns)
{
  int64_t sec = ns / kS, rem = ns % kS;
  if (rem < 0) {
    sec -= 1;
    rem += kS;
  }
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(sec);
  t.nanosec = static_cast<uint32_t>(rem);
  return t;
}

static int64_t from_stamp(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * kS + t.nanosec;
}

static V6 planar(double x, double y, double w) {return {x, y, 0.0, 0.0, 0.0, w};}

// One VelocitySmoother node and its recorder.
//
// events: ["in", t, stamp_ns, frame, v[6], accepted, threw],
// ["in_twist", t, v[6], accepted],
// ["tick", t, null | [stamp_ns, frame, v[6]], stopped]
struct Run
{
  std::string name;
  std::shared_ptr<OracleSmoother> node;
  rclcpp::Node::SharedPtr recorder;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub;
  rclcpp::executors::SingleThreadedExecutor exec;
  std::vector<TwistStamped> received;
  json events = json::array();
  int64_t t = kT0;
  bool configured = false;

  Run(const std::string & name_, bool production, const std::vector<rclcpp::Parameter> & overrides)
  : name(name_)
  {
    std::vector<std::string> args{"--ros-args"};
    if (production) {
      args.insert(args.end(), {"--params-file", NAV2_PARAMS});
    }
    args.insert(args.end(), {"-r", "cmd_vel:=/oracle/" + name + "/cmd_vel",
      "-r", "cmd_vel_smoothed:=/oracle/" + name + "/cmd_vel_smoothed"});
    rclcpp::NodeOptions options;
    options.arguments(args).parameter_overrides(overrides).use_intra_process_comms(true);
    node = std::make_shared<OracleSmoother>(options);
    check_ok(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), "enable time");

    recorder = std::make_shared<rclcpp::Node>(
      "recorder_" + name, rclcpp::NodeOptions().use_intra_process_comms(true));
    sub = recorder->create_subscription<TwistStamped>(
      "/oracle/" + name + "/cmd_vel_smoothed", rclcpp::QoS(10),
      [this](TwistStamped::ConstSharedPtr m) {received.push_back(*m);});
    exec.add_node(recorder);

    configured = node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    if (configured) {
      if (node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        throw std::runtime_error(name + ": activate failed");
      }
      node->cancel_timer();
    }
  }

  void set_time(int64_t now)
  {
    check_ok(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), now), "set time");
  }

  void input(int64_t now, int64_t stamp_ns, const std::string & frame, const V6 & v)
  {
    set_time(now);
    auto msg = std::make_shared<TwistStamped>();
    msg->header.stamp = to_stamp(stamp_ns);
    msg->header.frame_id = frame;
    msg->twist.linear.x = v[0];
    msg->twist.linear.y = v[1];
    msg->twist.linear.z = v[2];
    msg->twist.angular.x = v[3];
    msg->twist.angular.y = v[4];
    msg->twist.angular.z = v[5];
    bool threw = false;
    try {
      node->inputCommandStampedCallback(msg);
    } catch (const std::runtime_error &) {
      threw = true;
    }
    events.push_back(json::array({"in", now, stamp_ns, frame, v6_json(v),
        node->command() == msg.get(), threw}));
  }

  void input_twist(int64_t now, const V6 & v)
  {
    set_time(now);
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = v[0];
    msg->linear.y = v[1];
    msg->linear.z = v[2];
    msg->angular.x = v[3];
    msg->angular.y = v[4];
    msg->angular.z = v[5];
    const void * before = node->command();
    node->inputCommandCallback(msg);
    events.push_back(json::array({"in_twist", now, v6_json(v), node->command() != before}));
  }

  void tick(int64_t now)
  {
    set_time(now);
    node->last_cmd().header.frame_id = kSentinel;
    size_t before = received.size();
    node->smootherTimer();
    bool published = node->last_cmd().header.frame_id != kSentinel;
    auto deadline = std::chrono::steady_clock::now() +
      (published ? std::chrono::milliseconds(5000) : std::chrono::milliseconds(2));
    do {
      exec.spin_some();
    } while (received.size() == before && std::chrono::steady_clock::now() < deadline);
    if (received.size() != before + (published ? 1 : 0)) {
      throw std::runtime_error(name + ": publication does not match the timer's path");
    }
    json out = nullptr;
    if (published) {
      const auto & m = received.back();
      out = json::array({from_stamp(m.header.stamp), m.header.frame_id,
          v6_json({m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
            m.twist.angular.x, m.twist.angular.y, m.twist.angular.z})});
    }
    events.push_back(json::array({"tick", now, out, node->stopped()}));
  }

  // n cycles of a 20 Hz controller: the command arrives 13 ms before the
  // tick, stamped 2 ms before it arrives (controller_server stamps now())
  void drive(int n, const V6 & v, const std::string & frame = "base_footprint")
  {
    for (int i = 0; i < n; ++i) {
      t += 50 * kMs;
      input(t - 13 * kMs, t - 15 * kMs, frame, v);
      tick(t);
    }
  }

  void idle(int n)
  {
    for (int i = 0; i < n; ++i) {
      t += 50 * kMs;
      tick(t);
    }
  }

  // random commands, stamps and timing
  void random(std::mt19937 & rng, int n, double vx, double vy, double vw)
  {
    std::uniform_real_distribution<double> u(-1.0, 1.0), u01(0.0, 1.0);
    for (int i = 0; i < n; ++i) {
      double r = u01(rng);
      if (r < 0.5) {
        t += static_cast<int64_t>(30 * kMs * u01(rng));
        double s = u01(rng);
        int64_t stamp = t - static_cast<int64_t>(5 * kMs * u01(rng));
        if (s < 0.07) {
          stamp = 0;
        } else if (s < 0.14) {
          stamp = t - static_cast<int64_t>(2.5 * kS * u01(rng));
        } else if (s < 0.2) {
          stamp = t + static_cast<int64_t>(1.5 * kS * u01(rng));
        }
        V6 v = u01(rng) < 0.2 ? planar(0.0, 0.0, 0.0) :
          planar(vx * u(rng), u01(rng) < 0.3 ? vy * u(rng) : 0.0, vw * u(rng));
        input(t, stamp, u01(rng) < 0.5 ? "base_footprint" : "odom", v);
      } else {
        if (u01(rng) < 0.04) {
          t += static_cast<int64_t>(2.5 * kS * u01(rng));
        }
        t += 40 * kMs + static_cast<int64_t>(20 * kMs * u01(rng));
        tick(t);
      }
    }
  }

  json params()
  {
    json p = {{"configured", configured}};
    for (const char * k : {"max_velocity", "min_velocity", "max_accel", "max_decel"}) {
      p[k] = node->get_parameter(k).as_double_array();
    }
    if (!configured) {
      return p;
    }
    p["smoothing_frequency"] = node->get_parameter("smoothing_frequency").as_double();
    p["scale_velocities"] = node->get_parameter("scale_velocities").as_bool();
    p["feedback"] = node->get_parameter("feedback").as_string();
    p["deadband_velocity"] = node->get_parameter("deadband_velocity").as_double_array();
    p["velocity_timeout"] = node->get_parameter("velocity_timeout").as_double();
    p["stamp_smoothed_velocity_with_smoothing_time"] =
      node->get_parameter("stamp_smoothed_velocity_with_smoothing_time").as_bool();
    p["enable_stamped_cmd_vel"] = node->get_parameter("enable_stamped_cmd_vel").as_bool();
    p["timer_period_ns"] = node->timer_period_ns();
    return p;
  }

  json dump()
  {
    return {{"name", name}, {"params", params()}, {"events", events}};
  }
};

static std::vector<rclcpp::Parameter> base_overrides()
{
  return {{"bond_heartbeat_period", 0.0}, {"enable_stamped_cmd_vel", true}};
}

// --- this robot ---------------------------------------------------------------

static json steps()
{
  Run run("steps", true, {});
  run.tick(run.t);  // no command yet: nothing
  run.t += 50 * kMs;
  run.tick(run.t);
  run.drive(6, planar(0.26, 0, 0));  // accelerate past the limit
  run.drive(8, planar(-0.26, 0, 0));  // reverse through zero
  run.drive(6, planar(0.1, 0, 0));
  run.drive(6, planar(5.0, 0, 0));  // above max_velocity
  run.drive(10, planar(0.2, 0, 0.8));  // linear and angular together
  run.drive(12, {-0.1, 0.3, 0.0, 0.0, 0.0, -1.5});  // y is limited to 0, w to -1
  run.drive(10, planar(0, 0, 3.0));
  run.drive(14, planar(0, 0, -3.0));
  run.drive(5, planar(0.26, 0, 0));
  run.drive(6, planar(-5.0, 0, 0.0));
  run.drive(30, planar(0, 0, 0));  // zero commands: zeros go on being published
  run.drive(4, planar(0.05, 0, 0.05));  // steps smaller than the limit
  run.drive(4, planar(0.13, 0, 0.17));
  run.drive(3, planar(0.125, 0, 0.16));  // exactly one tick's worth
  run.drive(3, planar(0.0, 0, 0.0));
  run.drive(3, planar(0.26, 0, -1.0));
  run.drive(5, planar(-0.0, -0.0, -0.0));  // negative zeros
  run.drive(4, {0.1, 0.0, 0.7, -0.2, 0.3, 0.0});  // unused axes are dropped
  return run.dump();
}

static json timeout()
{
  Run run("timeout", true, {});
  run.drive(20, planar(0.26, 0, 1.0));
  // the controller goes quiet while the robot moves: the last command is
  // replayed until velocity_timeout after its stamp, then the robot is
  // ramped to zero (that zero once) and the smoother falls silent
  run.idle(60);
  run.drive(10, planar(0.15, 0, -0.5));  // resumption from rest
  // the timeout boundary: delta == velocity_timeout is not a timeout
  run.t += 50 * kMs;
  int64_t s = run.t;
  run.input(s, s, "base_footprint", planar(0.2, 0, 0.1));
  run.tick(s + kS - 1);
  run.tick(s + kS);
  run.tick(s + kS + 1);
  run.t = s + kS + 1;
  run.idle(10);
  // controller_server's publishZeroVelocity at the end of a goal: one zero
  // command, then nothing more
  run.drive(10, planar(0.26, 0, 0.6));
  run.t += 50 * kMs;
  run.input(run.t - 13 * kMs, run.t - 13 * kMs, "base_footprint", planar(0, 0, 0));
  run.tick(run.t);
  run.idle(40);
  run.drive(5, planar(0.1, 0, 0.2));  // resumption after silence
  run.idle(40);
  // a timeout while only turning, and a long silence at rest
  run.drive(8, planar(0.0, 0, -1.0));
  run.idle(200);
  return run.dump();
}

static json stamps()
{
  Run run("stamps", true, {});
  // a zero stamp: timed from its arrival, published with the zero stamp
  run.t += 50 * kMs;
  run.input(run.t - 13 * kMs, 0, "base_footprint", planar(0.2, 0, 0.3));
  run.tick(run.t);
  run.idle(30);
  // stale while at rest: nothing is published at all
  for (int i = 0; i < 5; ++i) {
    run.t += 50 * kMs;
    run.input(run.t - 13 * kMs, run.t - 2 * kS, "base_footprint", planar(0.2, 0, 0.0));
    run.tick(run.t);
  }
  // somewhat stale (0.6 s): taken, and times out 0.4 s after arrival
  run.t += 50 * kMs;
  run.input(run.t - 13 * kMs, run.t - 613 * kMs, "base_footprint", planar(0.26, 0, 0));
  run.tick(run.t);
  run.idle(30);
  // stale while moving: the next tick times out and ramps down
  run.drive(10, planar(0.26, 0, 0.5));
  run.t += 50 * kMs;
  run.input(run.t - 13 * kMs, run.t - 1500 * kMs, "base_footprint", planar(-0.2, 0, 0));
  run.tick(run.t);
  run.idle(10);
  // 5 s in the future: replayed until 1 s after that stamp
  run.t += 50 * kMs;
  run.input(run.t - 13 * kMs, run.t + 5 * kS, "base_footprint", planar(0.1, 0, -0.2));
  run.tick(run.t);
  run.idle(140);
  // non-finite values in any field are dropped; the last good command goes on
  run.drive(5, planar(0.2, 0, 0.1));
  for (const V6 & bad : std::vector<V6>{planar(kNan, 0, 0), planar(0, 0, kInf),
      {0, 0, kNan, 0, 0, 0}, {0, 0, 0, -kInf, 0, 0}, {0, 0, 0, 0, kNan, 0}, planar(0, kInf, 0),
      planar(-kInf, kNan, kInf)})
  {
    run.t += 50 * kMs;
    run.input(run.t - 13 * kMs, run.t - 15 * kMs, "base_footprint", bad);
    run.tick(run.t);
  }
  // frames are passed through
  run.drive(3, planar(0.1, 0, 0), "odom");
  run.drive(3, planar(0.15, 0, 0.2), "");
  // several commands between two ticks: the last one wins
  run.t += 50 * kMs;
  run.input(run.t - 40 * kMs, run.t - 41 * kMs, "a", planar(-0.26, 0, 1.0));
  run.input(run.t - 30 * kMs, run.t - 31 * kMs, "b", planar(0.26, 0, -1.0));
  run.input(run.t - 20 * kMs, run.t - 21 * kMs, "c", planar(0.05, 0, 0.0));
  run.tick(run.t);
  // a controller slower than the smoother: each command is replayed
  for (int i = 0; i < 10; ++i) {
    run.t += 50 * kMs;
    run.input(run.t - 13 * kMs, run.t - 15 * kMs, "base_footprint",
      planar(i % 2 ? 0.26 : -0.1, 0, i % 3 ? 0.9 : -0.4));
    run.tick(run.t);
    run.idle(2);
  }
  // irregular timer firings: the step per firing is fixed regardless
  double sign = 1.0;
  for (int64_t dt : {10, 200, 5, 120, 50, 1, 333, 0, 75}) {
    run.t += dt * kMs;
    run.input(run.t - kMs / 2, run.t - kMs, "base_footprint", planar(0.26 * sign, 0, -sign));
    run.tick(run.t);
    sign = -sign;
  }
  // stamps behind each other (out of order): the latest received wins
  run.t += 50 * kMs;
  run.input(run.t - 20 * kMs, run.t - 20 * kMs, "base_footprint", planar(0.2, 0, 0));
  run.input(run.t - 10 * kMs, run.t - 300 * kMs, "base_footprint", planar(0.1, 0, 0));
  run.tick(run.t);
  run.idle(25);
  // the unstamped callback (enable_stamped_cmd_vel: false): a zero stamp
  run.t += 50 * kMs;
  run.input_twist(run.t - 13 * kMs, planar(0.15, 0, 0.4));
  run.tick(run.t);
  run.t += 50 * kMs;
  run.input_twist(run.t - 13 * kMs, planar(kNan, 0, 0.4));
  run.tick(run.t);
  run.idle(30);
  return run.dump();
}

static json negative_stamp()
{
  // rclcpp::Time throws on a negative stamp, after command_ has been stored
  Run run("negative_stamp", true, {});
  run.t += 50 * kMs;
  run.input(run.t, -5 * kS, "base_footprint", planar(0.2, 0, 0));
  run.input(run.t, -1, "base_footprint", planar(0.2, 0, 0));
  return run.dump();
}

static json random_robot(std::mt19937 & rng)
{
  Run run("random_robot", true, {});
  run.random(rng, 1200, 0.4, 0.1, 1.5);
  return run.dump();
}

// --- other parameter sets: the branches this robot does not reach -------------

static json generic(std::mt19937 & rng, const std::string & name, bool scale, bool stamp_smoothed)
{
  auto o = base_overrides();
  o.insert(o.end(), {
    {"smoothing_frequency", 15.0},
    {"scale_velocities", scale},
    {"max_velocity", std::vector<double>{0.5, 0.3, 1.5}},
    {"min_velocity", std::vector<double>{-0.4, -0.3, -1.2}},
    {"max_accel", std::vector<double>{1.0, 0.5, 2.0}},
    {"max_decel", std::vector<double>{-2.0, -0.7, -3.0}},
    {"deadband_velocity", std::vector<double>{0.02, 0.01, 0.05}},
    {"velocity_timeout", 0.5},
    {"stamp_smoothed_velocity_with_smoothing_time", stamp_smoothed}});
  Run run(name, false, o);
  run.drive(6, planar(0.5, 0.3, 1.5));
  run.drive(8, planar(-0.4, -0.3, -1.2));  // every axis through zero at once
  run.drive(6, planar(0.3, -0.1, 0.0));
  run.drive(6, planar(0.0, 0.3, 0.2));
  run.drive(4, planar(0.01, 0.005, 0.04));  // inside the deadband
  run.drive(6, planar(2.0, 0.0, -0.1));
  run.idle(30);  // timeout and ramp down
  run.t += 50 * kMs;
  run.input(run.t - 13 * kMs, 0, "base_footprint", planar(0.3, 0.1, 0.5));
  run.idle(20);
  run.random(rng, 900, 0.7, 0.4, 2.0);
  return run.dump();
}

static json defaults(std::mt19937 & rng)
{
  Run run("defaults", false, base_overrides());
  run.drive(10, planar(0.6, 0.0, 3.0));
  run.drive(10, planar(-0.6, 0.0, -3.0));
  run.random(rng, 300, 0.7, 0.2, 3.0);
  return run.dump();
}

static json bad_configurations()
{
  json out = json::array();
  int i = 0;
  using A = std::vector<double>;
  for (const auto & p : std::vector<std::pair<std::string, A>>{
      {"max_decel", A{2.5, 0.0, -3.2}}, {"max_decel", A{-2.5, 0.1, -3.2}},
      {"max_accel", A{2.5, 0.0, -3.2}}, {"min_velocity", A{-0.5, 0.0, 0.5}},
      {"max_velocity", A{-0.1, 0.0, 2.5}}, {"max_velocity", A{0.0, 0.0, 0.0}},
      {"min_velocity", A{0.0, 0.0, 0.0}}, {"max_accel", A{0.0, 0.0, 0.0}}})
  {
    auto o = base_overrides();
    o.push_back({p.first, p.second});
    Run run("config_" + std::to_string(i++), false, o);
    out.push_back(run.params());
  }
  return out;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::mt19937 rng(20261025);
  json out;
  out["scenarios"] = json::array({steps(), timeout(), stamps(), negative_stamp(),
      random_robot(rng), generic(rng, "generic_scaled", true, true),
      generic(rng, "generic_unscaled", false, false), defaults(rng)});
  out["configurations"] = bad_configurations();
  std::ofstream(argv[argc - 1]) << out.dump() << "\n";
  rclcpp::shutdown();
  return 0;
}
