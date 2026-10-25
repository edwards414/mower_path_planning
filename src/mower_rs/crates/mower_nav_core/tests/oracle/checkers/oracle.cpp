// Oracle for mower_nav_core::checkers: drives the installed nav2_controller
// (Jazzy 1.3.13) SimpleGoalChecker and SimpleProgressChecker, initialised on
// a node that loads nav2_no_map_params.yaml as controller_server (plus two
// other configurations to reach the remaining branches), and dumps every
// call with its inputs and result as JSON. The progress checker's clock is
// the node's ROS clock with the time override enabled, set before each call.
#include <cmath>
#include <fstream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nlohmann/json.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"

using nlohmann::json;
using geometry_msgs::msg::Pose;

// The state the harness reads back is protected; these only re-expose it.
struct OracleGoalChecker : public nav2_controller::SimpleGoalChecker
{
  bool check_xy() const {return check_xy_;}
};

static Pose pose(double x, double y, double yaw)
{
  Pose p;
  p.position.x = x;
  p.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

// A pose from setRPY(0, 0, yaw) has orientation x = y = 0: (x, y, qz, qw).
static json pose_json(const Pose & p)
{
  return {p.position.x, p.position.y, p.orientation.z, p.orientation.w};
}

static void check_ok(rcl_ret_t ret, const char * what)
{
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(what);
  }
}

// --- goal checker -------------------------------------------------------------
//
// ops: ["goal", x, y, qz, qw] (the goal of the following calls),
// ["reached", x, y, qz, qw, vx, vw, result, check_xy], ["reset", check_xy],
// ["tolerances", ok, position[3], orientation[4], linear[3], angular[3]]

struct GoalRun
{
  std::shared_ptr<OracleGoalChecker> checker;
  json ops = json::array();
  json last_goal;

  void reached(const Pose & q, const Pose & g, double vx = 0.0, double vw = 0.0)
  {
    json gj = pose_json(g);
    if (gj != last_goal) {
      ops.push_back(json::array({"goal", gj[0], gj[1], gj[2], gj[3]}));
      last_goal = gj;
    }
    geometry_msgs::msg::Twist v;
    v.linear.x = vx;
    v.angular.z = vw;
    bool r = checker->isGoalReached(q, g, v);
    json qj = pose_json(q);
    ops.push_back(json::array({"reached", qj[0], qj[1], qj[2], qj[3], vx, vw, r,
        checker->check_xy()}));
  }
  void reset()
  {
    checker->reset();
    ops.push_back(json::array({"reset", checker->check_xy()}));
  }
  void tolerances()
  {
    Pose pt;
    geometry_msgs::msg::Twist vt;
    bool ok = checker->getTolerances(pt, vt);
    ops.push_back(json::array({"tolerances", ok,
        {pt.position.x, pt.position.y, pt.position.z},
        {pt.orientation.x, pt.orientation.y, pt.orientation.z, pt.orientation.w},
        {vt.linear.x, vt.linear.y, vt.linear.z},
        {vt.angular.x, vt.angular.y, vt.angular.z}}));
  }
};

static json goal_params(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string & name)
{
  return {{"xy_goal_tolerance", node->get_parameter(name + ".xy_goal_tolerance").as_double()},
    {"yaw_goal_tolerance", node->get_parameter(name + ".yaw_goal_tolerance").as_double()},
    {"stateful", node->get_parameter(name + ".stateful").as_bool()},
    {"symmetric_yaw_tolerance", node->get_parameter(name + ".symmetric_yaw_tolerance").as_bool()}};
}

static json goal_scenarios(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::mt19937 & rng)
{
  json out = json::array();
  for (const std::string name : {"general_goal_checker", "default_goal_checker",
      "stateless_symmetric_goal_checker"})
  {
    GoalRun run;
    run.checker = std::make_shared<OracleGoalChecker>();
    run.checker->initialize(node, name, nullptr);
    run.tolerances();

    // approaches: straight, off by 3 cm (inside 5 cm), off by 6 cm (never
    // inside 5 cm), an arc and a spiral; each from 0.4 m out to beyond the goal
    const double gx = 1.234, gy = -0.567, gyaw = 0.8;
    const Pose goal = pose(gx, gy, gyaw);
    for (double offset : {0.0, 0.03, 0.06, 0.2}) {
      run.reset();
      for (int i = 0; i <= 80; ++i) {
        double s = -0.4 + 0.007 * i;  // along the heading
        double x = gx + s * std::cos(gyaw) - offset * std::sin(gyaw);
        double y = gy + s * std::sin(gyaw) + offset * std::cos(gyaw);
        run.reached(pose(x, y, gyaw + 0.02 * std::sin(i)), goal, 0.26, 0.1);
      }
    }
    run.reset();
    for (int i = 0; i <= 60; ++i) {  // arc of radius 0.5 through the goal
      double a = -0.9 + 0.025 * i;
      double x = gx + 0.5 * std::sin(a), y = gy - 0.5 + 0.5 * std::cos(a);
      run.reached(pose(x, y, -a + 3.0), goal);
    }
    run.reset();
    for (int i = 0; i <= 90; ++i) {  // spiral in, the yaw sweeping all the way round
      double r = 0.3 * std::exp(-0.04 * i), a = 0.37 * i;
      run.reached(pose(gx + r * std::cos(a), gy + r * std::sin(a), 0.21 * i), goal);
    }

    // the xy boundary: points on and one ulp either side of the tolerance
    // circle, so the squared distance compares with and without FMA
    double tol = node->get_parameter(name + ".xy_goal_tolerance").as_double();
    for (int k = 0; k < 48; ++k) {
      double a = 2.0 * M_PI * k / 48.0 + 0.013;
      for (int side = -1; side <= 1; ++side) {
        double r = tol;
        if (side < 0) {r = std::nextafter(tol, 0.0);}
        if (side > 0) {r = std::nextafter(tol, 1.0);}
        run.reset();
        run.reached(pose(gx + r * std::cos(a), gy + r * std::sin(a), gyaw), goal);
      }
    }
    for (const auto & d : std::vector<std::pair<double, double>>{{tol, 0.0}, {-tol, 0.0},
        {0.0, tol}, {0.0, -tol}, {0.6 * tol, 0.8 * tol}, {-0.8 * tol, 0.6 * tol}})
    {
      run.reset();
      run.reached(pose(d.first, d.second, 0.0), pose(0.0, 0.0, 0.0));
      run.reset();
      run.reached(pose(gx + d.first, gy + d.second, 0.0), goal);
    }

    // yaw errors around this checker's tolerance and around pi, both signs
    double ytol = node->get_parameter(name + ".yaw_goal_tolerance").as_double();
    for (double e : {0.0, 0.1, ytol - 1e-9, ytol + 1e-9, ytol - 1e-6, ytol + 1e-6, 3.139, 3.1399,
        3.1401, 3.141, M_PI - 1e-9, M_PI, M_PI + 1e-9, 3.2, M_PI / 2, 4.0, 6.0, 2 * M_PI - 3.14 - 1e-6,
        2 * M_PI - 3.14 + 1e-6, M_PI - ytol - 1e-6, M_PI - ytol + 1e-6})
    {
      for (double sign : {1.0, -1.0}) {
        for (double base : {0.3, -2.9, 3.1}) {
          run.reset();
          run.reached(pose(gx + 0.01, gy - 0.01, base), pose(gx, gy, base + sign * e));
        }
      }
    }

    // the latch: inside xy with the yaw way off, then drift out with the yaw
    // right, then reset
    run.reset();
    run.reached(pose(gx + 0.02, gy, gyaw + M_PI), goal);
    run.reached(pose(gx + 0.04, gy, gyaw + M_PI - 0.0005), goal);
    run.reached(pose(gx + 1.0, gy + 2.0, gyaw), goal);
    run.reached(pose(gx + 5.0, gy, gyaw + 0.2), goal);
    run.reset();
    run.reached(pose(gx + 1.0, gy + 2.0, gyaw), goal);
    run.reached(pose(gx, gy, gyaw + M_PI), goal);
    run.reached(pose(gx + 0.3, gy, gyaw), goal);

    // random poses around random goals, random resets
    std::uniform_real_distribution<double> u(-1.0, 1.0), ua(-M_PI, M_PI), u01(0.0, 1.0);
    Pose g = goal;
    for (int i = 0; i < 250; ++i) {
      if (u01(rng) < 0.05) {
        g = pose(5 * u(rng), 5 * u(rng), ua(rng));
      }
      if (u01(rng) < 0.1) {
        run.reset();
      }
      double r = 0.15 * u01(rng) * u01(rng), a = ua(rng);
      double yaw = u01(rng) < 0.3 ? ua(rng) : tf2::getYaw(g.orientation) + M_PI + 0.01 * u(rng);
      run.reached(pose(g.position.x + r * std::cos(a), g.position.y + r * std::sin(a), yaw), g);
    }
    out.push_back({{"name", name}, {"params", goal_params(node, name)}, {"ops", run.ops}});
  }
  return out;
}

// --- progress checker -----------------------------------------------------------

struct ProgressRun
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_controller::SimpleProgressChecker> checker;
  json ops = json::array();

  // ops: ["check", t, x, y, result] (the yaw is not used), ["reset"]
  void check(int64_t t, double x, double y, double yaw = 0.0)
  {
    check_ok(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), t), "set time");
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "odom";
    p.pose = pose(x, y, yaw);
    bool r = checker->check(p);
    ops.push_back(json::array({"check", t, x, y, r}));
  }
  void reset()
  {
    checker->reset();
    ops.push_back(json::array({"reset"}));
  }
};

static json progress_scenarios(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::mt19937 & rng)
{
  json out = json::array();
  const int64_t s = 1000000000LL, ms = 1000000LL;
  for (const std::string name : {"progress_checker", "default_progress_checker"}) {
    ProgressRun run;
    run.node = node;
    run.checker = std::make_shared<nav2_controller::SimpleProgressChecker>();
    run.checker->initialize(node, name);
    double radius = node->get_parameter(name + ".required_movement_radius").as_double();
    int64_t allowance = static_cast<int64_t>(
      node->get_parameter(name + ".movement_time_allowance").as_double() * s);
    std::uniform_real_distribution<double> u(-1.0, 1.0), u01(0.0, 1.0);

    // standing still with jitter well inside the radius: passes up to and
    // including exactly the allowance, fails one ns later and from then on
    int64_t t0 = 1761400000LL * s + 123456789LL;
    run.reset();
    int64_t t = t0;
    for (; t <= t0 + allowance; t += 50 * ms) {
      run.check(t, 2.0 + 0.3 * radius * u(rng), -1.0 + 0.3 * radius * u(rng));
    }
    run.check(t0 + allowance, 2.0, -1.0);
    run.check(t0 + allowance + 1, 2.0, -1.0);
    for (int i = 0; i < 10; ++i, t += 50 * ms) {
      run.check(t, 2.0, -1.0);
    }
    // moving on clears it (re-baseline), standing still again counts afresh
    run.check(t, 2.0 + 1.01 * radius, -1.0);
    t += 50 * ms;
    run.check(t + allowance, 2.0 + 1.01 * radius, -1.0);
    run.check(t + allowance + 1, 2.0 + 1.01 * radius, -1.0);

    // exactly the radius along an axis is not enough (strict >)
    t += allowance + s;
    run.reset();
    run.check(t, 0.0, 0.0);
    run.check(t + s, radius, 0.0);
    run.check(t + 2 * s, 0.0, -radius);
    run.check(t + allowance + 1, -radius, 0.0);
    run.check(t + allowance + 2, std::nextafter(radius, 1.0), 0.0);
    run.check(t + allowance + 3, std::nextafter(radius, 1.0), 0.0);
    run.check(t + 2 * allowance + 3, std::nextafter(radius, 1.0), radius);
    run.check(t + 2 * allowance + 4, std::nextafter(radius, 1.0), radius);

    // the radius is from the baseline, not from the last pose: a slow creep
    // re-baselines only when it has gone the whole radius
    t += 3 * allowance;
    run.reset();
    double x = 7.0;
    for (int i = 0; i < 200; ++i, t += 50 * ms) {
      x += radius / 37.0;
      run.check(t, x, 3.0);
    }
    // a creep too slow to cover the radius within the allowance: fails from
    // the allowance until it has covered the radius, then passes again
    run.reset();
    double step = radius / (allowance / (50 * ms)) * 0.7;
    for (int i = 0; i < (allowance / (50 * ms)) * 3 / 2; ++i, t += 50 * ms) {
      x += step;
      run.check(t, x, 3.0);
    }

    // reset part-way restarts the allowance; a reset without a check changes nothing else
    run.reset();
    run.check(t, x, 3.0);
    run.check(t + allowance * 3 / 4, x, 3.0);
    run.reset();
    run.check(t + allowance * 3 / 4 + 1, x, 3.0);
    run.check(t + allowance * 3 / 2, x, 3.0);
    run.check(t + allowance * 7 / 4 + 1, x, 3.0);
    run.check(t + allowance * 7 / 4 + 2, x, 3.0);
    // time going backwards
    run.check(t - s, x, 3.0);

    // random walks with random steps in time and random resets
    t += 2 * allowance;
    double y = 0.0;
    for (int i = 0; i < 300; ++i) {
      if (u01(rng) < 0.03) {
        run.reset();
      }
      double r = u01(rng);
      t += r < 0.05 ? static_cast<int64_t>(allowance * u01(rng)) : static_cast<int64_t>(
        100 * ms * u01(rng));
      x += 0.4 * radius * u(rng) * u01(rng);
      y += 0.4 * radius * u(rng) * u01(rng);
      run.check(t, x, y);
    }

    out.push_back({{"name", name}, {"params", {{"required_movement_radius", radius},
      {"movement_time_allowance",
        node->get_parameter(name + ".movement_time_allowance").as_double()}}},
      {"ops", run.ops}});
  }
  return out;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::mt19937 rng(20261025);

  // controller_server as the robot configures it, plus two extra plugin
  // instances: upstream defaults, and stateless with symmetric yaw tolerance
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", NAV2_PARAMS});
  options.parameter_overrides({
    {"stateless_symmetric_goal_checker.stateful", false},
    {"stateless_symmetric_goal_checker.symmetric_yaw_tolerance", true},
    {"stateless_symmetric_goal_checker.xy_goal_tolerance", 0.1},
    {"stateless_symmetric_goal_checker.yaw_goal_tolerance", 0.3}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
  check_ok(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), "enable time");

  json out;
  out["goal_checker"] = goal_scenarios(node, rng);
  out["progress_checker"] = progress_scenarios(node, rng);

  std::ofstream(argv[argc - 1]) << out.dump() << "\n";
  rclcpp::shutdown();
  return 0;
}
