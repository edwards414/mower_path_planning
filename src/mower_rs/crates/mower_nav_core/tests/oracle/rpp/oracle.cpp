// Oracle for mower_nav_core::{rpp, path_handler, shim}: drives the installed
// nav2 (Jazzy 1.3.13) RotationShimController around RegulatedPurePursuit —
// configured from the robot's own nav2_no_map_params.yaml, as the controller
// server configures FollowPath — closed loop with a unicycle at 20 Hz, and
// records every control cycle's inputs and output (or exception) as JSON for
// an open-loop replay into the Rust port. Two more configurations ("variant_a",
// "variant_b") switch on the branches the robot's yaml leaves off.
//
// The local costmap is a real Costmap2DROS (frame odom, rolling 3 x 3 m,
// StaticLayer) configured from the same yaml but never activated, so there is
// no update thread: the harness feeds the StaticLayer its map and calls
// LayeredCostmap::updateMap itself at the yaml's update_frequency (2 Hz).
// TF goes into the costmap's own buffer, which is what the controller server
// hands its controllers; the node clock is overridden so that the shim's
// lookup at clock_->now() hits the odom -> base_footprint stamp exactly.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nlohmann/json.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

using nlohmann::json;
using nav_msgs::msg::Path;
using geometry_msgs::msg::PoseStamped;

// --- read-only access to protected state ------------------------------------

struct OracleShim : public nav2_rotation_shim_controller::RotationShimController
{
  bool inRotation() const {return in_rotation_;}
  bool pathUpdated() const {return path_updated_;}
  nav2_core::Controller * primary() const {return primary_controller_.get();}
};

struct RppAccess : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
  static size_t planSize(nav2_core::Controller * c)
  {
    auto * rpp = static_cast<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController *>(c);
    auto handler = &RppAccess::path_handler_;
    return (rpp->*handler)->getPlan().poses.size();
  }
};

struct StaticLayerAccess : public nav2_costmap_2d::StaticLayer
{
  static void feed(nav2_costmap_2d::StaticLayer * layer, const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
  {
    auto incoming = &StaticLayerAccess::incomingMap;
    (layer->*incoming)(map);
  }
};

// --- geometry helpers (harness only) ------------------------------------------

using Pts = std::vector<std::pair<double, double>>;
using Frame = std::array<double, 3>;  // x, y, yaw of `parent <- child`

// mower_nav geometry::densify_path: at most `step` between poses.
static Pts densify(const Pts & pts, double step)
{
  Pts out;
  if (pts.empty()) {
    return out;
  }
  out.push_back(pts[0]);
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    auto a = pts[i], b = pts[i + 1];
    int steps = static_cast<int>(std::ceil(std::hypot(b.first - a.first, b.second - a.second) / step));
    for (int k = 1; k < steps; ++k) {
      double t = static_cast<double>(k) / steps;
      out.push_back({a.first + (b.first - a.first) * t, a.second + (b.second - a.second) * t});
    }
    out.push_back(b);
  }
  return out;
}

static Pts line(double x0, double y0, double x1, double y1, double spacing = 0.2)
{
  int n = std::max(1, static_cast<int>(std::ceil(std::hypot(x1 - x0, y1 - y0) / spacing)));
  Pts out;
  for (int i = 0; i <= n; ++i) {
    double t = static_cast<double>(i) / n;
    out.push_back({x0 + (x1 - x0) * t, y0 + (y1 - y0) * t});
  }
  return out;
}

static Pts arc(double cx, double cy, double r, double a0, double a1, double spacing = 0.2)
{
  int n = std::max(2, static_cast<int>(std::ceil(std::fabs(a1 - a0) * r / spacing)));
  Pts out;
  for (int i = 0; i <= n; ++i) {
    double a = a0 + (a1 - a0) * i / n;
    out.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
  }
  return out;
}

static Pts join(std::initializer_list<Pts> parts)
{
  Pts out;
  for (auto & p : parts) {
    for (auto & q : p) {
      if (!out.empty() && std::hypot(out.back().first - q.first, out.back().second - q.second) < 1e-9) {
        continue;
      }
      out.push_back(q);
    }
  }
  return out;
}

static void set_yaw(geometry_msgs::msg::Quaternion & q, double yaw)
{
  tf2::Quaternion t;
  t.setRPY(0.0, 0.0, yaw);
  q.x = t.x();
  q.y = t.y();
  q.z = t.z();
  q.w = t.w();
}

// Poses face the next pose (the last one the previous heading).
static Path to_path(const Pts & pts)
{
  Path path;
  path.header.frame_id = "map";
  for (size_t i = 0; i < pts.size(); ++i) {
    PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = pts[i].first;
    ps.pose.position.y = pts[i].second;
    double yaw = 0.0;
    if (pts.size() > 1) {
      size_t a = i + 1 < pts.size() ? i : i - 1;
      yaw = std::atan2(pts[a + 1].second - pts[a].second, pts[a + 1].first - pts[a].first);
    }
    set_yaw(ps.pose.orientation, yaw);
    path.poses.push_back(ps);
  }
  return path;
}

static Path with_yaws(Path p, double yaw, size_t from = 0)
{
  for (size_t i = from; i < p.poses.size(); ++i) {
    set_yaw(p.poses[i].pose.orientation, yaw);
  }
  return p;
}

static json path_json(const Path & p)
{
  json poses = json::array();
  for (auto & ps : p.poses) {
    poses.push_back({ps.pose.position.x, ps.pose.position.y, ps.pose.orientation.z, ps.pose.orientation.w});
  }
  return poses;
}

static json costmap_json(nav2_costmap_2d::Costmap2D & c)
{
  json cells = json::array();
  const unsigned char * data = c.getCharMap();
  for (unsigned int i = 0; i < c.getSizeInCellsX() * c.getSizeInCellsY(); ++i) {
    if (data[i] != 0) {
      cells.push_back({i, static_cast<int>(data[i])});
    }
  }
  return {{"size_x", c.getSizeInCellsX()}, {"size_y", c.getSizeInCellsY()},
    {"resolution", c.getResolution()}, {"origin_x", c.getOriginX()},
    {"origin_y", c.getOriginY()}, {"cells", cells}};
}

// --- scenarios -------------------------------------------------------------------

struct Rect
{
  double x0, y0, x1, y1;  // map frame; cells whose centre is inside are lethal
};

struct Event
{
  int cycle;
  std::string type;  // "plan" | "speed_limit" | "jump"
  std::function<Path(double, double)> plan;  // from the robot position in map
  double a = 0.0, b = 0.0;
  bool percentage = false;
};

// Writes extra costs into the local costmap after each update (map <- odom).
using CostHook = std::function<void(nav2_costmap_2d::Costmap2D &, const Frame &)>;

struct Scenario
{
  std::string name;
  std::vector<Path> segments;  // FollowPath goals, the next one on goal reached
  double sx, sy, syaw;  // start, map frame
  std::vector<Rect> obstacles;
  std::vector<Event> events;
  int max_cycles;
  unsigned seed = 1;
  std::string config = "robot";
  CostHook cost_hook = nullptr;
};

static std::vector<Scenario> scenarios()
{
  std::vector<Scenario> s;
  auto row = [](double len) {return to_path(densify(line(0.0, 0.0, len, 0.0), 0.1));};

  s.push_back({"straight_row", {row(4.0)}, 0.0, 0.0, 0.0, {}, {}, 420, 11});
  s.push_back({"offset_start", {row(4.0)}, 0.0, 0.12, 0.35, {}, {}, 420, 12});
  s.push_back({"rotated_start_ccw", {row(2.5)}, 0.0, 0.0, -2.4, {}, {}, 400, 13});
  s.push_back({"rotated_start_cw", {row(2.5)}, 0.0, 0.02, 1.4, {}, {}, 400, 14});
  s.push_back({"goal_approach", {to_path(line(0.0, 0.0, 0.9, 0.0, 0.1))}, -0.05, 0.03, 0.1, {}, {}, 200, 15});
  s.push_back({"arc_tight",
      {to_path(densify(join({arc(0.0, 0.4, 0.4, -M_PI / 2, M_PI / 2), line(0.0, 0.8, -1.0, 0.8)}), 0.1))},
      0.0, 0.0, 0.0, {}, {}, 400, 16});
  s.push_back({"s_curve",
      {to_path(densify(join({arc(0.0, 1.0, 1.0, -M_PI / 2, -M_PI / 6),
          arc(std::sqrt(3.0), 0.0, 1.0, 5 * M_PI / 6, M_PI / 2),
          line(std::sqrt(3.0), 1.0, std::sqrt(3.0) + 1.0, 1.0)}), 0.1))},
      0.0, 0.0, 0.0, {}, {}, 400, 17});

  // Coverage: three 3 m lanes 0.8 m apart (strip_width_m) with the coverage
  // planner's U-turns (x linear, y = r sin(pi t), 8 samples), split at the
  // U-turn ends as mower_nav does, each segment densified to 0.1 m.
  {
    auto uturn = [](double x0, double x1, double cy, bool top) {
        Pts p;
        int n = 8;
        for (int i = 0; i < n; ++i) {
          double t = static_cast<double>(i) / (n - 1);
          double off = 0.4 * std::sin(M_PI * t);
          p.push_back({x0 + (x1 - x0) * t, top ? cy + off : cy - off});
        }
        return p;
      };
    Path s1 = to_path(densify(join({line(0.0, 0.0, 0.0, 2.6), uturn(0.0, 0.8, 2.6, true)}), 0.1));
    Path s2 = to_path(densify(join({line(0.8, 2.6, 0.8, 0.4), uturn(0.8, 1.6, 0.4, false)}), 0.1));
    Path s3 = to_path(densify(line(1.6, 0.4, 1.6, 3.0), 0.1));
    s.push_back({"zigzag_uturn", {s1, s2, s3}, 0.0, 0.0, M_PI / 2, {}, {}, 1400, 18});
  }
  // The same lanes joined by straight connectors: mower_nav splits at the
  // 90 deg corners (turn_split_angle_rad 0.8), so every segment starts with
  // a 90 deg turn for the shim.
  {
    Pts corners = {{0.0, 0.0}, {0.0, 3.0}, {0.8, 3.0}, {0.8, 0.0}, {1.6, 0.0}, {1.6, 3.0}};
    std::vector<Path> segs;
    for (size_t i = 0; i + 1 < corners.size(); ++i) {
      segs.push_back(to_path(densify(line(corners[i].first, corners[i].second,
        corners[i + 1].first, corners[i + 1].second), 0.1)));
    }
    s.push_back({"zigzag_corners", segs, 0.0, 0.0, M_PI / 2, {}, {}, 1700, 19});
    Pts all;
    for (size_t i = 0; i + 1 < corners.size(); ++i) {
      all = join({all, line(corners[i].first, corners[i].second, corners[i + 1].first, corners[i + 1].second)});
    }
    s.push_back({"zigzag_single", {to_path(densify(all, 0.1))}, 0.0, 0.0, M_PI / 2, {}, {}, 1500, 20});
  }

  s.push_back({"lethal_on_path", {row(4.0)}, 0.0, 0.0, 0.0, {{1.96, -0.06, 2.14, 0.06}}, {}, 500, 21});
  {
    Path up = to_path(densify(line(0.0, 0.0, 0.0, 2.0), 0.1));
    s.push_back({"rotate_blocked", {up}, 0.0, 0.0, 0.0, {{-1.0, -0.49, 1.0, -0.41}}, {}, 200, 22});
    s.push_back({"rotate_near_wall", {up}, 0.0, 0.0, 0.0, {{-1.0, -0.69, 1.0, -0.61}}, {}, 300, 23});
  }
  s.push_back({"far_from_path", {row(3.0)}, 0.5, -2.0, 0.0, {}, {}, 200, 24});
  s.push_back({"path_behind", {row(1.2)}, 1.6, 0.05, 0.0, {}, {}, 500, 25});
  s.push_back({"plan_behind_start", {to_path(densify(line(0.6, 0.0, -2.0, 0.0), 0.1))}, 0.5, 0.03, 0.0, {}, {},
      500, 26});
  {
    Path one = to_path({{0.8, 0.3}});
    s.push_back({"one_pose_plan", {one}, 0.0, 0.0, 0.0, {}, {}, 200, 27});
    s.push_back({"two_pose_plan", {to_path({{0.0, 0.0}, {1.0, 0.25}})}, 0.0, 0.0, 0.1, {}, {}, 250, 28});
    s.push_back({"sparse_plan", {to_path({{0.0, 0.0}, {2.5, 0.0}})}, 0.0, 0.0, 0.0, {}, {}, 120, 29});
    Path empty;
    empty.header.frame_id = "map";
    s.push_back({"empty_plan", {empty}, 0.0, 0.0, 0.0, {}, {}, 1, 30});
  }
  {
    Event turn{80, "plan", [](double x, double y) {
        return to_path(densify(join({line(x, y, x + 0.1, y + 0.05, 0.1), line(x + 0.1, y + 0.05, x + 0.1, y + 2.0)}),
          0.1));
      }};
    Event again{200, "plan", nullptr};  // the current plan once more
    s.push_back({"replan_midrun", {row(4.0)}, 0.0, 0.0, 0.0, {}, {turn, again}, 700, 31});
  }
  {
    Event half{40, "speed_limit", nullptr, 50.0, 0.0, true};
    Event abs{120, "speed_limit", nullptr, 0.1, 0.0, false};
    Event off{200, "speed_limit", nullptr, 0.0, 0.0, false};  // NO_SPEED_LIMIT
    s.push_back({"speed_limit", {row(5.0)}, 0.0, 0.0, 0.0, {}, {half, abs, off}, 600, 32});
  }
  {
    // odometry jumps 1.7 m between two 2 Hz costmap updates: the robot is
    // off the stale window
    Event jump{25, "jump", nullptr, 1.7, 0.0, false};
    s.push_back({"pose_jump", {row(4.0)}, 0.0, 0.0, 0.0, {}, {jump}, 60, 33});
  }

  // --- variant_a: RPP rotate-to-heading, velocity scaled lookahead; shim open
  // loop, rotate_to_heading_once
  {
    Scenario g{"a_goal_heading", {with_yaws(row(2.0), 1.5, 20)}, 0.0, 0.05, 0.2, {}, {}, 500, 41, "variant_a"};
    s.push_back(g);
    Path corner = to_path(densify(join({line(0.0, 0.0, 1.2, 0.0), line(1.2, 0.0, 0.5, 0.7)}), 0.1));  // 135 deg
    s.push_back({"a_rotate_to_path", {corner}, 0.0, 0.0, 0.0, {}, {}, 500, 42, "variant_a"});
    Event same{60, "plan", nullptr};
    Event other{120, "plan", [](double x, double y) {
        return to_path(densify(line(x, y, x - 0.3, y + 1.5), 0.1));
      }};
    s.push_back({"a_replan_same_goal", {row(3.0)}, 0.0, 0.0, 0.0, {}, {same, other}, 500, 43, "variant_a"});
    s.push_back({"a_start_rotated", {row(2.0)}, 0.0, 0.0, 2.0, {}, {}, 400, 44, "variant_a"});
  }
  // --- variant_b: reversing with cusps, fixed curvature lookahead interpolated
  // past the goal, cost regulation; shim rotate_to_goal_heading,
  // use_path_orientations
  {
    Path cusp = with_yaws(to_path(densify(join({line(0.0, 0.0, 1.2, 0.0), line(1.2, 0.0, 0.3, 0.3)}), 0.1)), 0.0);
    s.push_back({"b_reverse_cusp", {cusp}, 0.0, 0.0, 0.0, {}, {}, 500, 51, "variant_b"});
    s.push_back({"b_goal_heading_shim", {with_yaws(row(1.5), 1.2, 15)}, 0.0, 0.0, 0.0, {}, {}, 400, 52,
        "variant_b"});
    CostHook band = [](nav2_costmap_2d::Costmap2D & c, const Frame & m) {
        // an inflation-like band over x in [1, 2] (map), 240 on the row
        for (unsigned int j = 0; j < c.getSizeInCellsY(); ++j) {
          for (unsigned int i = 0; i < c.getSizeInCellsX(); ++i) {
            double wx, wy;
            c.mapToWorld(i, j, wx, wy);
            double mx = m[0] + std::cos(m[2]) * wx - std::sin(m[2]) * wy;
            double my = m[1] + std::sin(m[2]) * wx + std::cos(m[2]) * wy;
            if (mx >= 1.0 && mx <= 2.0 && std::fabs(my) < 0.35 && c.getCost(i, j) == 0) {
              c.setCost(i, j, static_cast<unsigned char>(240 - static_cast<int>(200 * std::fabs(my))));
            }
          }
        }
      };
    Scenario cost{"b_cost_regulation", {row(3.0)}, 0.0, 0.0, 0.0, {}, {}, 400, 53, "variant_b", band};
    s.push_back(cost);
    s.push_back({"b_start_rotated", {row(2.0)}, 0.0, 0.0, -1.5, {}, {}, 400, 54, "variant_b"});
    // interpolate_curvature_after_goal makes transformGlobalPlan reject it
    s.push_back({"b_one_pose_plan", {to_path({{0.8, 0.3}})}, 0.0, 0.0, 0.0, {}, {}, 50, 55, "variant_b"});
    // an in-place rotation in the plan: two poses at (0.8, 0), yaw 0 then 0.6
    Path turn = to_path(densify(line(0.0, 0.0, 0.8, 0.0), 0.1));
    turn.poses.push_back(turn.poses.back());
    set_yaw(turn.poses.back().pose.orientation, 0.6);
    Path rest = to_path(densify(line(0.8, 0.0, 2.0, 0.0), 0.1));
    turn.poses.insert(turn.poses.end(), rest.poses.begin() + 1, rest.poses.end());
    s.push_back({"b_in_place_pose", {turn}, 0.0, 0.0, 0.0, {}, {}, 300, 56, "variant_b"});
  }
  return s;
}

// --- controller configurations ----------------------------------------------------

using Params = std::vector<rclcpp::Parameter>;

static Params common_params()
{
  return {
    {"controller_frequency", 20.0},
    {"FollowPath.primary_controller",
      std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")},
  };
}

static Params variant_a()
{
  auto p = common_params();
  Params more = {
    {"FollowPath.angular_dist_threshold", 0.6}, {"FollowPath.angular_disengage_threshold", 0.35},
    {"FollowPath.forward_sampling_distance", 0.25}, {"FollowPath.rotate_to_heading_angular_vel", 0.8},
    {"FollowPath.max_angular_accel", 1.6}, {"FollowPath.closed_loop", false},
    {"FollowPath.rotate_to_heading_once", true},
    {"FollowPath.desired_linear_vel", 0.3}, {"FollowPath.lookahead_dist", 0.3},
    {"FollowPath.min_lookahead_dist", 0.2}, {"FollowPath.max_lookahead_dist", 0.4},
    {"FollowPath.lookahead_time", 1.2}, {"FollowPath.use_velocity_scaled_lookahead_dist", true},
    {"FollowPath.use_rotate_to_heading", true}, {"FollowPath.rotate_to_heading_min_angle", 0.785},
    {"FollowPath.use_cost_regulated_linear_velocity_scaling", false},
    {"FollowPath.approach_velocity_scaling_dist", 0.45}, {"FollowPath.min_approach_linear_velocity", 0.05},
    {"FollowPath.regulated_linear_scaling_min_radius", 0.9},
    {"FollowPath.regulated_linear_scaling_min_speed", 0.1},
    {"FollowPath.max_robot_pose_search_dist", -1.0}, {"FollowPath.stateful", true},
    {"variant_goal_checker.xy_goal_tolerance", 0.1}, {"variant_goal_checker.yaw_goal_tolerance", 0.15},
    {"variant_goal_checker.stateful", true},
  };
  p.insert(p.end(), more.begin(), more.end());
  return p;
}

static Params variant_b()
{
  auto p = common_params();
  Params more = {
    {"FollowPath.angular_dist_threshold", 0.7}, {"FollowPath.angular_disengage_threshold", 0.3},
    {"FollowPath.forward_sampling_distance", 0.3}, {"FollowPath.rotate_to_heading_angular_vel", 0.7},
    {"FollowPath.max_angular_accel", 2.0}, {"FollowPath.simulate_ahead_time", 0.6},
    {"FollowPath.max_cost_threshold", 253.0}, {"FollowPath.rotate_to_goal_heading", true},
    {"FollowPath.use_path_orientations", true},
    {"FollowPath.desired_linear_vel", 0.25}, {"FollowPath.lookahead_dist", 0.3},
    {"FollowPath.use_rotate_to_heading", false}, {"FollowPath.allow_reversing", true},
    {"FollowPath.use_fixed_curvature_lookahead", true}, {"FollowPath.curvature_lookahead_dist", 0.45},
    {"FollowPath.interpolate_curvature_after_goal", true},
    {"FollowPath.use_cost_regulated_linear_velocity_scaling", true}, {"FollowPath.cost_scaling_dist", 0.6},
    {"FollowPath.cost_scaling_gain", 1.0}, {"FollowPath.inflation_cost_scaling_factor", 3.0},
    {"FollowPath.approach_velocity_scaling_dist", 0.37}, {"FollowPath.min_approach_linear_velocity", 0.04},
    {"FollowPath.regulated_linear_scaling_min_radius", 0.7},
    {"FollowPath.regulated_linear_scaling_min_speed", 0.06},
    {"FollowPath.max_allowed_time_to_collision_up_to_carrot", 1.5}, {"FollowPath.stateful", false},
    {"variant_goal_checker.xy_goal_tolerance", 0.1}, {"variant_goal_checker.yaw_goal_tolerance", 0.15},
    {"variant_goal_checker.stateful", true},
  };
  p.insert(p.end(), more.begin(), more.end());
  return p;
}

// --- harness ----------------------------------------------------------------------

struct Ctl
{
  nav2_util::LifecycleNode::SharedPtr node;
  std::string goal_checker_name;
  std::shared_ptr<nav2_controller::SimpleGoalChecker> goal_checker;
};

struct Env
{
  std::map<std::string, Ctl> ctls;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;
  std::shared_ptr<tf2_ros::Buffer> tf;
  nav2_costmap_2d::StaticLayer * static_layer = nullptr;
  Frame map_from_odom{1.3, -0.6, 0.4};
  int64_t tick = 0;  // the 20 Hz clock, never reset between scenarios
};

static rclcpp::Time now_of(const Env & env)
{
  return rclcpp::Time(static_cast<int64_t>(1000000000000LL + env.tick * 50000000LL), RCL_ROS_TIME);
}

static geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent, const std::string & child, double x, double y, double yaw, const rclcpp::Time & t)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.header.stamp = t;
  ts.child_frame_id = child;
  ts.transform.translation.x = x;
  ts.transform.translation.y = y;
  set_yaw(ts.transform.rotation, yaw);
  return ts;
}

static nav_msgs::msg::OccupancyGrid::SharedPtr world_map(const std::vector<Rect> & obstacles)
{
  auto g = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  g->header.frame_id = "map";
  g->info.resolution = 0.1;  // the mission map's cells
  g->info.width = 160;
  g->info.height = 140;
  g->info.origin.position.x = -6.0;
  g->info.origin.position.y = -6.0;
  g->info.origin.orientation.w = 1.0;
  g->data.assign(g->info.width * g->info.height, 0);
  for (unsigned int j = 0; j < g->info.height; ++j) {
    for (unsigned int i = 0; i < g->info.width; ++i) {
      double cx = -6.0 + (i + 0.5) * 0.1, cy = -6.0 + (j + 0.5) * 0.1;
      for (auto & r : obstacles) {
        if (cx >= r.x0 && cx <= r.x1 && cy >= r.y0 && cy <= r.y1) {
          g->data[j * g->info.width + i] = 100;
        }
      }
    }
  }
  return g;
}

static std::pair<std::string, std::string> run_cycle(
  OracleShim & shim, Ctl & ctl, const PoseStamped & pose, const geometry_msgs::msg::Twist & vel,
  geometry_msgs::msg::TwistStamped & cmd)
{
  try {
    cmd = shim.computeVelocityCommands(pose, vel, ctl.goal_checker.get());
    return {"", ""};
  } catch (nav2_core::InvalidController & e) {
    return {"InvalidController", e.what()};
  } catch (nav2_core::ControllerTFError & e) {
    return {"ControllerTFError", e.what()};
  } catch (nav2_core::FailedToMakeProgress & e) {
    return {"FailedToMakeProgress", e.what()};
  } catch (nav2_core::PatienceExceeded & e) {
    return {"PatienceExceeded", e.what()};
  } catch (nav2_core::InvalidPath & e) {
    return {"InvalidPath", e.what()};
  } catch (nav2_core::NoValidControl & e) {
    return {"NoValidControl", e.what()};
  } catch (nav2_core::ControllerTimedOut & e) {
    return {"ControllerTimedOut", e.what()};
  } catch (nav2_core::ControllerException & e) {
    return {"ControllerException", e.what()};
  } catch (std::exception & e) {
    return {"std::exception", e.what()};
  }
}

static json run(const Scenario & sc, Env & env)
{
  Ctl & ctl = env.ctls.at(sc.config);
  auto shim = std::make_shared<OracleShim>();
  shim->configure(ctl.node, "FollowPath", env.tf, env.costmap_ros);
  shim->activate();
  StaticLayerAccess::feed(env.static_layer, world_map(sc.obstacles));

  std::mt19937 rng(sc.seed);
  std::uniform_real_distribution<double> noise(-1.0, 1.0);
  const auto & m = env.map_from_odom;
  const double cm = std::cos(m[2]), sm = std::sin(m[2]);
  // robot state in odom
  double dx = sc.sx - m[0], dy = sc.sy - m[1];
  double x = cm * dx + sm * dy, y = -sm * dx + cm * dy, yaw = sc.syaw - m[2];
  double v = 0.0, w = 0.0;  // odometry twist
  const double dt = 0.05;

  json steps = json::array();
  size_t segment = 0;
  Path current = sc.segments[0];
  rclcpp::Time last_valid_cmd_time = now_of(env);
  auto set_plan = [&](const Path & p) {
      current = p;
      shim->setPlan(p);
      ctl.goal_checker->reset();
      last_valid_cmd_time = now_of(env);
      steps.push_back({{"plan", path_json(p)}});
    };
  auto to_map = [&](double ox, double oy) {
      return std::make_pair(m[0] + cm * ox - sm * oy, m[1] + sm * ox + cm * oy);
    };
  set_plan(current);

  std::string end = "max_cycles";
  int cycles = 0;
  for (int k = 0; k < sc.max_cycles; ++k, ++env.tick) {
    const rclcpp::Time t = now_of(env);
    rcl_set_ros_time_override(ctl.node->get_clock()->get_clock_handle(), t.nanoseconds());

    for (auto & e : sc.events) {
      if (e.cycle != k) {
        continue;
      }
      if (e.type == "plan") {
        auto [mx, my] = to_map(x, y);
        set_plan(e.plan ? e.plan(mx, my) : current);
      } else if (e.type == "speed_limit") {
        shim->setSpeedLimit(e.a, e.percentage);
        steps.push_back({{"speed_limit", {e.a, e.percentage}}});
      } else if (e.type == "jump") {
        x += e.a;
        y += e.b;
      }
    }

    // the 2 Hz local costmap update, centred on the robot
    if (k % 10 == 0) {
      env.costmap_ros->getLayeredCostmap()->updateMap(x, y, yaw);
      if (sc.cost_hook) {
        sc.cost_hook(*env.costmap_ros->getCostmap(), m);
      }
      steps.push_back({{"costmap", costmap_json(*env.costmap_ros->getCostmap())}});
    }
    env.tf->setTransform(make_tf("odom", "base_footprint", x, y, yaw, t), "oracle", false);

    PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.header.stamp = t;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    set_yaw(pose.pose.orientation, yaw);
    // ControllerServer::getThresholdedTwist (min_x 0.001, min_y 0.5, min_theta 0.001)
    geometry_msgs::msg::Twist vel;
    vel.linear.x = std::fabs(v) > 0.001 ? v : 0.0;
    vel.angular.z = std::fabs(w) > 0.001 ? w : 0.0;

    double cmd_v = 0.0, cmd_w = 0.0;
    bool stop = false;
    // ControllerServer::isGoalReached before computing, on the path's end
    // pose in the costmap frame
    if (!current.poses.empty()) {
      PoseStamped end_odom;
      auto & ep = current.poses.back().pose;
      double ex = ep.position.x - m[0], ey = ep.position.y - m[1];
      end_odom.pose.position.x = cm * ex + sm * ey;
      end_odom.pose.position.y = -sm * ex + cm * ey;
      set_yaw(end_odom.pose.orientation, tf2::getYaw(ep.orientation) - m[2]);
      if (ctl.goal_checker->isGoalReached(pose.pose, end_odom.pose, vel)) {
        steps.push_back({{"goal", segment}});
        shim->reset();  // ControllerServer::onGoalExit
        steps.push_back({{"reset", true}});
        if (++segment < sc.segments.size()) {
          set_plan(sc.segments[segment]);
        } else {
          end = "goal_reached";
          stop = true;
        }
      }
    }

    if (!stop) {
      geometry_msgs::msg::TwistStamped cmd;
      auto [kind, what] = run_cycle(*shim, ctl, pose, vel, cmd);
      json c = {{"pose", {x, y, pose.pose.orientation.z, pose.pose.orientation.w}},
        {"speed", {vel.linear.x, vel.angular.z}},
        {"rot", shim->inRotation()}, {"upd", shim->pathUpdated()},
        {"plan", RppAccess::planSize(shim->primary())}};
      ++cycles;
      if (kind.empty()) {
        c["out"] = {cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z};
        cmd_v = cmd.twist.linear.x;
        cmd_w = cmd.twist.angular.z;
        last_valid_cmd_time = t;
        steps.push_back({{"cycle", c}});
      } else {
        c["err"] = {kind, what};
        steps.push_back({{"cycle", c}});
        if (kind == "NoValidControl") {
          // failure_tolerance 2.0: zero velocity until patience runs out
          if ((t - last_valid_cmd_time).seconds() > 2.0) {
            end = "PatienceExceeded";
            stop = true;
          }
        } else {
          end = kind;
          stop = true;
        }
        if (stop) {
          shim->reset();
          steps.push_back({{"reset", true}});
        }
      }
    }
    if (stop) {
      ++env.tick;
      break;
    }

    // velocity smoother limits (0.26 m/s, 1.0 rad/s, 2.5 m/s^2, 3.2 rad/s^2),
    // then the base with a little slip
    cmd_v = std::clamp(cmd_v, -0.26, 0.26);
    cmd_w = std::clamp(cmd_w, -1.0, 1.0);
    v += std::clamp(cmd_v - v, -2.5 * dt, 2.5 * dt);
    w += std::clamp(cmd_w - w, -3.2 * dt, 3.2 * dt);
    double va = v * (1.0 + 0.03 * noise(rng));
    double wa = w * (1.0 + 0.05 * noise(rng)) + 0.004 * noise(rng);
    double mid = yaw + 0.5 * wa * dt;
    x += va * std::cos(mid) * dt;
    y += va * std::sin(mid) * dt;
    yaw += wa * dt;
    v = va;
    w = wa;
  }

  shim->deactivate();
  shim->cleanup();
  std::printf("%-20s %-9s %5d cycles, %zu segment(s): %s\n", sc.name.c_str(), sc.config.c_str(), cycles,
    sc.segments.size(), end.c_str());
  return {{"name", sc.name}, {"config", sc.config}, {"end", end}, {"steps", steps}};
}

static json param_value(const rclcpp::Parameter & p)
{
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return p.as_bool();
    case rclcpp::ParameterType::PARAMETER_INTEGER: return p.as_int();
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return p.as_double();
    case rclcpp::ParameterType::PARAMETER_STRING: return p.as_string();
    default: return nullptr;
  }
}

static Ctl make_ctl(
  const std::string & node_name, const std::string & goal_checker, const Params & overrides,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  Ctl c;
  c.node = std::make_shared<nav2_util::LifecycleNode>(
    node_name, "", rclcpp::NodeOptions().parameter_overrides(overrides));
  rcl_enable_ros_time_override(c.node->get_clock()->get_clock_handle());
  // as the controller server declares it
  c.node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  c.goal_checker_name = goal_checker;
  c.goal_checker = std::make_shared<nav2_controller::SimpleGoalChecker>();
  c.goal_checker->initialize(c.node, goal_checker, costmap_ros);
  return c;
}

int main(int argc, char ** argv)
{
  // The parameter file goes in as a global argument, so the controller server
  // node and the local costmap node read their sections from it (the variant
  // nodes have no section and use their overrides only).
  std::vector<const char *> args = {argv[0], "--ros-args", "--params-file", NAV2_PARAMS_YAML,
    "--log-level", "error"};
  rclcpp::init(static_cast<int>(args.size()), args.data());

  Env env;
  env.costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", "/", "local_costmap", false);
  env.costmap_ros->configure();
  env.tf = env.costmap_ros->getTfBuffer();
  env.tf->setTransform(
    make_tf("map", "odom", env.map_from_odom[0], env.map_from_odom[1], env.map_from_odom[2], rclcpp::Time(0, 0)),
    "oracle", true);
  auto plugins = env.costmap_ros->getLayeredCostmap()->getPlugins();
  env.static_layer = dynamic_cast<nav2_costmap_2d::StaticLayer *>(plugins->at(0).get());

  env.ctls["robot"] = make_ctl("controller_server", "general_goal_checker", {}, env.costmap_ros);
  env.ctls["variant_a"] = make_ctl("controller_variant_a", "variant_goal_checker", variant_a(), env.costmap_ros);
  env.ctls["variant_b"] = make_ctl("controller_variant_b", "variant_goal_checker", variant_b(), env.costmap_ros);

  json out;
  out["map_from_odom"] = env.map_from_odom;
  {
    auto * c = env.costmap_ros->getLayeredCostmap();
    json fp = json::array();
    for (auto & p : env.costmap_ros->getRobotFootprint()) {
      fp.push_back({p.x, p.y});
    }
    out["costmap"] = {{"global_frame", env.costmap_ros->getGlobalFrameID()},
      {"base_frame", env.costmap_ros->getBaseFrameID()},
      {"size_x", c->getCostmap()->getSizeInCellsX()}, {"resolution", c->getCostmap()->getResolution()},
      {"inscribed", c->getInscribedRadius()}, {"circumscribed", c->getCircumscribedRadius()},
      {"tracking_unknown", c->isTrackingUnknown()}, {"footprint", fp}};
  }

  json runs = json::array();
  for (auto & sc : scenarios()) {
    runs.push_back(run(sc, env));
  }
  out["scenarios"] = runs;

  // What the plugins actually read (declared during configure), per node.
  json configs = json::object();
  for (auto & [name, ctl] : env.ctls) {
    json params = json::object();
    for (auto & p : ctl.node->list_parameters({"FollowPath", ctl.goal_checker_name}, 2).names) {
      params[p] = param_value(ctl.node->get_parameter(p));
    }
    for (auto & p : {".position_checker.xy_goal_tolerance", ".position_checker.stateful"}) {
      if (ctl.node->has_parameter(p)) {
        params[p] = param_value(ctl.node->get_parameter(p));
      }
    }
    params["controller_frequency"] = ctl.node->get_parameter("controller_frequency").as_double();
    geometry_msgs::msg::Pose pose_tol;
    geometry_msgs::msg::Twist vel_tol;
    ctl.goal_checker->getTolerances(pose_tol, vel_tol);
    configs[name] = {{"params", params}, {"xy_goal_tolerance", pose_tol.position.x}};
  }
  out["configs"] = configs;

  std::ofstream(argv[argc - 1]) << out.dump() << "\n";
  std::fflush(stdout);
  // Skip the node/thread teardown; everything is written.
  std::_Exit(0);
}
