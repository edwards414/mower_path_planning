// Oracle for mower_nav_core::{navfn, planner}: drives the installed
// nav2_navfn_planner (Jazzy 1.3.13) — the NavFn class directly, and the
// NavfnPlanner plugin configured the way planner_server configures it, on a
// global costmap built by the real LayeredCostmap + StaticLayer — on
// deterministic synthetic grids, and dumps inputs and results as JSON.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

using nlohmann::json;
using nav2_navfn_planner::NavFn;
using Pt = std::array<double, 2>;

// ---- helpers ------------------------------------------------------------------

// FNV-1a over the float bits: equal hashes mean bit-equal potentials.
static uint64_t pot_hash(const float * p, int n)
{
  uint64_t h = 14695981039346656037ull;
  for (int i = 0; i < n; ++i) {
    uint32_t b;
    std::memcpy(&b, &p[i], 4);
    h ^= b;
    h *= 1099511628211ull;
  }
  return h;
}

static json pot_summary(const NavFn & nav)
{
  int finite = 0;
  for (int i = 0; i < nav.ns; ++i) {
    if (nav.potarr[i] < POT_HIGH) {
      ++finite;
    }
  }
  return {{"hash", pot_hash(nav.potarr, nav.ns)}, {"finite", finite}};
}

static json floats(const float * p, int n)
{
  json a = json::array();
  for (int i = 0; i < n; ++i) {
    a.push_back(static_cast<double>(p[i]));
  }
  return a;
}

static json bytes(const unsigned char * p, int n)
{
  json a = json::array();
  for (int i = 0; i < n; ++i) {
    a.push_back(static_cast<int>(p[i]));
  }
  return a;
}

static bool in_polygon(const std::vector<Pt> & poly, double x, double y)
{
  bool in = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (((poly[i][1] > y) != (poly[j][1] > y)) &&
      (x < (poly[j][0] - poly[i][0]) * (y - poly[i][1]) / (poly[j][1] - poly[i][1]) + poly[i][0]))
    {
      in = !in;
    }
  }
  return in;
}

// An OccupancyGrid under construction, addressed by cell or by cell centre.
struct Grid
{
  std::string name;
  int w, h;
  double res, ox, oy;
  std::vector<int8_t> data;

  Grid(std::string n, int w_, int h_, double r, double x, double y, int8_t fill)
  : name(std::move(n)), w(w_), h(h_), res(r), ox(x), oy(y), data(w_ * h_, fill) {}

  double cx(int i) const {return ox + (i + 0.5) * res;}
  double cy(int j) const {return oy + (j + 0.5) * res;}
  int ci(double x) const {return static_cast<int>(std::floor((x - ox) / res));}
  int cj(double y) const {return static_cast<int>(std::floor((y - oy) / res));}
  // the point at fraction (fx, fy) of cell (i, j): the costmap truncates it to
  // (i, j); NavfnPlanner rounds a fraction below 0.5 to i, above to i + 1
  Pt at(int i, int j, double fx, double fy) const {return {ox + (i + fx) * res, oy + (j + fy) * res};}
  int8_t get(int i, int j) const {return data[j * w + i];}
  void set(int i, int j, int8_t v)
  {
    if (i >= 0 && j >= 0 && i < w && j < h) {
      data[j * w + i] = v;
    }
  }
  template<class F>
  void fill_if(F f, int8_t v)
  {
    for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
        if (f(cx(i), cy(j))) {
          data[j * w + i] = v;
        }
      }
    }
  }
  void disc(double x, double y, double r, int8_t v)
  {
    fill_if([&](double px, double py) {return std::hypot(px - x, py - y) <= r;}, v);
  }
  void rect(double x0, double x1, double y0, double y1, int8_t v)
  {
    fill_if([&](double px, double py) {return px >= x0 && px <= x1 && py >= y0 && py <= y1;}, v);
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr msg() const
  {
    auto g = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    g->header.frame_id = "map";
    g->info.width = w;
    g->info.height = h;
    g->info.resolution = res;
    g->info.origin.position.x = ox;
    g->info.origin.position.y = oy;
    g->info.origin.orientation.w = 1.0;
    g->data = data;
    return g;
  }
  // as the message carries it: MapMetaData.resolution is a float32
  json to_json() const
  {
    auto m = msg();
    json d = json::array();
    for (auto v : m->data) {
      d.push_back(static_cast<int>(v));
    }
    return {{"width", m->info.width}, {"height", m->info.height},
      {"resolution", m->info.resolution}, {"origin_x", m->info.origin.position.x},
      {"origin_y", m->info.origin.position.y}, {"data", d}};
  }
};

// ---- NavfnPlanner on a global costmap ------------------------------------------

struct OracleStaticLayer : public nav2_costmap_2d::StaticLayer
{
  using nav2_costmap_2d::StaticLayer::incomingMap;
};

// A Costmap2DROS that is never configured: NavfnPlanner only calls getCostmap()
// and getGlobalFrameID(), so it gets the master grid of a LayeredCostmap filled
// by a real StaticLayer, as the global costmap's is (frame map, not rolling,
// track_unknown_space true).
struct OracleCostmapROS : public nav2_costmap_2d::Costmap2DROS
{
  explicit OracleCostmapROS(const std::string & name)
  : Costmap2DROS(name) {}
  void adopt(std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered)
  {
    layered_costmap_ = std::move(layered);
    global_frame_ = "map";
  }
};

struct OraclePlanner : public nav2_navfn_planner::NavfnPlanner
{
  NavFn * navfn() {return planner_.get();}
};

struct Params
{
  std::string name;
  double tolerance;
  bool use_astar, allow_unknown, use_final_approach_orientation;
  json to_json() const
  {
    return {{"name", name}, {"tolerance", tolerance}, {"use_astar", use_astar},
      {"allow_unknown", allow_unknown},
      {"use_final_approach_orientation", use_final_approach_orientation}};
  }
};

static const Params kRobot{"robot", 0.0, false, true, false};
static const Params kDefaults{"upstream_defaults", 0.5, false, true, false};
static const Params kAstar{"astar", 0.0, true, true, false};
static const Params kFinalApproach{"final_approach", 0.0, false, true, true};
static const Params kNoUnknown{"no_unknown", 0.0, false, false, false};

struct Scenario
{
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<OracleCostmapROS> costmap_ros;
  nav2_util::LifecycleNode::SharedPtr node;
  std::shared_ptr<OraclePlanner> planner;
  std::vector<unsigned char> initial;
  nav2_costmap_2d::Costmap2D * costmap() {return costmap_ros->getCostmap();}
};

static std::unique_ptr<Scenario> make_scenario(const std::string & id, const Grid & grid, const Params & p)
{
  auto s = std::make_unique<Scenario>();
  s->costmap_ros = std::make_shared<OracleCostmapROS>(id + "_global_costmap");
  s->costmap_ros->set_parameter(rclcpp::Parameter("track_unknown_space", true));
  s->tf = std::make_unique<tf2_ros::Buffer>(s->costmap_ros->get_clock());
  auto layered = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, true);
  auto layer = std::make_shared<OracleStaticLayer>();
  layer->initialize(layered.get(), "static_layer", s->tf.get(), s->costmap_ros, nullptr);
  layered->addPlugin(layer);
  layer->incomingMap(grid.msg());
  layered->updateMap(0.0, 0.0, 0.0);
  s->costmap_ros->adopt(std::move(layered));

  // planner_server: the plugin's parameters live on the server node under its id
  s->node = std::make_shared<nav2_util::LifecycleNode>(id + "_planner_server");
  s->node->declare_parameter("GridBased.tolerance", rclcpp::ParameterValue(p.tolerance));
  s->node->declare_parameter("GridBased.use_astar", rclcpp::ParameterValue(p.use_astar));
  s->node->declare_parameter("GridBased.allow_unknown", rclcpp::ParameterValue(p.allow_unknown));
  s->node->declare_parameter(
    "GridBased.use_final_approach_orientation",
    rclcpp::ParameterValue(p.use_final_approach_orientation));
  s->planner = std::make_shared<OraclePlanner>();
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent = s->node;
  s->planner->configure(parent, "GridBased", nullptr, s->costmap_ros);
  s->planner->activate();

  auto * c = s->costmap();
  s->initial.assign(c->getCharMap(), c->getCharMap() + c->getSizeInCellsX() * c->getSizeInCellsY());
  return s;
}

struct Req
{
  std::string name;
  Pt start;
  double start_yaw;
  Pt goal;
  double goal_yaw;
  int cancel_at = 0;  // the cancel checker returns true from this call on (0: never)
};

static geometry_msgs::msg::PoseStamped pose(const Pt & p, double yaw)
{
  geometry_msgs::msg::PoseStamped s;
  s.header.frame_id = "map";
  s.pose.position.x = p[0];
  s.pose.position.y = p[1];
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  s.pose.orientation.x = q.x();
  s.pose.orientation.y = q.y();
  s.pose.orientation.z = q.z();
  s.pose.orientation.w = q.w();
  return s;
}

static json run_request(Scenario & s, const Req & r)
{
  int calls = 0;
  std::function<bool()> cancel = [&]() {
      ++calls;
      return r.cancel_at > 0 && calls >= r.cancel_at;
    };
  std::string error;
  nav_msgs::msg::Path path;
  try {
    path = s.planner->createPlan(pose(r.start, r.start_yaw), pose(r.goal, r.goal_yaw), cancel);
  } catch (const nav2_core::StartOutsideMapBounds &) {
    error = "StartOutsideMapBounds";
  } catch (const nav2_core::GoalOutsideMapBounds &) {
    error = "GoalOutsideMapBounds";
  } catch (const nav2_core::GoalOccupied &) {
    error = "GoalOccupied";
  } catch (const nav2_core::NoValidPathCouldBeFound &) {
    error = "NoValidPathCouldBeFound";
  } catch (const nav2_core::PlannerCancelled &) {
    error = "PlannerCancelled";
  } catch (const std::exception & e) {
    error = std::string("Unknown: ") + e.what();
  }

  json poses = json::array();
  for (auto & p : path.poses) {
    auto & o = p.pose.orientation;
    poses.push_back(
      {p.pose.position.x, p.pose.position.y, tf2::getYaw(tf2::Quaternion(o.x, o.y, o.z, o.w))});
  }
  json out = {{"name", r.name}, {"start", {r.start[0], r.start[1], r.start_yaw}},
    {"goal", {r.goal[0], r.goal[1], r.goal_yaw}}, {"cancel_at", r.cancel_at},
    {"cancel_calls", calls}, {"error", error.empty() ? json(nullptr) : json(error)},
    {"path", poses}};

  // makePlan ran (and left a potential) unless createPlan returned before it
  bool ran = (error.empty() || error == "NoValidPathCouldBeFound" || error == "PlannerCancelled") &&
    !(r.start[0] == r.goal[0] && r.start[1] == r.goal[1]);
  if (ran) {
    out["potential"] = pot_summary(*s.planner->navfn());
  }

  // master-grid cells clearRobotCell has changed so far, as [index, cost]
  json changed = json::array();
  const unsigned char * cells = s.costmap()->getCharMap();
  for (size_t i = 0; i < s.initial.size(); ++i) {
    if (cells[i] != s.initial[i]) {
      changed.push_back({i, static_cast<int>(cells[i])});
    }
  }
  out["changed_cells"] = changed;
  return out;
}

static json run_scenario(const std::string & id, const Grid & grid, const Params & p, const std::vector<Req> & reqs)
{
  auto s = make_scenario(id, grid, p);
  json requests = json::array();
  for (auto & r : reqs) {
    requests.push_back(run_request(*s, r));
  }
  return {{"id", id}, {"grid", grid.name}, {"params", p.to_json()}, {"requests", requests}};
}

// ---- the grids -------------------------------------------------------------------

// A mowing zone: a concave free polygon, lethal outside, a tree, a flowerbed, a
// wall across the zone passable through a 1-cell and a 3-cell gap, an island
// walled off by a 1-cell ring, unknown patches inside and across the boundary,
// and single lethal / unknown cells sprinkled in.
static Grid zone_grid()
{
  Grid g("zone", 220, 170, 0.05, -3.215, -1.937, 100);
  const std::vector<Pt> poly{{-2.85, -1.55}, {6.95, -1.62}, {7.30, 2.95}, {4.15, 3.20}, {3.55, 5.95},
    {-2.55, 6.15}};
  g.fill_if([&](double x, double y) {return in_polygon(poly, x, y);}, 0);
  g.disc(1.23, 1.41, 0.45, 100);
  g.rect(3.52, 5.21, -0.63, 0.42, 100);
  g.fill_if([](double x, double) {return x >= -0.62 && x <= -0.52;}, 100);
  g.fill_if([](double x, double y) {return x >= -0.62 && x <= -0.52 && y >= -0.03 && y <= 0.02;}, 0);
  g.fill_if([](double x, double y) {return x >= -0.62 && x <= -0.52 && y >= 2.98 && y <= 3.13;}, 0);
  int i0 = g.ci(1.80), i1 = g.ci(2.65), j0 = g.cj(3.60), j1 = g.cj(4.45);
  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      g.set(i, j, (i == i0 || i == i1 || j == j0 || j == j1) ? 100 : 0);
    }
  }
  g.rect(5.02, 6.21, 1.03, 2.31, -1);
  g.disc(-2.6, 3.0, 0.5, -1);

  const std::vector<Pt> keep{{-2.0, -1.0}, {-1.5, 0.0}, {0.5, 0.0}, {-1.8, 4.0}, {1.0, 5.0}, {3.0, 1.5},
    {5.6, 1.6}, {5.4, 1.4}, {2.0, -1.0}, {5.6, 2.6}, {0.7, 2.2}, {0.8, 2.4}, {6.5, -1.2}, {-2.2, 5.8},
    {2.2, 3.0}, {2.0, -1.5}};
  std::mt19937 rng(20261025);
  std::uniform_int_distribution<int> ui(0, g.w - 1), uj(0, g.h - 1);
  int lethal = 0, unknown = 0;
  while (lethal < 40 || unknown < 15) {
    int i = ui(rng), j = uj(rng);
    double x = g.cx(i), y = g.cy(j);
    if (!in_polygon(poly, x, y) || g.get(i, j) != 0) {
      continue;
    }
    bool near = false;
    for (auto & k : keep) {
      near = near || std::hypot(x - k[0], y - k[1]) < 0.35;
    }
    if (near) {
      continue;
    }
    if (lethal < 40) {
      g.set(i, j, 100);
      ++lethal;
    } else {
      g.set(i, j, -1);
      ++unknown;
    }
  }
  return g;
}

// Free up to the map edge, an unknown band, two obstacles.
static Grid edge_grid()
{
  Grid g("edge", 60, 40, 0.1, 1.05, -2.35, 0);
  for (int j = 25; j < 40; ++j) {
    for (int i = 3; i <= 6; ++i) {
      g.set(i, j, -1);
    }
  }
  for (int j = 5; j <= 30; ++j) {
    for (int i = 25; i <= 28; ++i) {
      g.set(i, j, 100);
    }
  }
  g.disc(g.cx(45), g.cy(20), 0.4, 100);
  // what clearRobotCell frees for a start rounded past the right edge in row 15
  g.set(0, 16, 100);
  return g;
}

// A zone in unknown space, split by a lethal wall: the halves connect only
// through unknown cells.
static Grid unknown_grid()
{
  Grid g("unknown", 150, 120, 0.05, 0.4, 0.3, -1);
  g.rect(1.0, 7.0, 1.0, 5.5, 0);
  g.rect(3.98, 4.08, 1.0, 5.5, 100);
  return g;
}

// A serpentine: walls every 4 rows with a 2-cell gap alternately right and
// left; the corridor path is far longer than the propagation's cycle limit.
static Grid maze_grid()
{
  Grid g("maze", 60, 60, 0.05, -1.5, -1.5, 0);
  for (int j = 4; j < 60; j += 4) {
    for (int i = 0; i < 60; ++i) {
      bool gap = ((j / 4) % 2 == 1) ? (i == 56 || i == 57) : (i == 2 || i == 3);
      if (!gap) {
        g.set(i, j, 100);
      }
    }
  }
  return g;
}

// ---- NavFn directly -------------------------------------------------------------

struct NavfnCase
{
  std::string name;
  int nx, ny;
  std::vector<uint8_t> cmap;
  bool open = false;  // cmap is all FREE_SPACE (not dumped)
  bool allow_unknown = true;
  std::array<int, 2> goal, start;
  bool astar = false;
  bool at_start = true;
  bool dump_pot = false;
  // more calcPath calls on the same potential: cell x, y and a step limit
  // (0: four times the larger side, as getPlanFromPotential)
  std::vector<std::array<int, 3>> extra_starts;
};

static json run_navfn(const NavfnCase & k)
{
  NavFn nav(k.nx, k.ny);
  nav.setCostmap(k.cmap.data(), true, k.allow_unknown);
  int goal[2] = {k.goal[0], k.goal[1]};
  int start[2] = {k.start[0], k.start[1]};
  nav.setGoal(goal);
  nav.setStart(start);
  int calls = 0;
  std::function<bool()> cancel = [&]() {
      ++calls;
      return false;
    };
  bool result = k.astar ? nav.calcNavFnAstar(cancel) : nav.calcNavFnDijkstra(cancel, k.at_start);

  json out = {{"name", k.name}, {"nx", k.nx}, {"ny", k.ny}, {"allow_unknown", k.allow_unknown},
    {"goal", k.goal}, {"start", k.start}, {"astar", k.astar}, {"at_start", k.at_start},
    {"result", result}, {"nobs", nav.nobs}, {"cancel_calls", calls}, {"potential", pot_summary(nav)}};
  out["cmap"] = k.open ? json("open") : bytes(k.cmap.data(), k.nx * k.ny);
  if (k.dump_pot) {
    out["potarr"] = floats(nav.potarr, nav.ns);
  }
  if (k.astar) {
    out["last_path_cost"] = nav.getLastPathCost();
  }

  int max_cycles = 4 * std::max(k.nx, k.ny);
  json paths = json::array();
  auto one = [&](int * st, int n) {
      int len = st ? nav.calcPath(n, st) : nav.calcPath(n);
      paths.push_back(
        {{"st", st ? json{st[0], st[1]} : json(nullptr)}, {"n", n}, {"len", len},
          {"x", floats(nav.getPathX(), nav.getPathLen())},
          {"y", floats(nav.getPathY(), nav.getPathLen())}});
    };
  one(nullptr, max_cycles);
  for (auto & e : k.extra_starts) {
    int st[2] = {e[0], e[1]};
    one(st, e[2] > 0 ? e[2] : max_cycles);
  }
  out["paths"] = paths;
  return out;
}

// Inflation-like costs: lethal discs, costs decaying away from them, unknown
// rectangles and a few 253s.
static std::vector<uint8_t> cost_grid(int nx, int ny, std::mt19937 & rng)
{
  std::vector<uint8_t> c(nx * ny, 0);
  std::uniform_int_distribution<int> ux(3, nx - 4), uy(3, ny - 4), ur(1, 3);
  std::vector<std::array<int, 3>> discs;
  for (int k = 0; k < 7; ++k) {
    discs.push_back({ux(rng), uy(rng), ur(rng)});
  }
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      double d = 1e9;
      for (auto & s : discs) {
        d = std::min(d, std::hypot(x - s[0], y - s[1]) - s[2]);
      }
      if (d <= 0) {
        c[y * nx + x] = 254;
      } else if (d < 6) {
        c[y * nx + x] = static_cast<uint8_t>(std::max(1.0, 252.0 * std::exp(-0.7 * (d - 0.5))));
      }
    }
  }
  for (int y = 30; y < 36; ++y) {
    for (int x = 8; x < 20; ++x) {
      c[y * nx + x] = 255;
    }
  }
  for (int y = 5; y < 9; ++y) {
    for (int x = 40; x < 47; ++x) {
      c[y * nx + x] = 255;
    }
  }
  std::uniform_int_distribution<int> any(0, nx * ny - 1);
  for (int k = 0; k < 12; ++k) {
    c[any(rng)] = 253;
  }
  return c;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::mt19937 rng(20261025);
  json out;

  // --- glibc hypotf / hypot as the port computes them -----------------------------
  {
    std::uniform_real_distribution<float> u(-300.f, 300.f), s(-1.f, 1.f);
    std::uniform_int_distribution<int> e(-30, 30);
    long mismatches = 0, checked = 0;
    json samples = json::array();
    for (int i = 0; i < 2000000; ++i) {
      float x, y;
      if (i % 3 == 0) {
        x = u(rng);
        y = u(rng);
      } else if (i % 3 == 1) {
        x = s(rng);
        y = s(rng);
      } else {
        x = std::ldexp(s(rng), e(rng));
        y = std::ldexp(s(rng), e(rng));
      }
      float h = hypotf(x, y);
      float f = static_cast<float>(std::sqrt(static_cast<double>(x) * x + static_cast<double>(y) * y));
      ++checked;
      mismatches += std::memcmp(&h, &f, 4) != 0;
      if (i % 1000 == 0) {
        samples.push_back({x, y, h});
      }
    }
    long imismatches = 0, ichecked = 0;
    for (int a = -400; a <= 400; ++a) {
      for (int b = -400; b <= 400; ++b) {
        ++ichecked;
        imismatches += hypot(a, b) != std::sqrt(static_cast<double>(a * a + b * b));
      }
    }
    out["hypot"] = {{"hypotf_samples", samples}, {"hypotf_checked", checked},
      {"hypotf_mismatches", mismatches}, {"hypot_int_checked", ichecked},
      {"hypot_int_mismatches", imismatches}};
  }

  // --- NavFn::setCostmap on every cost ------------------------------------------
  {
    std::vector<uint8_t> all(256);
    std::iota(all.begin(), all.end(), 0);
    NavFn nav(16, 16);
    nav.setCostmap(all.data(), true, true);
    json ros_unknown = bytes(nav.costarr, 256);
    nav.setCostmap(all.data(), true, false);
    json ros_no_unknown = bytes(nav.costarr, 256);
    std::vector<uint8_t> v(32 * 32);
    for (int i = 0; i < 32 * 32; ++i) {
      v[i] = static_cast<uint8_t>((i * 37) % 256);
    }
    NavFn pgm(32, 32);
    pgm.setCostmap(v.data(), false, false);
    out["set_costmap"] = {{"ros_allow_unknown", ros_unknown}, {"ros_no_unknown", ros_no_unknown},
      {"pgm", {{"nx", 32}, {"ny", 32}, {"cmap", bytes(v.data(), 1024)}, {"costarr", bytes(pgm.costarr, 1024)}}}};
  }

  // --- NavFn directly ---------------------------------------------------------------
  {
    json cases = json::array();
    auto costs = cost_grid(64, 48, rng);
    NavfnCase full{"costs_full", 64, 48, costs};
    full.goal = {10, 8};
    full.start = {55, 40};
    full.at_start = false;
    full.dump_pot = true;
    // the last three fail: out of steps, and on the bottom / top row
    full.extra_starts = {{5, 40, 0}, {60, 5, 0}, {32, 24, 0}, {1, 1, 0}, {0, 10, 0}, {62, 46, 0},
      {12, 32, 0}, {43, 6, 0}, {55, 40, 12}, {20, 0, 0}, {20, 47, 0}};
    cases.push_back(run_navfn(full));

    NavfnCase at_start = full;
    at_start.name = "costs_at_start";
    at_start.at_start = true;
    at_start.dump_pot = false;
    cases.push_back(run_navfn(at_start));

    NavfnCase astar = full;
    astar.name = "costs_astar";
    astar.astar = true;
    cases.push_back(run_navfn(astar));

    NavfnCase no_unknown = at_start;
    no_unknown.name = "costs_no_unknown";
    no_unknown.allow_unknown = false;
    cases.push_back(run_navfn(no_unknown));

    // a wavefront longer than PRIORITYBUFSIZE: pushes are dropped
    NavfnCase big{"big_open", 2400, 2400, std::vector<uint8_t>(2400 * 2400, 0)};
    big.open = true;
    big.goal = {1200, 1200};
    big.start = {2380, 1203};
    big.extra_starts = {{1201, 30, 0}};
    cases.push_back(run_navfn(big));

    NavfnCase tiny3{"tiny_3x3", 3, 3, std::vector<uint8_t>(9, 0)};
    tiny3.goal = {1, 1};
    tiny3.start = {1, 1};
    cases.push_back(run_navfn(tiny3));
    NavfnCase tiny4{"tiny_4x4", 4, 4, std::vector<uint8_t>(16, 0)};
    tiny4.goal = {1, 1};
    tiny4.start = {2, 2};
    tiny4.dump_pot = true;
    cases.push_back(run_navfn(tiny4));

    NavfnCase border{"goal_on_border", 20, 12, std::vector<uint8_t>(240, 0)};
    border.goal = {0, 5};
    border.start = {15, 6};
    border.dump_pot = true;
    cases.push_back(run_navfn(border));
    out["navfn"] = cases;
  }

  // --- NavfnPlanner -----------------------------------------------------------------
  Grid zone = zone_grid(), edge = edge_grid(), unknown = unknown_grid(), maze = maze_grid();
  out["grids"] = {{"zone", zone.to_json()}, {"edge", edge.to_json()}, {"unknown", unknown.to_json()},
    {"maze", maze.to_json()}};
  json scenarios = json::array();

  {
    const Grid & g = zone;
    // tree edges: (i, j) free next to lethal (i + 1, j), and lethal next to free
    int jt = g.cj(1.41), il = -1, ir = -1;
    for (int i = 0; i < g.w; ++i) {
      if (g.cx(i) > 0.5 && g.cx(i) < 2.0 && g.get(i, jt) == 100) {
        if (il < 0) {
          il = i;
        }
        ir = i;
      }
    }
    // the lethal cell just below the zone's bottom edge at x = 2.0
    int ie = g.ci(2.0), je = 0;
    while (g.get(ie, je + 1) != 0) {
      ++je;
    }
    int ri0 = g.ci(1.80), ri1 = g.ci(2.65), rj0 = g.cj(3.60);
    int ring_i = (ri0 + ri1) / 2;
    Pt island{(1.80 + 2.65) / 2, (3.60 + 4.45) / 2};
    int si = g.ci(0.8), sj = g.cj(2.4);

    std::vector<Req> robot{
      {"across_the_wall", {-2.0, -1.0}, 0.3, {5.6, 2.6}, 1.0},
      {"one_cell_gap", {-1.5, 0.0}, 0.0, {0.5, 0.0}, -0.4},
      {"three_cell_gap", {-1.8, 4.0}, 2.0, {1.0, 5.0}, 3.1},
      {"goal_in_unknown", {3.0, 1.5}, 0.0, {5.6, 1.6}, -0.5},
      {"start_in_unknown", {5.4, 1.4}, 1.0, {2.0, -1.0}, 0.0},
      {"goal_lethal", {0.7, 2.2}, 0.0, {1.23, 1.41}, 0.0},
      {"goal_truncated_free_rounded_lethal", {0.7, 2.2}, 0.0, g.at(il - 1, jt, 0.7, 0.3), 0.0},
      {"goal_truncated_lethal_rounded_free", {2.0, -1.0}, 0.0, g.at(ir, jt, 0.7, 0.3), 0.0},
      {"goal_outside_map", {0.7, 2.2}, 0.0, {8.5, 2.0}, 0.0},
      {"start_outside_map", {-3.5, 0.0}, 0.0, {0.7, 2.2}, 0.0},
      {"both_outside_map", {-4.0, 0.0}, 0.0, {9.0, 0.0}, 0.0},
      {"goal_in_island", {0.7, 2.2}, 0.0, island, 0.0},
      {"start_equals_goal", {0.7, 2.2}, 0.1, {0.7, 2.2}, 2.5},
      {"start_equals_goal_same_yaw", {0.7, 2.2}, -1.0, {0.7, 2.2}, -1.0},
      {"start_equals_goal_in_lethal", {1.23, 1.41}, 0.0, {1.23, 1.41}, 1.0},
      {"same_rounded_cell", g.at(si, sj, 0.1, 0.15), 0.2, g.at(si, sj, 0.4, -0.3), 1.7},
      {"start_in_lethal_next_to_free", g.at(ie, je, 0.2, 0.2), 0.0, {3.0, 1.5}, 0.5},
      {"start_deep_in_lethal", {-3.0, -1.8}, 0.0, {0.7, 2.2}, 0.0},
      {"start_on_island_ring", g.at(ring_i, rj0, 0.2, 0.2), 0.0, {2.2, 3.0}, 0.0},
      {"goal_in_island_after_ring_cleared", {0.7, 2.2}, 0.0, island, 0.7},
      {"cancelled", {-2.0, -1.0}, 0.0, {5.6, 2.6}, 0.0, 1},
      {"long_diagonal", {6.5, -1.2}, 2.2, {-2.2, 5.8}, -2.9},
    };
    scenarios.push_back(run_scenario("zone_robot", g, kRobot, robot));

    std::vector<Req> defaults{
      {"goal_lethal_within_tolerance", {0.7, 2.2}, 0.0, {1.23, 1.41}, 0.4},
      {"goal_in_island", {0.7, 2.2}, 0.0, island, 0.0},
      {"goal_far_outside_zone", {0.7, 2.2}, 0.0, {7.6, 6.3}, 0.0},
      {"goal_just_outside_zone", {2.0, -1.0}, 0.0, {2.0, -1.75}, -1.2},
      {"across_the_wall", {-2.0, -1.0}, 0.3, {5.6, 2.6}, 1.0},
      {"goal_truncated_lethal_rounded_free", {2.0, -1.0}, 0.0, g.at(ir, jt, 0.7, 0.3), 0.0},
    };
    scenarios.push_back(run_scenario("zone_defaults", g, kDefaults, defaults));

    std::vector<Req> astar{
      {"across_the_wall", {-2.0, -1.0}, 0.3, {5.6, 2.6}, 1.0},
      {"goal_in_unknown", {3.0, 1.5}, 0.0, {5.6, 1.6}, -0.5},
      {"one_cell_gap", {-1.5, 0.0}, 0.0, {0.5, 0.0}, -0.4},
      {"goal_in_island", {0.7, 2.2}, 0.0, island, 0.0},
      {"long_diagonal", {6.5, -1.2}, 2.2, {-2.2, 5.8}, -2.9},
    };
    scenarios.push_back(run_scenario("zone_astar", g, kAstar, astar));

    std::vector<Req> final_approach{
      {"across_the_wall", {-2.0, -1.0}, 0.3, {5.6, 2.6}, 1.0},
      {"same_rounded_cell", g.at(si, sj, 0.1, 0.15), 0.2, g.at(si, sj, 0.4, -0.3), 1.7},
      {"start_equals_goal", {0.7, 2.2}, 0.1, {0.7, 2.2}, 2.5},
      {"three_cell_gap", {-1.8, 4.0}, 2.0, {1.0, 5.0}, 3.1},
    };
    scenarios.push_back(run_scenario("zone_final_approach", g, kFinalApproach, final_approach));
  }

  {
    const Grid & g = edge;
    std::vector<Req> robot{
      {"start_rounds_to_left_border", g.at(0, 12, 0.3, 0.2), 0.0, g.at(40, 20, 0.4, 0.3), 0.5},
      {"start_rounds_past_right_edge", g.at(g.w - 1, 15, 0.7, 0.2), 0.0, g.at(30, 10, 0.2, 0.2), 0.0},
      {"start_rounds_to_bottom_border", g.at(20, 0, 0.2, 0.3), 0.0, g.at(50, 30, 0.2, 0.2), 1.0},
      {"start_at_origin", {g.ox, g.oy}, 0.0, g.at(10, 10, 0.2, 0.2), 0.0},
      {"goal_rounds_to_left_border", g.at(10, 10, 0.2, 0.2), 0.0, g.at(0, 22, 0.3, 0.2), 0.0},
      {"goal_rounds_past_right_edge", g.at(10, 10, 0.2, 0.2), 0.0, g.at(g.w - 1, 22, 0.7, 0.2), 0.0},
      {"goal_rounds_to_top_border", g.at(10, 10, 0.2, 0.2), 0.0, g.at(30, g.h - 1, 0.2, 0.2), 0.0},
      {"goal_at_map_end", g.at(10, 10, 0.2, 0.2), 0.0, {g.ox + g.w * g.res, g.oy + 1.0}, 0.0},
      {"start_below_origin", {g.ox - 1e-9, g.oy + 1.0}, 0.0, g.at(10, 10, 0.2, 0.2), 0.0},
      {"along_the_bottom_row", g.at(2, 1, 0.2, 0.2), 0.0, g.at(55, 1, 0.2, 0.2), 0.0},
      {"start_in_unknown_band", g.at(4, 30, 0.2, 0.2), 0.0, g.at(50, 30, 0.2, 0.2), 0.0},
      {"around_the_obstacles", g.at(15, 20, 0.2, 0.2), -0.5, g.at(55, 25, 0.2, 0.2), 0.8},
    };
    scenarios.push_back(run_scenario("edge_robot", g, kRobot, robot));
  }

  {
    const Grid & g = unknown;
    std::vector<Req> reqs{
      {"around_the_wall_through_unknown", {2.0, 3.0}, 0.0, {6.0, 3.0}, 0.0},
      {"goal_in_unknown_outside_zone", {2.0, 2.0}, 0.0, {7.6, 6.0}, 0.3},
      {"start_in_unknown_outside_zone", {0.6, 0.5}, 0.0, {2.5, 2.5}, 0.0},
    };
    scenarios.push_back(run_scenario("unknown_robot", g, kRobot, reqs));
    scenarios.push_back(run_scenario("unknown_no_unknown", g, kNoUnknown, reqs));
  }

  {
    const Grid & g = maze;
    std::vector<Req> reqs{
      {"next_corridor", g.at(5, 2, 0.2, 0.2), 0.0, g.at(5, 6, 0.2, 0.2), 0.0},
      {"far_corridor_beyond_cycle_limit", g.at(5, 2, 0.2, 0.2), 0.0, g.at(30, 54, 0.2, 0.2), 0.0},
      {"same_corridor", g.at(5, 2, 0.2, 0.2), 0.0, g.at(50, 2, 0.2, 0.2), 0.0},
      {"cancelled", g.at(5, 2, 0.2, 0.2), 0.0, g.at(5, 6, 0.2, 0.2), 0.0, 1},
    };
    scenarios.push_back(run_scenario("maze_robot", g, kRobot, reqs));
  }
  out["scenarios"] = scenarios;

  std::ofstream(argv[argc - 1]) << out.dump() << "\n";
  rclcpp::shutdown();
  return 0;
}
