// Oracle for mower_nav_core::{costmap, footprint}: drives the installed
// nav2_costmap_2d (Jazzy 1.3.13) — the real LayeredCostmap + StaticLayer in
// both configurations nav2_no_map_params.yaml uses, Costmap2D::updateOrigin,
// the footprint helpers and FootprintCollisionChecker — on deterministic
// synthetic inputs, and dumps inputs and results as JSON.
#include <cmath>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"

using nlohmann::json;
using nav2_costmap_2d::Costmap2D;

// interpretValue and incomingMap are protected; the harness only re-exposes them.
struct OracleStaticLayer : public nav2_costmap_2d::StaticLayer
{
  using nav2_costmap_2d::StaticLayer::incomingMap;
  unsigned char interpret(unsigned char v) {return interpretValue(v);}
};

static json costmap_json(const Costmap2D & c)
{
  json data = json::array();
  const unsigned char * cells = c.getCharMap();
  for (unsigned int i = 0; i < c.getSizeInCellsX() * c.getSizeInCellsY(); ++i) {
    data.push_back(static_cast<int>(cells[i]));
  }
  return {{"size_x", c.getSizeInCellsX()}, {"size_y", c.getSizeInCellsY()},
    {"resolution", c.getResolution()}, {"origin_x", c.getOriginX()},
    {"origin_y", c.getOriginY()}, {"data", data}};
}

static nav_msgs::msg::OccupancyGrid::SharedPtr random_grid(
  std::mt19937 & rng, unsigned int w, unsigned int h, double res, double ox, double oy)
{
  auto g = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  g->header.frame_id = "map";
  g->info.width = w;
  g->info.height = h;
  g->info.resolution = res;
  g->info.origin.position.x = ox;
  g->info.origin.position.y = oy;
  g->info.origin.orientation.w = 1.0;
  // free space with rectangles of lethal and of unknown, and a sprinkle of
  // the odd values (99 stays free, -2 and 101 read as >= 100)
  g->data.assign(w * h, 0);
  std::uniform_int_distribution<int> ux(0, w - 1), uy(0, h - 1), us(1, 10);
  auto rect = [&](int8_t v) {
      int x0 = ux(rng), y0 = uy(rng), sx = us(rng), sy = us(rng);
      for (int y = y0; y < std::min<int>(h, y0 + sy); ++y) {
        for (int x = x0; x < std::min<int>(w, x0 + sx); ++x) {
          g->data[y * w + x] = v;
        }
      }
    };
  for (unsigned int i = 0; i < w * h / 900; ++i) {
    rect(100);
  }
  for (unsigned int i = 0; i < w * h / 1500; ++i) {
    rect(-1);
  }
  const int8_t odd[] = {30, 99, -2, 101, 100, -1};
  std::uniform_int_distribution<int> pick(0, 5);
  for (unsigned int i = 0; i < w * h / 600; ++i) {
    g->data[uy(rng) * w + ux(rng)] = odd[pick(rng)];
  }
  return g;
}

static json grid_json(const nav_msgs::msg::OccupancyGrid & g)
{
  json data = json::array();
  for (auto v : g.data) {
    data.push_back(static_cast<int>(v));
  }
  return {{"width", g.info.width}, {"height", g.info.height}, {"resolution", g.info.resolution},
    {"origin_x", g.info.origin.position.x}, {"origin_y", g.info.origin.position.y}, {"data", data}};
}

static nav2_util::LifecycleNode::SharedPtr costmap_node(const std::string & name, bool track_unknown)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>(name);
  node->declare_parameter("track_unknown_space", rclcpp::ParameterValue(track_unknown));
  node->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
  node->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
  node->declare_parameter("unknown_cost_value", rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
  node->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
  node->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
  node->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  return node;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::mt19937 rng(20261025);
  json out;

  // --- footprint ------------------------------------------------------------
  auto fp = nav2_costmap_2d::makeFootprintFromRadius(0.5);
  float padding = 0.01f;  // Costmap2DROS's footprint_padding default, a float
  nav2_costmap_2d::padFootprint(fp, padding);
  auto radii = nav2_costmap_2d::calculateMinAndMaxDistances(fp);
  json fp_json = json::array();
  for (auto & p : fp) {
    fp_json.push_back({p.x, p.y});
  }
  out["footprint"] = {{"radius", 0.5}, {"points", fp_json},
    {"inscribed", radii.first}, {"circumscribed", radii.second}};

  // --- interpretValue -------------------------------------------------------
  {
    auto layered = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, true);
    json table = json::object();
    for (bool track : {false, true}) {
      auto node = costmap_node(track ? "interp_track" : "interp_free", track);
      tf2_ros::Buffer tf(node->get_clock());
      auto layer = std::make_shared<OracleStaticLayer>();
      layer->initialize(layered.get(), "static_layer", &tf, node, nullptr);
      json row = json::array();
      for (int v = 0; v < 256; ++v) {
        row.push_back(static_cast<int>(layer->interpret(static_cast<unsigned char>(v))));
      }
      table[track ? "track_unknown" : "no_track_unknown"] = row;
    }
    out["interpret"] = table;
  }

  // --- global costmap: /map_grid_global, frame map, not rolling -------------
  {
    auto grid = random_grid(rng, 80, 60, 0.05, -2.0, -1.5);
    auto node = costmap_node("global_costmap", true);
    tf2_ros::Buffer tf(node->get_clock());
    nav2_costmap_2d::LayeredCostmap layered("map", false, true);
    layered.resizeMap(100, 100, 0.05, 0.0, 0.0);
    auto layer = std::make_shared<OracleStaticLayer>();
    layer->initialize(&layered, "static_layer", &tf, node, nullptr);
    layered.addPlugin(layer);
    layer->incomingMap(grid);
    layered.updateMap(0.0, 0.0, 0.0);
    out["global"] = {{"grid", grid_json(*grid)}, {"costmap", costmap_json(*layered.getCostmap())}};

    // collisions on the global costmap
    nav2_costmap_2d::FootprintCollisionChecker<Costmap2D *> checker(layered.getCostmap());
    std::uniform_real_distribution<double> ux(-2.6, 2.6), uy(-2.1, 1.8), ut(-M_PI, M_PI);
    json cases = json::array();
    for (int i = 0; i < 400; ++i) {
      double x = ux(rng), y = uy(rng), th = ut(rng);
      cases.push_back({x, y, th, checker.footprintCostAtPose(x, y, th, fp)});
    }
    out["global_collision"] = cases;
  }

  // --- local costmap: /map_grid resampled into odom, rolling 3 x 3 m -------
  {
    auto grid = random_grid(rng, 120, 100, 0.05, -3.1, -2.4);
    auto node = costmap_node("local_costmap", false);
    tf2_ros::Buffer tf(node->get_clock());
    tf.setUsingDedicatedThread(true);  // static transform only, no listener thread needed
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.translation.x = 0.7;
    t.transform.translation.y = -0.4;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0.3);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf.setTransform(t, "oracle", true);

    nav2_costmap_2d::LayeredCostmap layered("odom", true, false);
    layered.resizeMap(static_cast<unsigned int>(3 / 0.05), static_cast<unsigned int>(3 / 0.05), 0.05, 0.0, 0.0);
    auto layer = std::make_shared<OracleStaticLayer>();
    layer->initialize(&layered, "static_layer", &tf, node, nullptr);
    layered.addPlugin(layer);
    layer->incomingMap(grid);

    json steps = json::array();
    std::uniform_real_distribution<double> ut(-M_PI, M_PI);
    double rx = -1.37, ry = 0.52;
    for (int i = 0; i < 14; ++i) {
      // a curved drive with a few jumps, through negative and positive cells
      rx += 0.083 * i - 0.31 + (i == 6 ? 1.9 : 0.0);
      ry += 0.137 * std::sin(0.7 * i) - (i == 9 ? 2.3 : 0.0);
      layered.updateMap(rx, ry, 0.0);
      json step = {{"robot", {rx, ry}}, {"costmap", costmap_json(*layered.getCostmap())}};
      nav2_costmap_2d::FootprintCollisionChecker<Costmap2D *> checker(layered.getCostmap());
      json cases = json::array();
      std::uniform_real_distribution<double> dx(-1.6, 1.6);
      for (int k = 0; k < 40; ++k) {
        double x = rx + dx(rng), y = ry + dx(rng), th = ut(rng);
        cases.push_back({x, y, th, checker.footprintCostAtPose(x, y, th, fp)});
      }
      step["collision"] = cases;
      steps.push_back(step);
    }
    out["local"] = {{"grid", grid_json(*grid)}, {"map_from_odom", {0.7, -0.4, 0.3}}, {"steps", steps}};
  }

  // --- Costmap2D::updateOrigin on its own ------------------------------------
  {
    Costmap2D c(7, 5, 0.3, 0.1, -0.2, 0);
    std::uniform_int_distribution<int> cell(0, 254);
    for (unsigned int i = 0; i < 35; ++i) {
      c.setCost(i % 7, i / 7, static_cast<unsigned char>(cell(rng)));
    }
    json initial = costmap_json(c);
    json steps = json::array();
    const double moves[][2] = {{0.4, 0.0}, {0.1, 0.65}, {-0.95, -0.2}, {0.0, -0.001}, {-0.31, 0.31},
      {2.5, -1.7}, {-10.0, 0.0}, {0.29, 0.29}, {0.61, -0.89}, {-0.6, 0.9}};
    for (auto & m : moves) {
      double nx = c.getOriginX() + m[0], ny = c.getOriginY() + m[1];
      c.updateOrigin(nx, ny);
      steps.push_back({{"new_origin", {nx, ny}}, {"costmap", costmap_json(c)}});
    }
    out["update_origin"] = {{"initial", initial}, {"steps", steps}};
  }

  std::ofstream(argv[argc - 1]) << out.dump() << "\n";
  rclcpp::shutdown();
  return 0;
}
