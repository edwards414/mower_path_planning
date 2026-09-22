// Oracle for the mower_localize_core port: links the installed
// robot_localization (Jazzy 3.8.3, librl_lib.so) and GeographicLib, runs a
// deterministic synthetic sequence through the real implementation, and dumps
// the inputs and the resulting state/covariance as JSON for the Rust tests.
//
// RosFilter's preprocessing and NavSatTransform's maths are non-public members,
// so this harness reopens them. That changes no layout or ABI, it only lets the
// harness call them; the code under test is still the prebuilt shared library.
// Every system/ROS header is pulled in first, with normal access control; the
// access-reopening macros below then only affect robot_localization's own
// headers (they change no layout or ABI, they just let the harness call the
// preprocessing and the navsat maths, which are protected/private members).
#include <deque>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <cstdio>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "Eigen/Dense"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "GeographicLib/UTMUPS.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "robot_localization/srv/from_ll_array.hpp"
#include "robot_localization/srv/set_datum.hpp"
#include "robot_localization/srv/set_pose.hpp"
#include "robot_localization/srv/set_utm_zone.hpp"
#include "robot_localization/srv/to_ll.hpp"
#include "robot_localization/srv/toggle_filter_processing.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#define private public
#define protected public
#include "robot_localization/ekf.hpp"
#include "robot_localization/filter_common.hpp"
#include "robot_localization/measurement.hpp"
#include "robot_localization/ros_filter.hpp"
#include "robot_localization/ros_filter_types.hpp"
#include "robot_localization/navsat_transform.hpp"
#undef private
#undef protected


using robot_localization::Ekf;
using robot_localization::Measurement;
using robot_localization::STATE_SIZE;

static FILE * out = nullptr;

static void jnum(double v)
{
  if (std::isnan(v)) {
    fprintf(out, "\"nan\"");
  } else if (std::isinf(v)) {
    fprintf(out, v > 0 ? "\"inf\"" : "\"-inf\"");
  } else if (v == std::numeric_limits<double>::max()) {
    fprintf(out, "1.7976931348623157e308");
  } else {
    fprintf(out, "%.17g", v);
  }
}

static void jvec(const Eigen::VectorXd & v)
{
  fprintf(out, "[");
  for (int i = 0; i < v.rows(); ++i) {
    if (i) {fprintf(out, ",");}
    jnum(v(i));
  }
  fprintf(out, "]");
}

static void jmat(const Eigen::MatrixXd & m)
{
  fprintf(out, "[");
  for (int i = 0; i < m.rows(); ++i) {
    if (i) {fprintf(out, ",");}
    fprintf(out, "[");
    for (int j = 0; j < m.cols(); ++j) {
      if (j) {fprintf(out, ",");}
      jnum(m(i, j));
    }
    fprintf(out, "]");
  }
  fprintf(out, "]");
}

static void jbools(const std::vector<bool> & v)
{
  fprintf(out, "[");
  for (size_t i = 0; i < v.size(); ++i) {
    if (i) {fprintf(out, ",");}
    fprintf(out, v[i] ? "true" : "false");
  }
  fprintf(out, "]");
}

static void jtransform(const char * target, const char * source, const tf2::Transform & t)
{
  fprintf(out, "{\"target\":\"%s\",\"source\":\"%s\",\"origin\":[", target, source);
  jnum(t.getOrigin().getX()); fprintf(out, ",");
  jnum(t.getOrigin().getY()); fprintf(out, ",");
  jnum(t.getOrigin().getZ());
  fprintf(out, "],\"rotation\":[");
  tf2::Quaternion q = t.getRotation();
  jnum(q.x()); fprintf(out, ",");
  jnum(q.y()); fprintf(out, ",");
  jnum(q.z()); fprintf(out, ",");
  jnum(q.w());
  fprintf(out, "]}");
}

// A tiny deterministic generator so the sequence is reproducible.
struct Lcg
{
  uint64_t s = 88172645463325252ull;
  double next()
  {
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
    return static_cast<double>(s % 2000000ull) / 1000000.0 - 1.0;  // [-1, 1)
  }
};

static const int64_t T0 = 1700000000000000000LL;

static rclcpp::Time stamp(int64_t ns) {return rclcpp::Time(ns, RCL_ROS_TIME);}

// ---------------------------------------------------------------------------
// 1. The filter core: measurements fed straight to Ekf::processMeasurement.
// ---------------------------------------------------------------------------
static void ekf_core_vectors(const char * path)
{
  out = fopen(path, "w");
  Ekf filter;

  Eigen::MatrixXd q(STATE_SIZE, STATE_SIZE);
  q.setZero();
  const double qdiag[STATE_SIZE] =
  {1.0, 1.0, 1e-3, 0.3, 0.3, 0.01, 0.5, 0.5, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  for (int i = 0; i < STATE_SIZE; ++i) {q(i, i) = qdiag[i];}
  filter.setProcessNoiseCovariance(q);
  filter.setSensorTimeout(rclcpp::Duration::from_nanoseconds(50000000));

  fprintf(out, "{\n\"process_noise_covariance\":");
  jmat(q);
  fprintf(out, ",\n\"sensor_timeout_ns\":50000000,\n\"steps\":[\n");

  // The 2D-mode variables robot_localization forces on every measurement.
  const int two_d[] = {2, 3, 4, 8, 9, 10, 14};

  Lcg rng;
  bool first = true;
  int64_t t = T0;
  // 200 cycles at 25 Hz: odom every cycle, imu every 2.5, gps every 6.25.
  for (int cycle = 0; cycle < 80; ++cycle) {
    std::vector<std::pair<int64_t, int>> events;   // (time, kind)
    events.emplace_back(t, 0);                     // odom twist
    if (cycle % 2 == 0) {events.emplace_back(t + 7000000, 1);}      // imu
    if (cycle % 6 == 0) {events.emplace_back(t + 3000000, 2);}      // gps
    // Out-of-sequence data: every 37th cycle an odom sample arrives 80 ms late.
    if (cycle % 37 == 36) {events.emplace_back(t - 80000000, 0);}
    // A duplicate timestamp (delta == 0) every 53rd cycle.
    if (cycle % 53 == 52) {events.emplace_back(t, 1);}

    for (auto & ev : events) {
      Measurement m;
      m.time_ = stamp(ev.first);
      m.measurement_ = Eigen::VectorXd(STATE_SIZE);
      m.measurement_.setZero();
      m.covariance_ = Eigen::MatrixXd(STATE_SIZE, STATE_SIZE);
      m.covariance_.setZero();
      m.update_vector_ = std::vector<bool>(STATE_SIZE, false);
      m.latest_control_ = Eigen::VectorXd(6);
      m.latest_control_.setZero();
      m.latest_control_time_ = stamp(0);

      if (ev.second == 0) {
        m.topic_name_ = "odom";
        m.measurement_(6) = 0.4 + 0.05 * rng.next();
        m.measurement_(7) = 0.01 * rng.next();
        m.measurement_(11) = 0.15 + 0.02 * rng.next();
        m.update_vector_[6] = m.update_vector_[7] = m.update_vector_[11] = true;
        m.covariance_(6, 6) = 0.002;
        m.covariance_(7, 7) = 0.002;
        m.covariance_(11, 11) = 0.004;
        m.mahalanobis_thresh_ = std::numeric_limits<double>::max();
      } else if (ev.second == 1) {
        m.topic_name_ = "imu";
        m.measurement_(5) = 0.3 * rng.next();
        m.measurement_(11) = 0.15 + 0.03 * rng.next();
        m.measurement_(12) = 0.05 * rng.next();
        m.measurement_(13) = 0.05 * rng.next();
        m.update_vector_[5] = m.update_vector_[11] = true;
        m.update_vector_[12] = m.update_vector_[13] = true;
        m.covariance_(5, 5) = 0.01;
        m.covariance_(11, 11) = 0.005;
        m.covariance_(12, 12) = 0.02;
        m.covariance_(13, 13) = 0.02;
        // Exercise the Mahalanobis gate on one sensor.
        m.mahalanobis_thresh_ = 4.0;
      } else {
        m.topic_name_ = "odometry/gps";
        m.measurement_(0) = 0.4 * cycle * 0.04 + 0.2 * rng.next();
        m.measurement_(1) = 0.05 * cycle * 0.04 + 0.2 * rng.next();
        m.update_vector_[0] = m.update_vector_[1] = true;
        m.covariance_(0, 0) = 0.5;
        m.covariance_(1, 1) = 0.5;
        m.covariance_(0, 1) = 0.05;
        m.covariance_(1, 0) = 0.05;
        m.mahalanobis_thresh_ = std::numeric_limits<double>::max();
      }

      // two_d_mode is on for both filters, so the preprocessing always pins
      // these seven variables.
      for (int idx : two_d) {
        m.measurement_(idx) = 0.0;
        m.covariance_(idx, idx) = 1e-6;
        m.update_vector_[idx] = true;
      }

      if (!first) {fprintf(out, ",\n");}
      first = false;
      fprintf(out, "{\"topic\":\"%s\",\"time_ns\":%ld,\"mahalanobis\":",
        m.topic_name_.c_str(), ev.first);
      jnum(m.mahalanobis_thresh_);
      fprintf(out, ",\n \"update_vector\":");
      jbools(m.update_vector_);
      fprintf(out, ",\n \"measurement\":");
      jvec(m.measurement_);
      fprintf(out, ",\n \"covariance\":");
      jmat(m.covariance_);

      filter.processMeasurement(m);

      fprintf(out, ",\n \"state\":");
      jvec(filter.getState());
      fprintf(out, ",\n \"estimate_error_covariance\":");
      jmat(filter.getEstimateErrorCovariance());
      fprintf(out, ",\n \"last_measurement_time_ns\":%ld}",
        filter.getLastMeasurementTime().nanoseconds());
    }
    t += 40000000;   // 25 Hz
  }
  fprintf(out, "\n]}\n");
  fclose(out);
}

// ---------------------------------------------------------------------------
// 2. The RosFilter preprocessing: real messages through the real callbacks.
// ---------------------------------------------------------------------------
struct Frames
{
  tf2::Transform base_to_imu;
  tf2::Transform base_to_gps;
  tf2::Transform map_to_odom;
  tf2::Transform map_to_base;
};

static Frames robot_frames()
{
  // From mower_description/mower_robot/robot.urdf.xacro.
  tf2::Transform bf_to_bl;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 1.5708);
  bf_to_bl.setRotation(q);
  bf_to_bl.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

  tf2::Transform bl_to_imu;
  tf2::Quaternion qi;
  qi.setRPY(3.14159, 0.0, 0.0);
  bl_to_imu.setRotation(qi);
  bl_to_imu.setOrigin(tf2::Vector3(0.015897, 0.209534, -0.0129528));

  tf2::Transform bl_to_gps;
  bl_to_gps.setRotation(tf2::Quaternion::getIdentity());
  bl_to_gps.setOrigin(tf2::Vector3(0.0, 0.0, 0.12));

  Frames f;
  f.base_to_imu = bf_to_bl * bl_to_imu;
  f.base_to_gps = bf_to_bl * bl_to_gps;
  // A plausible, fixed map->odom and map->base_footprint for the lookups the
  // map instance and navsat make.
  tf2::Quaternion qm;
  qm.setRPY(0.0, 0.0, 0.05);
  f.map_to_odom.setRotation(qm);
  f.map_to_odom.setOrigin(tf2::Vector3(1.25, -0.75, 0.0));
  tf2::Quaternion qb;
  qb.setRPY(0.0, 0.0, 0.31);
  f.map_to_base.setRotation(qb);
  f.map_to_base.setOrigin(tf2::Vector3(3.5, 2.25, 0.0));
  return f;
}

static void add_static(
  tf2_ros::Buffer * buffer, const std::string & target,
  const std::string & source, const tf2::Transform & t)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = target;
  msg.child_frame_id = source;
  msg.transform = tf2::toMsg(t);
  buffer->setTransform(msg, "oracle", true);
}

static void dump_measurement_queue(robot_localization::RosEkf & f, bool & first)
{
  std::vector<robot_localization::MeasurementPtr> popped;
  while (!f.measurement_queue_.empty()) {
    popped.push_back(f.measurement_queue_.top());
    f.measurement_queue_.pop();
  }
  fprintf(out, "\"prepared\":[");
  for (size_t i = 0; i < popped.size(); ++i) {
    if (i) {fprintf(out, ",");}
    fprintf(out, "{\"topic\":\"%s\",\"time_ns\":%ld,\"update_vector\":",
      popped[i]->topic_name_.c_str(), popped[i]->time_.nanoseconds());
    jbools(popped[i]->update_vector_);
    fprintf(out, ",\"measurement\":");
    jvec(popped[i]->measurement_);
    fprintf(out, ",\"covariance\":");
    jmat(popped[i]->covariance_);
    fprintf(out, "}");
  }
  fprintf(out, "]");
  // Put them back in order so integrateMeasurements sees the same queue.
  for (auto & m : popped) {f.measurement_queue_.push(m);}
  (void)first;
}

static void ros_filter_vectors(const char * path, bool map_instance)
{
  out = fopen(path, "w");
  Frames frames = robot_frames();

  rclcpp::NodeOptions options;
  options.arguments({map_instance ? "ekf_filter_node_map" : "ekf_filter_node_odom"});
  options.clock_type(RCL_ROS_TIME);
  auto f = std::make_shared<robot_localization::RosEkf>(options);

  f->two_d_mode_ = true;
  f->print_diagnostics_ = false;
  f->map_frame_id_ = "map";
  f->odom_frame_id_ = "odom";
  f->base_link_frame_id_ = "base_footprint";
  f->base_link_output_frame_id_ = "base_footprint";
  f->world_frame_id_ = map_instance ? "map" : "odom";
  f->gravitational_acceleration_ = 9.80665;
  f->tf_timeout_ = rclcpp::Duration(0, 0u);
  f->smooth_lagged_data_ = false;
  f->predict_to_current_time_ = false;
  // RosFilter::reset() normally does this from initialize(); we do it by hand
  // so the tf buffer we fill below survives.
  f->angular_acceleration_cov_.resize(3, 3);
  f->angular_acceleration_cov_.setIdentity();
  f->angular_acceleration_cov_ *= 0.01;
  f->last_diff_time_ = static_cast<double>(T0) * 1e-9;
  f->last_state_twist_rot_.setZero();
  f->angular_acceleration_.setZero();

  Eigen::MatrixXd q(STATE_SIZE, STATE_SIZE);
  q.setZero();
  const double q_odom[STATE_SIZE] =
  {1e-3, 1e-3, 1e-3, 0.3, 0.3, 0.01, 0.5, 0.5, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  const double q_map[STATE_SIZE] =
  {1.0, 1.0, 1e-3, 0.3, 0.3, 0.01, 0.5, 0.5, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  for (int i = 0; i < STATE_SIZE; ++i) {q(i, i) = map_instance ? q_map[i] : q_odom[i];}
  f->filter_.setProcessNoiseCovariance(q);
  f->filter_.setSensorTimeout(rclcpp::Duration::from_nanoseconds(50000000));

  add_static(f->tf_buffer_.get(), "base_footprint", "imu_link", frames.base_to_imu);
  add_static(f->tf_buffer_.get(), "base_footprint", "gps_link", frames.base_to_gps);
  add_static(f->tf_buffer_.get(), "map", "odom", frames.map_to_odom);
  add_static(f->tf_buffer_.get(), "map", "base_footprint", frames.map_to_base);

  // The sensor configs straight out of dual_ekf_navsat_params.yaml.
  robot_localization::CallbackData odom_pose(
    "odom", std::vector<bool>(STATE_SIZE, false), 0, false, false, false,
    std::numeric_limits<double>::max());
  robot_localization::CallbackData odom_twist(
    "odom", std::vector<bool>(STATE_SIZE, false), 0, false, false, false,
    std::numeric_limits<double>::max());
  odom_twist.update_vector_[6] = true;
  odom_twist.update_vector_[7] = true;
  if (map_instance) {odom_twist.update_vector_[8] = true;}
  odom_twist.update_vector_[11] = true;
  odom_twist.update_sum_ = map_instance ? 4 : 3;

  robot_localization::CallbackData imu_pose(
    "imu", std::vector<bool>(STATE_SIZE, false), 0, false, false, false,
    std::numeric_limits<double>::max());
  imu_pose.update_vector_[5] = true;
  imu_pose.update_sum_ = 1;
  robot_localization::CallbackData imu_twist(
    "imu", std::vector<bool>(STATE_SIZE, false), 0, false, false, false,
    std::numeric_limits<double>::max());
  robot_localization::CallbackData imu_accel(
    "imu", std::vector<bool>(STATE_SIZE, false), 0, false, false, false,
    std::numeric_limits<double>::max());
  if (!map_instance) {
    imu_twist.update_vector_[11] = true;
    imu_twist.update_sum_ = 1;
    imu_accel.update_vector_[12] = true;
    imu_accel.update_vector_[13] = true;
    imu_accel.update_sum_ = 2;
  }
  f->remove_gravitational_acceleration_["imu"] = true;

  robot_localization::CallbackData gps_pose(
    "odometry/gps", std::vector<bool>(STATE_SIZE, false), 0, false, false, false,
    std::numeric_limits<double>::max());
  robot_localization::CallbackData gps_twist(
    "odometry/gps", std::vector<bool>(STATE_SIZE, false), 0, false, false, false,
    std::numeric_limits<double>::max());
  gps_pose.update_vector_[0] = true;
  gps_pose.update_vector_[1] = true;
  gps_pose.update_sum_ = 2;

  fprintf(out, "{\n\"instance\":\"%s\",\n\"process_noise_covariance\":",
    map_instance ? "map" : "odom");
  jmat(q);
  fprintf(out, ",\n\"sensor_timeout_ns\":50000000,\n\"transforms\":[");
  jtransform("base_footprint", "imu_link", frames.base_to_imu);
  fprintf(out, ",");
  jtransform("base_footprint", "gps_link", frames.base_to_gps);
  fprintf(out, ",");
  jtransform("map", "odom", frames.map_to_odom);
  fprintf(out, ",");
  jtransform("map", "base_footprint", frames.map_to_base);
  fprintf(out, "],\n\"steps\":[\n");

  Lcg rng;
  bool first = true;
  int64_t t = T0;
  int64_t now_ns = T0 - 1000000;
  for (int cycle = 0; cycle < 80; ++cycle) {
    struct Ev {int64_t ns; int kind;};
    std::vector<Ev> events;
    events.push_back({t, 0});
    if (cycle % 2 == 0) {events.push_back({t + 7000000, 1});}
    if (cycle % 6 == 0) {events.push_back({t + 3000000, 2});}
    if (cycle % 37 == 36) {events.push_back({t - 80000000, 0});}   // stale
    if (cycle % 53 == 52) {events.push_back({t, 1});}              // duplicate stamp

    for (auto & ev : events) {
      if (!first) {fprintf(out, ",\n");}
      first = false;

      if (ev.kind == 0) {
        auto msg = std::make_shared<nav_msgs::msg::Odometry>();
        msg->header.stamp = stamp(ev.ns);
        msg->header.frame_id = "odom";
        msg->child_frame_id = "base_footprint";
        msg->twist.twist.linear.x = 0.4 + 0.05 * rng.next();
        msg->twist.twist.linear.y = 0.01 * rng.next();
        msg->twist.twist.linear.z = 0.002 * rng.next();
        msg->twist.twist.angular.z = 0.15 + 0.02 * rng.next();
        msg->twist.covariance[0] = 0.002;
        msg->twist.covariance[7] = 0.002;
        msg->twist.covariance[14] = 0.01;
        msg->twist.covariance[35] = 0.004;
        msg->pose.covariance[0] = 0.01;
        msg->pose.covariance[7] = 0.01;
        msg->pose.covariance[35] = 0.02;
        fprintf(out, "{\"input\":{\"type\":\"odom\",\"stamp_ns\":%ld,"
          "\"frame_id\":\"odom\",\"child_frame_id\":\"base_footprint\","
          "\"twist_linear\":[%.17g,%.17g,%.17g],\"twist_angular\":[0,0,%.17g],"
          "\"twist_cov_diag\":[0.002,0.002,0.01,0,0,0.004]},\n ",
          ev.ns, msg->twist.twist.linear.x, msg->twist.twist.linear.y,
          msg->twist.twist.linear.z, msg->twist.twist.angular.z);
        f->odometryCallback(msg, "odom", odom_pose, odom_twist);
      } else if (ev.kind == 1) {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        msg->header.stamp = stamp(ev.ns);
        msg->header.frame_id = "imu_link";
        double yaw = 0.3 * rng.next();
        double roll = 0.02 * rng.next();
        double pitch = 0.02 * rng.next();
        tf2::Quaternion qq;
        qq.setRPY(roll, pitch, yaw);
        msg->orientation = tf2::toMsg(qq);
        msg->orientation_covariance[0] = 0.01;
        msg->orientation_covariance[4] = 0.01;
        msg->orientation_covariance[8] = 0.01;
        msg->angular_velocity.x = 0.01 * rng.next();
        msg->angular_velocity.y = 0.01 * rng.next();
        msg->angular_velocity.z = 0.15 + 0.03 * rng.next();
        msg->angular_velocity_covariance[0] = 0.005;
        msg->angular_velocity_covariance[4] = 0.005;
        msg->angular_velocity_covariance[8] = 0.005;
        msg->linear_acceleration.x = 0.05 * rng.next();
        msg->linear_acceleration.y = 0.05 * rng.next();
        msg->linear_acceleration.z = 9.7 + 0.05 * rng.next();
        msg->linear_acceleration_covariance[0] = 0.02;
        msg->linear_acceleration_covariance[4] = 0.02;
        msg->linear_acceleration_covariance[8] = 0.02;
        fprintf(out, "{\"input\":{\"type\":\"imu\",\"stamp_ns\":%ld,"
          "\"frame_id\":\"imu_link\",\"rpy\":[%.17g,%.17g,%.17g],"
          "\"angular_velocity\":[%.17g,%.17g,%.17g],"
          "\"linear_acceleration\":[%.17g,%.17g,%.17g],"
          "\"orientation_cov_diag\":[0.01,0.01,0.01],"
          "\"angular_velocity_cov_diag\":[0.005,0.005,0.005],"
          "\"linear_acceleration_cov_diag\":[0.02,0.02,0.02]},\n ",
          ev.ns, roll, pitch, yaw,
          msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
          msg->linear_acceleration.x, msg->linear_acceleration.y,
          msg->linear_acceleration.z);
        f->imuCallback(msg, "imu", imu_pose, imu_twist, imu_accel);
      } else {
        auto msg = std::make_shared<nav_msgs::msg::Odometry>();
        msg->header.stamp = stamp(ev.ns);
        msg->header.frame_id = "map";
        msg->child_frame_id = "";
        msg->pose.pose.position.x = 0.4 * cycle * 0.04 + 0.2 * rng.next();
        msg->pose.pose.position.y = 0.05 * cycle * 0.04 + 0.2 * rng.next();
        msg->pose.pose.orientation.w = 1.0;
        msg->pose.covariance[0] = 0.5;
        msg->pose.covariance[1] = 0.05;
        msg->pose.covariance[6] = 0.05;
        msg->pose.covariance[7] = 0.5;
        msg->pose.covariance[14] = 1.0;
        fprintf(out, "{\"input\":{\"type\":\"gps\",\"stamp_ns\":%ld,"
          "\"frame_id\":\"map\",\"position\":[%.17g,%.17g,0],"
          "\"pose_cov\":[0.5,0.05,0.05,0.5,1.0]},\n ",
          ev.ns, msg->pose.pose.position.x, msg->pose.pose.position.y);
        if (map_instance) {
          f->odometryCallback(msg, "odometry/gps", gps_pose, gps_twist);
        }
      }

      dump_measurement_queue(*f, first);

      // periodicUpdate always runs on a monotonically increasing wall clock.
      now_ns = std::max(now_ns + 1000000, ev.ns + 1000000);
      f->integrateMeasurements(stamp(now_ns));
      f->differentiateMeasurements(stamp(now_ns));

      fprintf(out, ",\"integrate_time_ns\":%ld,\"state\":", now_ns);
      jvec(f->filter_.getState());
      fprintf(out, ",\"estimate_error_covariance\":");
      jmat(f->filter_.getEstimateErrorCovariance());
      fprintf(out, ",\"last_measurement_time_ns\":%ld,\"angular_acceleration\":[",
        f->filter_.getLastMeasurementTime().nanoseconds());
      jnum(f->angular_acceleration_.x()); fprintf(out, ",");
      jnum(f->angular_acceleration_.y()); fprintf(out, ",");
      jnum(f->angular_acceleration_.z()); fprintf(out, "]}");
    }
    t += 40000000;
  }
  fprintf(out, "\n]}\n");
  fclose(out);
}

// ---------------------------------------------------------------------------
// 3. navsat_transform: UTM vectors plus the datum/transform maths.
// ---------------------------------------------------------------------------
static void navsat_vectors(const char * path)
{
  out = fopen(path, "w");
  fprintf(out, "{\n\"utm\":[\n");
  const double pts[][2] = {
    {25.0330, 121.5654},   // Taipei (the sim fallback datum)
    {24.9990, 121.4999},
    {-33.8688, 151.2093},
    {51.4779, -0.0015},
    {40.7128, -74.0060},
    {-22.9068, -43.1729},
    {0.0, 0.0},
    {0.0001, 179.9999},
    {60.0, 5.0},           // the Norway exception
    {72.0, 20.0},          // the Svalbard exception
    {-79.9999, 100.0},
    {83.9999, -100.0},
    {25.0330001, 121.5654001},
    {45.123456789, -93.987654321},
  };
  for (size_t i = 0; i < sizeof(pts) / sizeof(pts[0]); ++i) {
    int zone; bool northp; double x, y, gamma, k;
    GeographicLib::UTMUPS::Forward(pts[i][0], pts[i][1], zone, northp, x, y, gamma, k, -1);
    double lat2, lon2, gamma2, k2;
    GeographicLib::UTMUPS::Reverse(zone, northp, x, y, lat2, lon2, gamma2, k2);
    if (i) {fprintf(out, ",\n");}
    fprintf(out,
      "{\"lat\":%.17g,\"lon\":%.17g,\"zone\":%d,\"northp\":%s,\"x\":%.17g,\"y\":%.17g,"
      "\"gamma\":%.17g,\"k\":%.17g,\"rev_lat\":%.17g,\"rev_lon\":%.17g}",
      pts[i][0], pts[i][1], zone, northp ? "true" : "false", x, y, gamma, k, lat2, lon2);
  }
  fprintf(out, "\n],\n");

  Frames frames = robot_frames();
  rclcpp::NodeOptions options;
  options.arguments({"navsat_transform"});
  options.clock_type(RCL_ROS_TIME);
  options.parameter_overrides(
  {
    rclcpp::Parameter("frequency", 30.0),
    rclcpp::Parameter("delay", 0.0),
    rclcpp::Parameter("magnetic_declination_radians", 0.0),
    rclcpp::Parameter("yaw_offset", 0.0),
    rclcpp::Parameter("zero_altitude", true),
    rclcpp::Parameter("broadcast_cartesian_transform", true),
    rclcpp::Parameter("publish_filtered_gps", true),
    rclcpp::Parameter("use_odometry_yaw", false),
    rclcpp::Parameter("wait_for_datum", false),
  });
  auto n = std::make_shared<robot_localization::NavSatTransform>(options);
  add_static(n->tf_buffer_.get(), "base_footprint", "imu_link", frames.base_to_imu);
  add_static(n->tf_buffer_.get(), "base_footprint", "gps_link", frames.base_to_gps);
  add_static(n->tf_buffer_.get(), "map", "base_footprint", frames.map_to_base);

  fprintf(out, "\"transforms\":[");
  jtransform("base_footprint", "imu_link", frames.base_to_imu);
  fprintf(out, ",");
  jtransform("base_footprint", "gps_link", frames.base_to_gps);
  fprintf(out, ",");
  jtransform("map", "base_footprint", frames.map_to_base);
  fprintf(out, "],\n\"steps\":[\n");

  Lcg rng;
  bool first = true;
  int64_t t = T0;
  const double lat0 = 25.0330, lon0 = 121.5654;
  for (int cycle = 0; cycle < 40; ++cycle) {
    // Filtered odometry in the map frame at 20 Hz (we only need one per cycle).
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = stamp(t);
    odom->header.frame_id = "map";
    odom->child_frame_id = "base_footprint";
    odom->pose.pose.position.x = 0.4 * cycle + 0.05 * rng.next();
    odom->pose.pose.position.y = 0.1 * cycle + 0.05 * rng.next();
    tf2::Quaternion qo;
    qo.setRPY(0.0, 0.0, 0.31 + 0.01 * rng.next());
    odom->pose.pose.orientation = tf2::toMsg(qo);
    for (int i = 0; i < 6; ++i) {odom->pose.covariance[i * 6 + i] = 0.1 * (i + 1);}
    n->odomCallback(odom);

    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu->header.stamp = stamp(t);
    imu->header.frame_id = "imu_link";
    tf2::Quaternion qi;
    qi.setRPY(0.01 * rng.next(), 0.01 * rng.next(), 0.9 + 0.02 * rng.next());
    imu->orientation = tf2::toMsg(qi);
    n->imuCallback(imu);

    auto fix = std::make_shared<sensor_msgs::msg::NavSatFix>();
    fix->header.stamp = stamp(t);
    fix->header.frame_id = "gps_link";
    fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix->latitude = lat0 + 3.6e-6 * cycle + 2e-7 * rng.next();
    fix->longitude = lon0 + 4.0e-6 * cycle + 2e-7 * rng.next();
    fix->altitude = 12.5 + 0.05 * rng.next();
    fix->position_covariance[0] = 0.04;
    fix->position_covariance[4] = 0.04;
    fix->position_covariance[8] = 0.09;
    n->gpsFixCallback(fix);

    n->computeTransform();

    nav_msgs::msg::Odometry gps_odom;
    bool have_odom = n->prepareGpsOdometry(&gps_odom);
    sensor_msgs::msg::NavSatFix filtered;
    bool have_fix = n->prepareFilteredGps(&filtered);

    if (!first) {fprintf(out, ",\n");}
    first = false;
    fprintf(out,
      "{\"stamp_ns\":%ld,\n \"odom\":{\"position\":[%.17g,%.17g,0],\"yaw_quat\":[%.17g,%.17g,%.17g,%.17g],"
      "\"cov_diag\":[0.1,0.2,0.3,0.4,0.5,0.6]},\n"
      " \"imu_quat\":[%.17g,%.17g,%.17g,%.17g],\n"
      " \"fix\":{\"lat\":%.17g,\"lon\":%.17g,\"alt\":%.17g,\"cov_diag\":[0.04,0.04,0.09]},\n"
      " \"transform_good\":%s,\"utm_zone\":%d,\"northp\":%s,\"meridian_convergence\":%.17g,\n",
      t,
      odom->pose.pose.position.x, odom->pose.pose.position.y,
      odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z, odom->pose.pose.orientation.w,
      imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w,
      fix->latitude, fix->longitude, fix->altitude,
      n->transform_good_ ? "true" : "false", n->utm_zone_,
      n->northp_ ? "true" : "false", n->utm_meridian_convergence_);
    fprintf(out, " \"cartesian_world_transform\":");
    jtransform("map", "utm", n->cartesian_world_transform_);
    fprintf(out, ",\n \"have_gps_odom\":%s", have_odom ? "true" : "false");
    if (have_odom) {
      fprintf(out, ",\n \"gps_odom_position\":[%.17g,%.17g,%.17g],\n \"gps_odom_cov\":",
        gps_odom.pose.pose.position.x, gps_odom.pose.pose.position.y,
        gps_odom.pose.pose.position.z);
      fprintf(out, "[");
      for (int i = 0; i < 36; ++i) {
        if (i) {fprintf(out, ",");}
        jnum(gps_odom.pose.covariance[i]);
      }
      fprintf(out, "]");
    }
    fprintf(out, ",\n \"have_filtered_gps\":%s", have_fix ? "true" : "false");
    if (have_fix) {
      fprintf(out, ",\n \"filtered_gps\":{\"lat\":%.17g,\"lon\":%.17g,\"alt\":%.17g,\"cov\":[",
        filtered.latitude, filtered.longitude, filtered.altitude);
      for (int i = 0; i < 9; ++i) {
        if (i) {fprintf(out, ",");}
        jnum(filtered.position_covariance[i]);
      }
      fprintf(out, "]}");
    }
    fprintf(out, "}");
    t += 50000000;
  }
  fprintf(out, "\n]}\n");
  fclose(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string dir = argc > 1 ? argv[1] : ".";
  ekf_core_vectors((dir + "/ekf_core.json").c_str());
  ros_filter_vectors((dir + "/ros_filter_odom.json").c_str(), false);
  ros_filter_vectors((dir + "/ros_filter_map.json").c_str(), true);
  navsat_vectors((dir + "/navsat.json").c_str());
  rclcpp::shutdown();
  return 0;
}
