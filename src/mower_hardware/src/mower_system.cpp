#include "mower_hardware/mower_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mower_hardware {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace {

constexpr double kTwoPi = 6.283185307179586;

double param_or(const hardware_interface::HardwareInfo & info, const char * key, double def)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? def : std::stod(it->second);
}

std::string param_or(const hardware_interface::HardwareInfo & info, const char * key, const char * def)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? std::string(def) : it->second;
}

}  // namespace

CallbackReturn MowerSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  device_ = param_or(info, "device", "/dev/ttyS0");
  baud_ = static_cast<int>(param_or(info, "baud", 115200.0));
  max_rpm_ = param_or(info, "max_rpm", 58.0);
  counts_per_rev_ = param_or(info, "counts_per_rev", 8896.0);
  command_timeout_ms_ = static_cast<int>(param_or(info, "command_timeout_ms", 300.0));
  feedback_timeout_s_ = param_or(info, "feedback_timeout_s", 0.5);
  shutdown_command_ = param_or(info, "shutdown_command", shutdown_command_.c_str());
  firmware_info_topic_ = param_or(info, "firmware_info_topic", firmware_info_topic_.c_str());

  if (info.joints.size() != 2) {
    RCLCPP_ERROR(logger(), "expected exactly 2 joints (left, right), got %zu", info.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto & j : info.joints) {
    bool has_vel_cmd = j.command_interfaces.size() == 1 &&
                       j.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY;
    bool has_states = j.state_interfaces.size() == 2;
    if (!has_vel_cmd || !has_states) {
      RCLCPP_ERROR(logger(),
        "joint '%s' must have one velocity command and position+velocity state interfaces",
        j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  left_.joint_name = param_or(info, "left_joint", info.joints[0].name.c_str());
  right_.joint_name = param_or(info, "right_joint", info.joints[1].name.c_str());

  RCLCPP_INFO(logger(), "device=%s baud=%d max_rpm=%.1f counts_per_rev=%.0f left=%s right=%s",
    device_.c_str(), baud_, max_rpm_, counts_per_rev_, left_.joint_name.c_str(), right_.joint_name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MowerSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!port_.open(device_, baud_)) {
    RCLCPP_ERROR(logger(), "serial open failed: %s", port_.last_error().c_str());
    return CallbackReturn::ERROR;
  }
  left_.have_counts = right_.have_counts = false;
  left_.pos = right_.pos = 0.0;
  feedback_valid_ = false;
  warned_timeout_ = false;
  shutdown_acked_ = false;
  firmware_info_valid_ = false;

  // Publishing needs no executor, so a plain node inside the hardware
  // component is enough. transient_local keeps the last message for late
  // subscribers (the info aggregator, rosbridge).
  if (!info_node_ && !firmware_info_topic_.empty()) {
    try {
      info_node_ = std::make_shared<rclcpp::Node>("mower_hardware_info");
      firmware_info_pub_ = info_node_->create_publisher<std_msgs::msg::String>(
        firmware_info_topic_, rclcpp::QoS(1).transient_local().reliable());
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "firmware_info publisher unavailable: %s", e.what());
      info_node_.reset();
      firmware_info_pub_.reset();
    }
  }

  RCLCPP_INFO(logger(), "opened %s", device_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MowerSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  port_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MowerSystem::on_activate(const rclcpp_lifecycle::State &)
{
  left_.cmd_velocity = right_.cmd_velocity = 0.0;
  send_stop();
  // The firmware also sends 0x87 unsolicited every second; asking makes the
  // version show up in the log right away.
  auto f = build_info_request(tx_seq_++);
  port_.write_all(f.data(), f.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MowerSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  send_stop();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MowerSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> s;
  s.emplace_back(left_.joint_name, hardware_interface::HW_IF_POSITION, &left_.pos);
  s.emplace_back(left_.joint_name, hardware_interface::HW_IF_VELOCITY, &left_.vel);
  s.emplace_back(right_.joint_name, hardware_interface::HW_IF_POSITION, &right_.pos);
  s.emplace_back(right_.joint_name, hardware_interface::HW_IF_VELOCITY, &right_.vel);
  // diagnostics (read with a generic "gpio"/sensor broadcaster or just for debugging)
  s.emplace_back("mower_base", "wheel_flags", &diag_flags_);
  s.emplace_back("mower_base", "motor_flags", &diag_motor_flags_);
  s.emplace_back("mower_base", "command_age_ms", &diag_command_age_ms_);
  s.emplace_back("mower_base", "crc_errors", &diag_crc_errors_);
  s.emplace_back("mower_base", "feedback_age_s", &diag_feedback_age_s_);
  s.emplace_back("mower_base", "power_state", &diag_power_state_);
  s.emplace_back("mower_base", "power_flags", &diag_power_flags_);
  s.emplace_back("mower_base", "firmware_version", &diag_fw_version_);
  s.emplace_back("mower_base", "firmware_git_sha32", &diag_fw_git_sha32_);
  s.emplace_back("mower_base", "firmware_protocol", &diag_fw_protocol_);
  return s;
}

std::vector<hardware_interface::CommandInterface> MowerSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> c;
  c.emplace_back(left_.joint_name, hardware_interface::HW_IF_VELOCITY, &left_.cmd_velocity);
  c.emplace_back(right_.joint_name, hardware_interface::HW_IF_VELOCITY, &right_.cmd_velocity);
  return c;
}

void MowerSystem::handle_frame(uint8_t type, uint8_t seq, const uint8_t * payload, size_t len,
  const rclcpp::Time & now)
{
  if (type == kWheelFeedbackStatus) {
    WheelFeedback fb;
    if (!decode_wheel_feedback(seq, payload, len, fb)) {
      return;
    }
    const double rad_per_count = kTwoPi / counts_per_rev_;
    auto integrate = [&](Wheel & w, int32_t total, double rpm) {
      if (w.have_counts) {
        // signed 32-bit difference handles wrap-around
        int32_t d = static_cast<int32_t>(static_cast<uint32_t>(total) - static_cast<uint32_t>(w.last_total_counts));
        w.pos += d * rad_per_count;
      }
      w.last_total_counts = total;
      w.have_counts = true;
      w.vel = rpm * kTwoPi / 60.0;
    };
    integrate(left_, fb.left_total_counts, fb.left_measured_rpm);
    integrate(right_, fb.right_total_counts, fb.right_measured_rpm);
    diag_flags_ = fb.flags;
    last_feedback_time_ = now;
    feedback_valid_ = true;
    if (warned_timeout_) {
      RCLCPP_INFO(logger(), "feedback resumed");
      warned_timeout_ = false;
    }
  } else if (type == kMotorStatus) {
    MotorStatus ms;
    if (decode_motor_status(payload, len, ms)) {
      diag_motor_flags_ = ms.flags;
      diag_command_age_ms_ = ms.command_age_ms;
      if (ms.flags & kStatusFlagDriverAlarm) {
        RCLCPP_WARN_THROTTLE(logger(), throttle_clock_, 2000, "driver alarm flag set");
      }
    }
  } else if (type == kPowerStatus) {
    PowerStatus ps;
    if (decode_power_status(payload, len, ps)) {
      on_power_status(ps);
    }
  } else if (type == kFirmwareInfo) {
    FirmwareInfo fi;
    if (decode_firmware_info(payload, len, fi)) {
      on_firmware_info(fi);
    }
  }
}

void MowerSystem::on_firmware_info(const FirmwareInfo & fi)
{
  if (firmware_info_valid_ && fi == firmware_info_) {
    return;
  }
  firmware_info_ = fi;
  firmware_info_valid_ = true;
  diag_fw_version_ = fi.major * 1000000.0 + fi.minor * 1000.0 + fi.patch;
  diag_fw_git_sha32_ = fi.git_sha32;
  diag_fw_protocol_ = fi.protocol_version;

  RCLCPP_INFO(logger(), "STM32 firmware %s (protocol %u, built %u)",
    fi.version_string().c_str(), fi.protocol_version, fi.build_unix);
  if (fi.protocol_version != kProtocolVersion) {
    RCLCPP_ERROR(logger(), "firmware speaks protocol %u, this driver expects %u",
      fi.protocol_version, kProtocolVersion);
  }
  if (firmware_info_pub_) {
    std_msgs::msg::String msg;
    msg.data = fi.to_json();
    firmware_info_pub_->publish(msg);
  }
}

void MowerSystem::on_power_status(const PowerStatus & ps)
{
  diag_power_state_ = ps.state;
  diag_power_flags_ = ps.flags;

  if (!ps.shutdown_requested()) {
    shutdown_acked_ = false;  // request cleared (cancelled or rail cut)
    return;
  }
  if (shutdown_acked_) {
    return;
  }
  shutdown_acked_ = true;

  RCLCPP_WARN(logger(), "STM32 requests shutdown (reason %u): acking and running '%s'",
    ps.shutdown_reason, shutdown_command_.c_str());
  auto f = build_power_command(tx_seq_++, kPowerActionHostShutdownAck);
  if (!port_.write_all(f.data(), f.size())) {
    RCLCPP_ERROR(logger(), "power ack write failed: %s", port_.last_error().c_str());
  }
  if (!shutdown_command_.empty()) {
    // Detached so the control loop keeps acking status frames while the OS halts.
    std::string cmd = shutdown_command_ + " &";
    int rc = std::system(cmd.c_str());
    if (rc != 0) {
      RCLCPP_ERROR(logger(), "shutdown command returned %d", rc);
    }
  }
}

return_type MowerSystem::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  uint8_t buf[512];
  for (;;) {
    int n = port_.read_some(buf, sizeof(buf));
    if (n < 0) {
      RCLCPP_ERROR_THROTTLE(logger(), throttle_clock_, 2000, "serial read error: %s",
        port_.last_error().c_str());
      return return_type::ERROR;
    }
    if (n == 0) {
      break;
    }
    parser_.feed(buf, static_cast<size_t>(n),
      [&](uint8_t t, uint8_t s, const uint8_t * p, size_t l) { handle_frame(t, s, p, l, time); });
  }
  diag_crc_errors_ = static_cast<double>(parser_.crc_errors());

  if (feedback_valid_) {
    diag_feedback_age_s_ = (time - last_feedback_time_).seconds();
    if (diag_feedback_age_s_ > feedback_timeout_s_) {
      left_.vel = right_.vel = 0.0;
      if (!warned_timeout_) {
        RCLCPP_WARN(logger(), "no wheel feedback for %.2f s", diag_feedback_age_s_);
        warned_timeout_ = true;
      }
    }
  }
  return return_type::OK;
}

int16_t MowerSystem::rad_s_to_permille(double rad_s) const
{
  double rpm = rad_s * 60.0 / kTwoPi;
  double permille = rpm / max_rpm_ * 1000.0;
  permille = std::clamp(permille, -1000.0, 1000.0);
  return static_cast<int16_t>(std::lround(permille));
}

bool MowerSystem::send_stop()
{
  auto f = build_wheel_speed_command(tx_seq_++, 0, 0, static_cast<uint16_t>(command_timeout_ms_));
  return port_.write_all(f.data(), f.size());
}

return_type MowerSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  int16_t l = rad_s_to_permille(left_.cmd_velocity);
  int16_t r = rad_s_to_permille(right_.cmd_velocity);
  auto f = build_wheel_speed_command(tx_seq_++, l, r, static_cast<uint16_t>(command_timeout_ms_));
  if (!port_.write_all(f.data(), f.size())) {
    RCLCPP_ERROR_THROTTLE(logger(), throttle_clock_, 2000, "serial write error: %s",
      port_.last_error().c_str());
    return return_type::ERROR;
  }
  return return_type::OK;
}

}  // namespace mower_hardware

PLUGINLIB_EXPORT_CLASS(mower_hardware::MowerSystem, hardware_interface::SystemInterface)
