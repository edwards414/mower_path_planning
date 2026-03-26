
#include "mower_controller/mower_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

constexpr uint8_t kCommandValidMask = 0x01;
constexpr uint8_t kCommandTimeoutMask = 0x02;
constexpr uint8_t kDriverAlarmMask = 0x04;

int normalize_to_permille(double value, double max_abs_value) {
  if (max_abs_value <= 0.0) {
    return 0;
  }

  const double scaled =
      std::round((value / max_abs_value) * static_cast<double>(kCommandPermilleMax));
  const double clamped =
      std::clamp(scaled, -static_cast<double>(kCommandPermilleMax),
                 static_cast<double>(kCommandPermilleMax));
  return static_cast<int>(clamped);
}

double permille_to_rad_per_sec(int16_t permille) {
  return (static_cast<double>(permille) /
          static_cast<double>(kCommandPermilleMax)) *
         kMaxWheelCommandRadPerSec;
}

} // namespace

hardware_interface::CallbackReturn MowerSystemHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Set up the serial communication with STM32
  // Read parameters from URDF
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  // Set up the wheels with their names
  wheel_left_.setup(cfg_.left_wheel_name);
  wheel_right_.setup(cfg_.right_wheel_name);

  RCLCPP_INFO(get_logger(),
              "========== MowerSystemHardware Configuration ==========");
  RCLCPP_INFO(get_logger(), "  device          : %s", cfg_.device.c_str());
  RCLCPP_INFO(get_logger(), "  baud_rate       : %d", cfg_.baud_rate);
  RCLCPP_INFO(get_logger(), "  timeout         : %d ms", cfg_.timeout);
  RCLCPP_INFO(get_logger(), "  left_wheel_name : %s",
              cfg_.left_wheel_name.c_str());
  RCLCPP_INFO(get_logger(), "  right_wheel_name: %s",
              cfg_.right_wheel_name.c_str());
  RCLCPP_INFO(get_logger(),
              "=======================================================");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MowerSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  try {
    stm_comms_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize STM32 serial link: %s",
                 e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!stm_comms_.is_connected()) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to the STM32");
    return hardware_interface::CallbackReturn::ERROR;
  }
  stm_comms_.setLedOK();
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  // command and state should be equal when starting
  // for (const auto & [name, descr] : joint_command_interfaces_)
  // {
  //   set_command(name, get_state(name));
  // }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MowerSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  stm_comms_.setMotorValues(0, 0);
  stm_comms_.setMowerBladeValue(0);
  stm_comms_.setLedError();
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MowerSystemHardware::read(const rclcpp::Time & /*time*/,
                          const rclcpp::Duration &period) {
  // Without encoder feedback, the latest accepted STM32 open-loop command is
  // the best state estimate available for ros2_control.
  wheel_left_.vel = wheel_left_.cmd;
  wheel_right_.vel = wheel_right_.cmd;

  stm_comms_.poll();
  const auto motor_status = stm_comms_.getMotorStatus();
  if (motor_status.has_value()) {
    const bool command_valid =
        (motor_status->flags & kCommandValidMask) != 0U;
    const bool command_timeout =
        (motor_status->flags & kCommandTimeoutMask) != 0U;
    const bool driver_alarm =
        (motor_status->flags & kDriverAlarmMask) != 0U;

    if (command_timeout || driver_alarm) {
      wheel_left_.vel = 0.0;
      wheel_right_.vel = 0.0;
    } else if (command_valid) {
      wheel_left_.vel =
          permille_to_rad_per_sec(motor_status->commanded_left_permille);
      wheel_right_.vel =
          permille_to_rad_per_sec(motor_status->commanded_right_permille);
    }

    if (command_timeout) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "STM32 motor command timeout, last_rx_seq=%u age=%u ms",
          motor_status->last_rx_seq, motor_status->command_age_ms);
    }
    if (driver_alarm) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "STM32 motor driver alarm, last_rx_seq=%u flags=0x%02X",
          motor_status->last_rx_seq, motor_status->flags);
    }
  }

  for (const auto &[name, descr] : joint_state_interfaces_) {
    const std::string joint = descr.get_prefix_name();

    if (descr.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      if (joint == wheel_left_.name)
        set_state(name, wheel_left_.vel);
      else if (joint == wheel_right_.name)
        set_state(name, wheel_right_.vel);
    } else if (descr.get_interface_name() ==
               hardware_interface::HW_IF_POSITION) {
      if (joint == wheel_left_.name)
        set_state(name, get_state(name) + period.seconds() * wheel_left_.vel);
      else if (joint == wheel_right_.name)
        set_state(name, get_state(name) + period.seconds() * wheel_right_.vel);
    } else if (descr.get_interface_name() == hardware_interface::HW_IF_EFFORT) {
      if (joint == "mower_joint")
        set_state(name, mower_blade_cmd_);
    }
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                       "Read states - left vel: %.3f rad/s, right vel: %.3f "
                       "rad/s, blade effort: %.1f",
                       wheel_left_.vel, wheel_right_.vel, mower_blade_cmd_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MowerSystemHardware::write(const rclcpp::Time & /*time*/,
                           const rclcpp::Duration & /*period*/) {
  if (!stm_comms_.is_connected()) {
    return hardware_interface::return_type::ERROR;
  }
  // Set the motor values
  for (const auto &[name, descr] : joint_command_interfaces_) {
    // Collect commands for sending to hardware
    if (name == wheel_left_.name) {
      wheel_left_.cmd = get_command(name);
    } else if (name == wheel_right_.name) {
      wheel_right_.cmd = get_command(name);
    } else if (name == "mower_joint/effort") {
      mower_blade_cmd_ = get_command(name);
    }
  }
  const int left_permille =
      normalize_to_permille(wheel_left_.cmd, kMaxWheelCommandRadPerSec);
  const int right_permille =
      normalize_to_permille(wheel_right_.cmd, kMaxWheelCommandRadPerSec);
  const int blade_permille =
      normalize_to_permille(mower_blade_cmd_, kMaxMowerBladeEffort);

  stm_comms_.setMotorValues(left_permille, right_permille);
  stm_comms_.setMowerBladeValue(blade_permille);

  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Write commands - left: %.3f rad/s (%d permille), right: %.3f rad/s (%d "
      "permille), blade: %.1f (%d permille)",
      wheel_left_.cmd, left_permille, wheel_right_.cmd, right_permille,
      mower_blade_cmd_, blade_permille);
  //  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s",
  //  ss.str().c_str());
  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MowerSystemHardware, hardware_interface::SystemInterface)
