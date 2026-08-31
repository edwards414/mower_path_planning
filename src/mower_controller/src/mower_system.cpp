
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
#include <stdexcept>
#include <thread>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

constexpr uint8_t kCommandValidMask = 0x01;
constexpr uint8_t kCommandTimeoutMask = 0x02;
constexpr uint8_t kDriverAlarmMask = 0x04;

int normalize_to_permille(double value, double max_abs_value) {
  if (!std::isfinite(value) || !std::isfinite(max_abs_value) ||
      max_abs_value <= 0.0) {
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
  hardware_fault_latched_ = true;
  // Set up the serial communication with STM32
  // Read parameters from URDF
  try {
    cfg_.device = info_.hardware_parameters.at("device");
    cfg_.baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
    cfg_.timeout = std::stoi(info_.hardware_parameters.at("timeout"));
    cfg_.left_wheel_name =
        info_.hardware_parameters.at("left_wheel_name");
    cfg_.right_wheel_name =
        info_.hardware_parameters.at("right_wheel_name");
    const bool supported_baud =
        cfg_.baud_rate == 9600 || cfg_.baud_rate == 19200 ||
        cfg_.baud_rate == 38400 || cfg_.baud_rate == 57600 ||
        cfg_.baud_rate == 115200 || cfg_.baud_rate == 230400;
    if (cfg_.device.empty() || cfg_.left_wheel_name.empty() ||
        cfg_.right_wheel_name.empty() || !supported_baud ||
        cfg_.timeout < 10 || cfg_.timeout > 5000) {
      throw std::invalid_argument("invalid mower hardware parameter value");
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Invalid mower hardware configuration: %s",
                 error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
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
  hardware_fault_latched_ = true;
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
  stm_comms_.setLedError();
  if (!stm_comms_.setMowerBladeValue(0)) {
    RCLCPP_ERROR(get_logger(), "Failed to send initial blade stop command");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto stop_sequence = stm_comms_.setMotorValues(0, 0);
  if (!stop_sequence.has_value()) {
    RCLCPP_ERROR(get_logger(), "Failed to send initial STM32 stop command");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto handshake_deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(
                                      std::max(250, cfg_.timeout));
  while (std::chrono::steady_clock::now() < handshake_deadline) {
    stm_comms_.poll();
    if (stm_comms_.motor_status_is_fresh_and_acknowledged(
            std::chrono::milliseconds(250), stop_sequence)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!stm_comms_.motor_status_is_fresh_and_acknowledged(
          std::chrono::milliseconds(250), stop_sequence)) {
    RCLCPP_ERROR(get_logger(),
                 "STM32 did not acknowledge the activation stop command");
    stm_comms_.setLedError();
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_left_.cmd = 0.0;
  wheel_left_.vel = 0.0;
  wheel_right_.cmd = 0.0;
  wheel_right_.vel = 0.0;
  mower_blade_cmd_ = 0.0;

  for (const auto &[name, /*descr*/ _] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }

  for (const auto &[name, /*descr*/ _] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }
  hardware_fault_latched_ = false;
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

  hardware_fault_latched_ = true;
  wheel_left_.cmd = 0.0;
  wheel_right_.cmd = 0.0;
  mower_blade_cmd_ = 0.0;
  stm_comms_.setLedError();
  const bool blade_stop_sent = stm_comms_.setMowerBladeValue(0);
  // Keep the motor stop as the final frame so its exact sequence remains the
  // one proved by 0x81 even after blade/LED handlers are implemented in MCU
  // firmware and begin consuming the shared wire sequence.
  const auto stop_sequence = stm_comms_.setMotorValues(0, 0);
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  if (!stop_sequence.has_value() || !blade_stop_sent) {
    RCLCPP_ERROR(get_logger(), "Failed to transmit deactivation stop");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(std::max(250, cfg_.timeout));
  while (std::chrono::steady_clock::now() < deadline) {
    stm_comms_.poll();
    if (stm_comms_.motor_status_is_fresh_and_acknowledged(
            std::chrono::milliseconds(250), stop_sequence)) {
      RCLCPP_INFO(get_logger(), "Successfully deactivated!");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_ERROR(get_logger(), "STM32 did not acknowledge deactivation stop");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type
MowerSystemHardware::read(const rclcpp::Time & /*time*/,
                          const rclcpp::Duration &period) {
  if (hardware_fault_latched_) {
    wheel_left_.vel = 0.0;
    wheel_right_.vel = 0.0;
    return hardware_interface::return_type::ERROR;
  }
  stm_comms_.poll();
  if (!stm_comms_.motor_status_is_fresh_and_acknowledged(
          std::chrono::milliseconds(250))) {
    wheel_left_.vel = 0.0;
    wheel_right_.vel = 0.0;
    wheel_left_.cmd = 0.0;
    wheel_right_.cmd = 0.0;
    mower_blade_cmd_ = 0.0;
    hardware_fault_latched_ = true;
    stm_comms_.setMotorValues(0, 0);
    stm_comms_.setMowerBladeValue(0);
    stm_comms_.setLedError();
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STM32 status/command acknowledgement is stale; hardware faulted");
    return hardware_interface::return_type::ERROR;
  }
  const auto motor_status = stm_comms_.getMotorStatus();
  if (motor_status.has_value()) {
    const bool command_valid =
        (motor_status->flags & kCommandValidMask) != 0U;
    const bool command_timeout =
        (motor_status->flags & kCommandTimeoutMask) != 0U;
    const bool driver_alarm =
        (motor_status->flags & kDriverAlarmMask) != 0U;

    if (!command_valid || command_timeout || driver_alarm) {
      wheel_left_.vel = 0.0;
      wheel_right_.vel = 0.0;
      wheel_left_.cmd = 0.0;
      wheel_right_.cmd = 0.0;
      hardware_fault_latched_ = true;
      stm_comms_.setMotorValues(0, 0);
      stm_comms_.setMowerBladeValue(0);
      stm_comms_.setLedError();
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STM32 reported unsafe motor flags=0x%02X; hardware fault latched",
          motor_status->flags);
      return hardware_interface::return_type::ERROR;
    } else {
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
      if (joint == cfg_.left_wheel_name)
        set_state(name, wheel_left_.vel);
      else if (joint == cfg_.right_wheel_name)
        set_state(name, wheel_right_.vel);
      else if (joint == "mower_joint")
        set_state(name, 0.0);
    } else if (descr.get_interface_name() ==
               hardware_interface::HW_IF_POSITION) {
      if (joint == cfg_.left_wheel_name)
        set_state(name, get_state(name) + period.seconds() * wheel_left_.vel);
      else if (joint == cfg_.right_wheel_name)
        set_state(name, get_state(name) + period.seconds() * wheel_right_.vel);
      else if (joint == "mower_joint")
        set_state(name, 0.0);
    } else if (descr.get_interface_name() == hardware_interface::HW_IF_EFFORT) {
      if (joint == "mower_joint")
        set_state(name, mower_blade_cmd_);
      else
        set_state(name, 0.0);
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
  if (hardware_fault_latched_ || !stm_comms_.is_connected()) {
    if (stm_comms_.is_connected()) {
      stm_comms_.setMotorValues(0, 0);
      stm_comms_.setMowerBladeValue(0);
    }
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
      // Production blade actuation is intentionally disabled. A standard
      // effort controller retains its last command when its publisher dies,
      // while this 50 Hz write loop would keep refreshing the MCU watchdog.
      // Do not consume this interface until a steady-clock freshness watchdog
      // and independent hardware interlock are implemented and tested.
      mower_blade_cmd_ = 0.0;
    }
  }
  if (!std::isfinite(wheel_left_.cmd) ||
      !std::isfinite(wheel_right_.cmd) ||
      !std::isfinite(mower_blade_cmd_)) {
    // Never allow NaN/Inf to reach round(), an integer conversion, or the
    // motor MCU. Stop every actuator and fault the hardware write so the
    // controller manager cannot silently continue on a malformed command.
    stm_comms_.setMotorValues(0, 0);
    stm_comms_.setMowerBladeValue(0);
    hardware_fault_latched_ = true;
    stm_comms_.setLedError();
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected non-finite actuator command; all motors forced to zero");
    return hardware_interface::return_type::ERROR;
  }
  const int left_permille =
      normalize_to_permille(wheel_left_.cmd, kMaxWheelCommandRadPerSec);
  const int right_permille =
      normalize_to_permille(wheel_right_.cmd, kMaxWheelCommandRadPerSec);
  const int blade_permille = 0;

  if (!stm_comms_.setMotorValues(left_permille, right_permille).has_value()) {
    hardware_fault_latched_ = true;
    return hardware_interface::return_type::ERROR;
  }

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
