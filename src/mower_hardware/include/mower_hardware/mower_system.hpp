// ros2_control SystemInterface for the mower STM32 base over UART.
//
// Exposes two wheel joints with velocity command and position/velocity state
// interfaces (what diff_drive_controller needs), plus a few read-only
// diagnostics as extra state interfaces on a "base" sensor-like name.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mower_hardware/mower_protocol.hpp"
#include "mower_hardware/serial_port.hpp"

namespace mower_hardware {

class MowerSystem : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MowerSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Wheel {
    std::string joint_name;
    double cmd_velocity = 0.0;   // rad/s, from controller
    double pos = 0.0;            // rad, integrated from total counts
    double vel = 0.0;            // rad/s, from measured rpm
    int32_t last_total_counts = 0;
    bool have_counts = false;
  };

  // parameters
  std::string device_;
  int baud_ = 115200;
  double max_rpm_ = 58.0;              // firmware wheel_max_rpm; 1000 permille = this
  double counts_per_rev_ = 8896.0;     // FT-555 16 PPR x4 x 139:1
  int command_timeout_ms_ = 300;
  double feedback_timeout_s_ = 0.5;    // declare failure if no 0x85 for this long
  // Run when the STM32 reports SHUTDOWN_REQUESTED (button held 3 s). Empty
  // string disables; the ack is still sent so the STM32 does not wait 30 s.
  std::string shutdown_command_ = "systemctl poweroff";

  Wheel left_;
  Wheel right_;

  // diagnostics exported as state interfaces
  double diag_flags_ = 0.0;          // 0x85 flags
  double diag_motor_flags_ = 0.0;    // 0x81 flags
  double diag_command_age_ms_ = 0.0;
  double diag_crc_errors_ = 0.0;
  double diag_feedback_age_s_ = 0.0;
  double diag_power_state_ = 0.0;    // 0x86 state
  double diag_power_flags_ = 0.0;    // 0x86 flags

  SerialPort port_;
  FrameParser parser_;
  uint8_t tx_seq_ = 0;
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
  bool feedback_valid_ = false;
  bool warned_timeout_ = false;
  bool shutdown_acked_ = false;        // ack + command already sent for this request
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};  // for *_THROTTLE log macros

  void handle_frame(uint8_t type, uint8_t seq, const uint8_t * payload, size_t len, const rclcpp::Time & now);
  int16_t rad_s_to_permille(double rad_s) const;
  bool send_stop();
  void on_power_status(const PowerStatus & ps);
  rclcpp::Logger logger() const { return rclcpp::get_logger("MowerSystem"); }
};

}  // namespace mower_hardware
