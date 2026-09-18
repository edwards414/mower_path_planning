// ros2_control SystemInterface for the mower STM32 base over UART.
//
// Exposes two wheel joints with velocity command and position/velocity state
// interfaces (what diff_drive_controller needs), plus a few read-only
// diagnostics as extra state interfaces on a "base" sensor-like name.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"

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
  ~MowerSystem() override;

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
  double diag_fw_version_ = 0.0;     // 0x87 major*1e6 + minor*1e3 + patch, 0 = not seen yet
  double diag_fw_git_sha32_ = 0.0;   // 0x87 git_sha32
  double diag_fw_protocol_ = 0.0;    // 0x87 protocol_version

  // 0x87 build identity; published latched on /mower_base/firmware_info as
  // JSON so /robot/info (and the app) can show which firmware is running.
  FirmwareInfo firmware_info_;
  bool firmware_info_valid_ = false;
  rclcpp::Node::SharedPtr info_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr firmware_info_pub_;
  std::string firmware_info_topic_ = "/mower_base/firmware_info";

  // Raw STM32 status frames, republished as one JSON message on
  // /mower_base/telemetry at telemetry_rate_hz (0 disables) so the parameter
  // dashboard can plot wheel target vs. measured RPM, PID output / gains,
  // light state, power state, charger (0x89) and battery/analog readings
  // (0x8A, consumed by mower_mission battery_state_node) without touching
  // ros2_control. 20 Hz matches
  // the 0x85 period, which the PID auto-tune needs to sample a step response.
  std::string telemetry_topic_ = "/mower_base/telemetry";
  double telemetry_rate_hz_ = 20.0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
  rclcpp::Time telemetry_sent_time_{0, 0, RCL_ROS_TIME};
  WheelFeedback last_wheel_feedback_;
  MotorStatus last_motor_status_;
  PidConfigStatus last_pid_config_;
  Ws2812Status last_ws2812_status_;
  PowerStatus last_power_status_;
  ChargerStatus last_charger_status_;
  AnalogStatus last_analog_status_;
  bool have_motor_status_ = false;
  bool have_pid_config_ = false;
  bool have_ws2812_status_ = false;
  bool have_power_status_ = false;
  bool have_charger_status_ = false;
  bool have_analog_status_ = false;
  void publish_telemetry_if_due(const rclcpp::Time & now);
  std::string telemetry_json(const rclcpp::Time & now) const;

  // WS2812 light request (0x03), from the latched JSON topic
  //   {"mode":6,"r":255,"g":180,"b":0,"period_ms":1600}
  // The helper node is spun on its own thread; the callback packs the
  // request into one atomic word that write() picks up on the control
  // thread, so the real-time loop never blocks on ROS.
  std::string led_topic_ = "/mower_base/led_command";
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr led_sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> node_executor_;
  std::thread node_thread_;
  std::atomic<uint64_t> led_request_{0};  // 0 = nothing requested yet
  uint64_t led_sent_ = 0;                 // last request written to the port
  rclcpp::Time led_sent_time_{0, 0, RCL_ROS_TIME};
  static constexpr double kLedResendPeriodS = 5.0;  // re-assert in case a frame was lost
  static uint64_t pack_led(uint8_t mode, uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms, uint8_t serial);
  void on_led_command(const std_msgs::msg::String & msg);
  void send_led_if_needed(const rclcpp::Time & now);
  void stop_node_thread();

  // PID gains request (0x04), JSON on a reliable topic:
  //   {"left":{"kp":2,"ki":0.6,"kd":0},"right":{...},"persist":0,"closed_loop":1}
  // Sent once per message; the STM32 answers with 0x84 (LAST_APPLY_OK /
  // LAST_SAVE_OK in the telemetry pid.flags). Used by the PID auto-tune node.
  std::string pid_topic_ = "/mower_base/pid_command";
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pid_sub_;
  std::mutex pid_mutex_;
  PidConfig pid_request_;
  std::atomic<uint32_t> pid_serial_{0};   // bumped per request
  uint32_t pid_sent_serial_ = 0;
  void on_pid_command(const std_msgs::msg::String & msg);
  void send_pid_if_needed();

  // Raw wheel command override, JSON:
  //   {"left_permille":400,"right_permille":400,"ttl_ms":300}
  // While the ttl has not expired write() sends these permille instead of
  // the controller's velocity command, bypassing diff_drive_controller's
  // acceleration limits so an auto-tune step is a real step. ttl is clamped
  // to kOverrideMaxTtlMs; the requester has to keep re-publishing.
  std::string override_topic_ = "/mower_base/wheel_override";
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr override_sub_;
  static constexpr long kOverrideMaxTtlMs = 1000;
  std::atomic<uint64_t> override_request_{0};  // packed: [63:32] ttl_ms, [31:16] left, [15:0] right
  std::atomic<uint32_t> override_serial_{0};
  uint32_t override_seen_serial_ = 0;
  rclcpp::Time override_until_{0, 0, RCL_ROS_TIME};
  int16_t override_left_ = 0;
  int16_t override_right_ = 0;
  bool override_active_ = false;
  void on_wheel_override(const std_msgs::msg::String & msg);
  bool override_permille(const rclcpp::Time & now, int16_t & left, int16_t & right);

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
  void on_firmware_info(const FirmwareInfo & fi);
  rclcpp::Logger logger() const { return rclcpp::get_logger("MowerSystem"); }
};

}  // namespace mower_hardware
