// Frame builder / parser for the STM32 mower UART protocol
// (see UART_OPEN_LOOP_PROTOCOL.md at the repo root). No ROS dependencies.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace mower_hardware {

constexpr uint8_t kSof0 = 0xA5;
constexpr uint8_t kSof1 = 0x5A;
constexpr uint8_t kProtocolVersion = 0x01;
constexpr size_t kFrameOverhead = 8;  // sof0 sof1 ver type seq len ... crc_lo crc_hi

enum FrameType : uint8_t {
  kWheelSpeedCommand = 0x01,
  kLawerMotorCommand = 0x02,
  kWs2812Command = 0x03,
  kPidConfigCommand = 0x04,
  kMotorStatus = 0x81,
  kLawerMotorStatus = 0x82,
  kWs2812Status = 0x83,
  kPidConfigStatus = 0x84,
  kWheelFeedbackStatus = 0x85,
};

// 0x81 payload flags
constexpr uint8_t kStatusFlagCommandValid = 0x01;
constexpr uint8_t kStatusFlagCommandTimeout = 0x02;
constexpr uint8_t kStatusFlagDriverAlarm = 0x04;

// 0x85 payload flags
constexpr uint8_t kWheelFlagOutputEnabled = 0x01;
constexpr uint8_t kWheelFlagClosedLoop = 0x02;

struct WheelFeedback {
  double left_target_rpm = 0.0;
  double left_measured_rpm = 0.0;
  double right_target_rpm = 0.0;
  double right_measured_rpm = 0.0;
  int16_t left_pid_output = 0;
  int16_t right_pid_output = 0;
  int32_t left_total_counts = 0;
  int32_t right_total_counts = 0;
  uint8_t flags = 0;
  uint8_t seq = 0;
};

struct MotorStatus {
  int16_t commanded_left_permille = 0;
  int16_t commanded_right_permille = 0;
  int16_t applied_left_pwm = 0;
  int16_t applied_right_pwm = 0;
  uint16_t command_age_ms = 0;
  uint8_t flags = 0;
  uint8_t last_rx_seq = 0;
};

uint16_t crc16_ccitt_false(const uint8_t * data, size_t len);

// Build a complete frame (SOF + header + payload + CRC).
std::vector<uint8_t> build_frame(uint8_t type, uint8_t seq, const uint8_t * payload, size_t len);

std::vector<uint8_t> build_wheel_speed_command(
  uint8_t seq, int16_t left_permille, int16_t right_permille, uint16_t timeout_ms);

std::vector<uint8_t> build_lawer_motor_command(
  uint8_t seq, int16_t permille, uint16_t timeout_ms);

// Incremental parser: feed bytes, get callbacks for every CRC-valid frame.
class FrameParser {
public:
  using Callback = std::function<void(uint8_t type, uint8_t seq, const uint8_t * payload, size_t len)>;

  void feed(const uint8_t * data, size_t len, const Callback & on_frame);
  size_t crc_errors() const { return crc_errors_; }

private:
  std::vector<uint8_t> buf_;
  size_t crc_errors_ = 0;
};

// Decoders return false when the payload length does not match.
bool decode_wheel_feedback(uint8_t seq, const uint8_t * payload, size_t len, WheelFeedback & out);
bool decode_motor_status(const uint8_t * payload, size_t len, MotorStatus & out);

}  // namespace mower_hardware
