#ifndef MOWER_CONTROLLER_STM_COMMS_HPP
#define MOWER_CONTROLLER_STM_COMMS_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <libserial/SerialPort.h>
#include <optional>
#include <string>
#include <vector>

constexpr double kMaxWheelCommandRadPerSec = 10.0;
constexpr double kMaxMowerBladeEffort = 100.0;
constexpr int16_t kCommandPermilleMax = 1000;
constexpr uint16_t kDefaultCommandTimeoutMs = 200;

class StmComms {
public:
  struct MotorStatus {
    uint8_t seq{0};
    int16_t commanded_left_permille{0};
    int16_t commanded_right_permille{0};
    int16_t applied_left_pwm{0};
    int16_t applied_right_pwm{0};
    uint16_t command_age_ms{0};
    uint8_t flags{0};
    uint8_t last_rx_seq{0};
  };

  StmComms() = default;

  StmComms(const std::string &serial_device, int32_t baud_rate,
           int32_t timeout_ms);

  void setup(const std::string &serial_device, int32_t baud_rate,
             int32_t timeout_ms);
  std::optional<uint8_t>
  setMotorValues(int left_permille, int right_permille,
                 uint16_t command_timeout_ms = kDefaultCommandTimeoutMs);
  bool setMowerBladeValue(
      int permille, uint16_t command_timeout_ms = kDefaultCommandTimeoutMs);
  void setWs2812Mode(uint8_t mode, uint8_t red, uint8_t green, uint8_t blue,
                     uint16_t effect_period_ms);
  bool poll();
  std::optional<MotorStatus> getMotorStatus() const;
  bool is_connected() const;
  bool motor_status_is_fresh_and_acknowledged(
      std::chrono::milliseconds max_age,
      std::optional<uint8_t> expected_sequence = std::nullopt) const;
  void setLedOK();
  void setLedError();

private:
  struct MotorCommandRecord {
    uint8_t seq{0};
    int16_t left_permille{0};
    int16_t right_permille{0};
    uint16_t timeout_ms{0};
    std::chrono::steady_clock::time_point sent_at{};
  };

  enum class FrameType : uint8_t {
    kMotorOpenLoopCommand = 0x01,
    kMowerBladeOpenLoopCommand = 0x02,
    kWs2812Command = 0x03,
    kMotorStatus = 0x81,
  };

  enum class Ws2812Mode : uint8_t {
    kClear = 0x00,
    kAllOn = 0x01,
    kFlow = 0x02,
    kTurnLeft = 0x03,
    kTurnRight = 0x04,
    kShow = 0x05,
  };

  static uint16_t compute_crc(const std::vector<uint8_t> &bytes);
  static void append_int16_le(std::vector<uint8_t> &payload, int16_t value);
  static void append_uint16_le(std::vector<uint8_t> &payload, uint16_t value);
  static uint16_t read_uint16_le(const std::vector<uint8_t> &payload,
                                 size_t offset);
  static int16_t read_int16_le(const std::vector<uint8_t> &payload,
                               size_t offset);

  std::optional<uint8_t>
  write_frame(FrameType type, const std::vector<uint8_t> &payload);
  void mark_io_fault(const char *operation, const std::exception &error);
  void read_incoming_bytes();
  bool process_incoming_frames();
  bool handle_frame(uint8_t type, uint8_t seq,
                    const std::vector<uint8_t> &payload);

  LibSerial::SerialPort serial_conn_;
  int32_t timeout_ms_{1000};
  uint8_t next_sequence_{0};
  std::vector<uint8_t> rx_buffer_;
  MotorStatus motor_status_{};
  bool has_motor_status_{false};
  bool io_fault_{false};
  std::chrono::steady_clock::time_point last_motor_status_at_{};
  bool has_motor_status_time_{false};
  std::chrono::steady_clock::time_point last_ack_progress_at_{};
  bool has_ack_progress_time_{false};
  std::optional<uint8_t> last_acknowledged_sequence_;
  std::deque<MotorCommandRecord> recent_motor_commands_;
};

#endif // MOWER_CONTROLLER_STM_COMMS_HPP
